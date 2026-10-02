/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

/*
    RealtimeBGM: host-side SSEQ renderer, wall-clock paced. See BgmRenderer.h.
*/

#include <math.h>
#include <algorithm>
#include <mutex>
#include <vector>
#include <string.h>
#include <stdio.h>
#include "BgmRenderer.h"
#include "SSEQPlayer/Player.h"

namespace melonDS::Sound
{
namespace SP = SSEQPlayer;

namespace
{
// Sequence, bank and wave archives of the loaded song. The player, its tracks and channels
// keep raw pointers into this, so it stays at one heap address until the next Load().
struct Song
{
    SP::SSEQ Seq;
    SP::SBNK Bank;
    SP::SWAR WaveArc[4];
};

// Ticks before the seek target that are simulated with notes and envelopes; earlier ticks
// only run the sequence (like SND_SkipSeq), so notes held across the target are still sounding.
constexpr u32 SeekFullSimTicks = 768;

constexpr double FrameSeconds = 560190.0 / 33513982.0; // one emulated frame at 1x
constexpr int FaderQueueLen = 256;     // queued fader steps; beyond that the oldest is skipped
constexpr int SilentFader = -723;      // SND_CalcChannelVolume clamps the total decay at -723 (volume 0)
constexpr u32 DefaultReleaseMs = 250;
constexpr double FadeInSeconds = 0.04;
constexpr int RenderChunk = 128;       // frames between fader updates, below the 5.2 ms sequencer period
constexpr size_t MaxParked = 4;

// One rendered song: the player, the fader glide and an output gain ramp.
struct Voice
{
    struct FaderStep
    {
        s16 Target;
        double Seconds;
    };

    std::unique_ptr<SP::Player> Ply;
    std::shared_ptr<Song> Data;  // what Ply points into, kept alive while this voice plays
    double Fader = 0;            // extFader in effect, glides through Steps
    FaderStep Steps[FaderQueueLen];
    int StepFirst = 0, StepCount = 0;
    double StepLeft = 0;         // seconds left in Steps[StepFirst]
    double Gain = 1, GainRate = 0;  // output gain and its change per second
    double ReleaseSeconds = 0;   // outgoing voice: fade-out once the steps are played out

    void ResetFader(s16 value)
    {
        Fader = value;
        StepCount = 0;
    }

    void QueueFader(s16 target, u32 frames)
    {
        if (StepCount == FaderQueueLen)
        {
            Fader = Steps[StepFirst].Target;
            StepFirst = (StepFirst + 1) % FaderQueueLen;
            StepCount--;
            StepLeft = Steps[StepFirst].Seconds;
        }
        Steps[(StepFirst + StepCount) % FaderQueueLen] = {target, frames * FrameSeconds};
        if (StepCount++ == 0)
            StepLeft = frames * FrameSeconds;
    }

    void AdvanceFader(double dt)
    {
        while (StepCount)
        {
            const FaderStep& step = Steps[StepFirst];
            if (dt < StepLeft)
            {
                Fader += (step.Target - Fader) * dt / StepLeft;
                StepLeft -= dt;
                return;
            }
            Fader = step.Target;
            dt -= StepLeft;
            StepFirst = (StepFirst + 1) % FaderQueueLen;
            if (--StepCount)
                StepLeft = Steps[StepFirst].Seconds;
        }
    }

    void ApplyFader()
    {
        s16 val = (s16)lround(Fader);
        if (val == Ply->extFader)
            return;
        Ply->extFader = val;
        Ply->FlagTracks(0xFFFF, SP::TUF_VOL);
    }

    // Writes (does not mix) frames stereo frames.
    void Render(s16* buf, int frames, double rate)
    {
        ApplyFader();
        Ply->GenerateSamples(buf, frames);
        if (GainRate != 0 || Gain != 1)
        {
            double step = GainRate / rate;
            for (int i = 0; i < frames * 2; i += 2)
            {
                Gain = Gain + step < 0 ? 0 : Gain + step > 1 ? 1 : Gain + step;
                buf[i] = (s16)(buf[i] * Gain);
                buf[i + 1] = (s16)(buf[i + 1] * Gain);
            }
            if (Gain == 1 && GainRate > 0)
                GainRate = 0;
        }
        AdvanceFader(frames / rate);
    }

    // Outgoing voice: true once it is inaudible for good. Starts the fade-out when the fader
    // steps are played out without reaching silence.
    bool OutgoingDone()
    {
        if (Ply->Finished() || (Gain == 0 && GainRate < 0))
            return true;
        if (StepCount)
            return false;
        if (Fader <= SilentFader)
            return true;
        if (GainRate >= 0)
            GainRate = -1 / ReleaseSeconds;
        return false;
    }
};

// Driver-side parameters of the current song as last set by the game. The driver resets them when a
// sequence starts (InitPlayer/InitTrack), so Load() resets them; Start() applies them.
struct SongParams
{
    s16 ExtFader;
    u16 TempoRatio;
    s16 TrackFader[16];
    s16 TrackPitch[16];
    s8 TrackPan[16];
    u16 TrackMute;
    s16 Variables[32];
    u32 VariablesSet;
    u16 ChannelMask = 0xFFFF;  // survives Load()
};

struct ParkedVoice
{
    int Key;
    Voice V;
    SongParams Params;
};
}

struct BgmRenderer::Impl : SongParams
{
    // Guards Cur, Out, OutActive, Playing, OutputRate, OutputSkew and Interp, which Render() uses
    // from the audio thread. Loaded, Parked and the parameter copies are only touched by the emu thread.
    mutable std::mutex Lock;
    Voice Cur;                   // current song
    Voice Out;                   // outgoing song, fading out
    bool OutActive = false;
    bool Playing = false;
    double OutputRate = 48000;
    double OutputSkew = 1;
    SP::Interpolation Interp = SP::INTERPOLATION_NONE;

    std::shared_ptr<Song> Loaded;
    std::vector<ParkedVoice> Parked;  // oldest first
    u8 MasterVolume = 127; // hardware setting, survives Load()

    void ResetParams()
    {
        ExtFader = 0;
        TempoRatio = 256;
        memset(TrackFader, 0, sizeof(TrackFader));
        memset(TrackPitch, 0, sizeof(TrackPitch));
        memset(TrackPan, 0, sizeof(TrackPan));
        TrackMute = 0;
        VariablesSet = 0;
    }

    // Rate the players synthesise at: the output at 1/skew speed plays skew times faster and higher.
    double Rate() const { return OutputRate / OutputSkew; }

    std::unique_ptr<SP::Player> NewPlayer(double rate) const
    {
        auto ply = std::make_unique<SP::Player>();
        ply->SetSampleRate(rate);
        ply->outputVol = MasterVolume == 127 ? 128 : MasterVolume;
        ply->channelMask = ChannelMask;
        return ply;
    }

    // Under Lock, before ply is swapped in: output settings may have changed since NewPlayer().
    void Configure(SP::Player& ply) const
    {
        ply.SetSampleRate(Rate());
        ply.interpolation = Interp;
    }

    // Under Lock: the current song becomes the outgoing voice. Returns the previous outgoing voice,
    // to be freed outside the lock.
    Voice ReleaseCur(u32 fadeMs)
    {
        Voice old = std::move(Out);
        Out = std::move(Cur);
        Out.ReleaseSeconds = fadeMs / 1000.0;
        OutActive = true;
        return old;
    }

    void ApplyParams(SP::Player& ply) const
    {
        ply.extFader = ExtFader;
        ply.tempoRate = TempoRatio;
        memcpy(ply.trackExtFader, TrackFader, sizeof(TrackFader));
        memcpy(ply.trackExtPitch, TrackPitch, sizeof(TrackPitch));
        memcpy(ply.trackExtPan, TrackPan, sizeof(TrackPan));
        ply.trackMute = TrackMute;
        for (int i = 0; i < 32; i++)
            if (VariablesSet & (1u << i))
                ply.variables[i] = Variables[i];
    }
};

BgmRenderer::BgmRenderer() : P(std::make_unique<Impl>())
{
    P->ResetParams();
    P->Cur.Ply = P->NewPlayer(P->OutputRate);
}

BgmRenderer::~BgmRenderer() = default;

bool BgmRenderer::Load(const u8* mml, u32 mmlLen,
                       const u8* sbnk, u32 sbnkLen,
                       const u8* const swar[4], const u32 swarLen[4])
{
    // Parsed into Loaded only: the playing song is not touched, so it keeps playing if this fails.
    std::unique_ptr<Song> song;
    if (mml && mmlLen && sbnk && sbnkLen)
    {
        song = std::make_unique<Song>();
        try
        {
            // The MML is used as-is; the SSEQ header is not needed. Pad with END so a track
            // running off the end (or a clamped bad jump target, see Track.cpp DataAt) stops.
            song->Seq.data.assign(mml, mml + mmlLen);
            song->Seq.data.insert(song->Seq.data.end(), 16, 0xFF);
            song->Seq.bank = &song->Bank;

            std::vector<u8> buf(sbnk, sbnk + sbnkLen);
            SP::PseudoFile file;
            file.data = &buf;
            song->Bank.Read(file);

            for (int i = 0; i < 4; i++)
            {
                if (!swar[i] || !swarLen[i])
                    continue;
                buf.assign(swar[i], swar[i] + swarLen[i]);
                file.pos = 0;
                song->WaveArc[i].Read(file);
                song->Bank.waveArc[i] = &song->WaveArc[i];
            }
        }
        catch (const std::exception&)
        {
            song.reset();
        }
    }
    if (!song)
        return false;

    P->Loaded = std::move(song);
    P->ResetParams();
    return true;
}

void BgmRenderer::Start(u32 atTick)
{
    if (!P->Loaded)
        return;

    double rate;
    {
        std::lock_guard<std::mutex> lock(P->Lock);
        rate = P->Rate();
    }

    // Build and seek a fresh player outside the lock, then swap it in.
    auto ply = P->NewPlayer(rate);
    ply->Setup(&P->Loaded->Seq);
    P->ApplyParams(*ply);

    if (atTick > 0)
    {
        ply->skipNotes = true;
        while (ply->tickCounter + SeekFullSimTicks < atTick && !ply->seqEnded)
            ply->RunTick();
        ply->skipNotes = false;

        // Same per-period work as during playback, minus mixing. Bounded in case of tempo 0.
        double samplesPerClock = SP::SecondsPerClockCycle * rate;
        u32 maxPeriods = (atTick - ply->tickCounter + 1) * 240;
        for (u32 n = 0; n < maxPeriods && ply->tickCounter < atTick && !ply->seqEnded; n++)
        {
            ply->Timer(atTick);
            for (auto& chn : ply->channels)
                chn.SkipSamples(samplesPerClock);
        }
    }

    std::shared_ptr<Song> data = P->Loaded;
    {
        std::lock_guard<std::mutex> lock(P->Lock);
        P->Configure(*ply);
        std::swap(P->Cur.Ply, ply);
        std::swap(P->Cur.Data, data);
        P->Cur.ResetFader(P->ExtFader);
        P->Cur.Gain = atTick > 0 ? 0 : 1;
        P->Cur.GainRate = atTick > 0 ? 1 / FadeInSeconds : 0;
        P->Playing = true;
    }
}

void BgmRenderer::Release(u32 fadeMs)
{
    double rate;
    {
        std::lock_guard<std::mutex> lock(P->Lock);
        if (!P->Playing || P->Cur.Ply->Finished())
        {
            P->Playing = false;
            return;
        }
        rate = P->Rate();
    }

    auto ply = P->NewPlayer(rate);
    Voice old;
    {
        std::lock_guard<std::mutex> lock(P->Lock);
        P->Playing = false;
        P->Configure(*ply);
        old = P->ReleaseCur(fadeMs);
        P->Cur.Ply = std::move(ply);
        P->Cur.ResetFader(P->ExtFader);
        P->Cur.Gain = 1;
        P->Cur.GainRate = 0;
    }
    // a previous outgoing voice is cut here, outside the lock
}

void BgmRenderer::Stop()
{
    Release(DefaultReleaseMs);
}

void BgmRenderer::Kill()
{
    Voice old;
    {
        std::lock_guard<std::mutex> lock(P->Lock);
        P->Playing = false;
        P->OutActive = false;
        old = std::move(P->Out);
        for (auto& chn : P->Cur.Ply->channels)
            chn.Kill();
    }
}

void BgmRenderer::Pause(bool paused)
{
    std::lock_guard<std::mutex> lock(P->Lock);
    P->Cur.Ply->SetPaused(paused);
}

void BgmRenderer::Park(int key)
{
    double rate;
    {
        std::lock_guard<std::mutex> lock(P->Lock);
        if (!P->Playing || P->Cur.Ply->Finished())
            return;
        rate = P->Rate();
    }
    DropParked(key);

    ParkedVoice slot;
    slot.Key = key;
    slot.Params = *P;
    auto ply = P->NewPlayer(rate);
    {
        std::lock_guard<std::mutex> lock(P->Lock);
        P->Playing = false;
        P->Configure(*ply);
        slot.V = std::move(P->Cur);
        P->Cur.Ply = std::move(ply);
        P->Cur.ResetFader(P->ExtFader);
        P->Cur.Gain = 1;
        P->Cur.GainRate = 0;
    }

    // the driver releases the notes on pause; a parked voice renders nothing, so they end here
    slot.V.Ply->SetPaused(true);
    for (auto& chn : slot.V.Ply->channels)
        chn.Kill();
    // the fade the host still had queued is over in the driver; the resume continues from its target
    slot.V.ResetFader(slot.Params.ExtFader);
    slot.V.ApplyFader();
    P->Parked.push_back(std::move(slot));
    if (P->Parked.size() > MaxParked)
        P->Parked.erase(P->Parked.begin());
}

bool BgmRenderer::Unpark(int key)
{
    auto it = std::find_if(P->Parked.begin(), P->Parked.end(), [key](const ParkedVoice& v) { return v.Key == key; });
    if (it == P->Parked.end())
        return false;
    ParkedVoice slot = std::move(*it);
    P->Parked.erase(it);

    static_cast<SongParams&>(*P) = slot.Params;
    slot.V.Ply->outputVol = P->MasterVolume == 127 ? 128 : P->MasterVolume;
    slot.V.Ply->SetPaused(false);
    Voice old;  // freed outside the lock
    {
        std::lock_guard<std::mutex> lock(P->Lock);
        P->Configure(*slot.V.Ply);
        if (P->Playing && !P->Cur.Ply->Finished())
            old = P->ReleaseCur(DefaultReleaseMs);
        else
            old = std::move(P->Cur);
        P->Cur = std::move(slot.V);
        P->Playing = true;
    }
    return true;
}

void BgmRenderer::DropParked(int key)
{
    auto it = std::find_if(P->Parked.begin(), P->Parked.end(), [key](const ParkedVoice& v) { return v.Key == key; });
    if (it != P->Parked.end())
        P->Parked.erase(it);
}

void BgmRenderer::DropAllParked()
{
    P->Parked.clear();
}

bool BgmRenderer::Active() const
{
    std::lock_guard<std::mutex> lock(P->Lock);
    return (P->Playing && !P->Cur.Ply->Finished()) || P->OutActive;
}

bool BgmRenderer::Playing() const
{
    std::lock_guard<std::mutex> lock(P->Lock);
    return P->Playing && !P->Cur.Ply->Finished();
}

u32 BgmRenderer::Tick() const
{
    std::lock_guard<std::mutex> lock(P->Lock);
    return P->Cur.Ply->tickCounter;
}

// extFader is in the driver's centibel (0.1 dB) volume units and is added straight onto the
// channel attenuation, see Channel::UpdateVol and NitroSDK:
//   snd_seq.c:779-780       UpdateTrackChannel(): user_decay = DecibelSquare(track volume)
//                           + DecibelSquare(track volume2) + DecibelSquare(player volume);
//                           user_decay2 = track->extFader + player->extFader (each clamped >= -32768)
//   snd_exchannel.c:153-204 SND_ExChannelMain(): decay = DecibelSquare(velocity) + envelope
//                           + user_decay + user_decay2 (+ volume LFO); SND_CalcChannelVolume(decay)
//   snd_util.c:303-331      SND_CalcChannelVolume(): clamp decay to [-723, 0], VolumeTable[decay + 723]
//                           gives the 7-bit SOUNDxCNT volume, shift /2 /4 /16 below -60/-120/-240
// i.e. channel volume = VolumeTable(clamp(vel + env + vol + vol2 + mainvol + trackFader + extFader)).
void BgmRenderer::SetExtFader(s16 driverDecibel, u32 frames)
{
    P->ExtFader = driverDecibel;
    std::lock_guard<std::mutex> lock(P->Lock);
    if (frames)
    {
        P->Cur.QueueFader(driverDecibel, frames);
        return;
    }
    P->Cur.ResetFader(driverDecibel);
    P->Cur.ApplyFader();
}

void BgmRenderer::SetTempoRatio(u16 ratio256)
{
    P->TempoRatio = ratio256;
    std::lock_guard<std::mutex> lock(P->Lock);
    P->Cur.Ply->tempoRate = ratio256;
}

void BgmRenderer::SetTrackFader(u16 trackMask, s16 driverDecibel)
{
    for (int i = 0; i < 16; i++)
        if (trackMask & (1 << i))
            P->TrackFader[i] = driverDecibel;
    std::lock_guard<std::mutex> lock(P->Lock);
    memcpy(P->Cur.Ply->trackExtFader, P->TrackFader, sizeof(P->TrackFader));
    P->Cur.Ply->FlagTracks(trackMask, SP::TUF_VOL);
}

void BgmRenderer::SetTrackPitch(u16 trackMask, s16 pitch)
{
    for (int i = 0; i < 16; i++)
        if (trackMask & (1 << i))
            P->TrackPitch[i] = pitch;
    std::lock_guard<std::mutex> lock(P->Lock);
    memcpy(P->Cur.Ply->trackExtPitch, P->TrackPitch, sizeof(P->TrackPitch));
    P->Cur.Ply->FlagTracks(trackMask, SP::TUF_TIMER);
}

void BgmRenderer::SetTrackPan(u16 trackMask, s8 pan)
{
    for (int i = 0; i < 16; i++)
        if (trackMask & (1 << i))
            P->TrackPan[i] = pan;
    std::lock_guard<std::mutex> lock(P->Lock);
    memcpy(P->Cur.Ply->trackExtPan, P->TrackPan, sizeof(P->TrackPan));
    P->Cur.Ply->FlagTracks(trackMask, SP::TUF_PAN);
}

void BgmRenderer::SetTrackMute(u16 trackMask, int mode)
{
    if (mode)
        P->TrackMute |= trackMask;
    else
        P->TrackMute &= ~trackMask;
    std::lock_guard<std::mutex> lock(P->Lock);
    for (int i = 0; i < 16; i++)
        if (trackMask & (1 << i))
            P->Cur.Ply->SetTrackMute(i, mode);
}

void BgmRenderer::SetVariable(u8 index, s16 value)
{
    if (index >= 32)
        return;
    P->Variables[index] = value;
    P->VariablesSet |= 1u << index;
    std::lock_guard<std::mutex> lock(P->Lock);
    P->Cur.Ply->variables[index] = value;
}

void BgmRenderer::SetMasterVolume(u8 vol127)
{
    P->MasterVolume = vol127 & 0x7F;
    std::lock_guard<std::mutex> lock(P->Lock);
    // same as SPU::Write SOUNDCNT: 127 means unity
    P->Cur.Ply->outputVol = P->MasterVolume == 127 ? 128 : P->MasterVolume;
}

void BgmRenderer::SetChannelMask(u16 mask)
{
    P->ChannelMask = mask;
    std::lock_guard<std::mutex> lock(P->Lock);
    P->Cur.Ply->channelMask = mask;
}

void BgmRenderer::SetOutputRate(double hz)
{
    if (hz <= 0)
        return;
    std::lock_guard<std::mutex> lock(P->Lock);
    P->OutputRate = hz;
    P->Cur.Ply->SetSampleRate(P->Rate());
    if (P->Out.Ply)
        P->Out.Ply->SetSampleRate(P->Rate());
}

void BgmRenderer::SetOutputSkew(double skew)
{
    if (skew <= 0)
        return;
    std::lock_guard<std::mutex> lock(P->Lock);
    P->OutputSkew = skew;
    P->Cur.Ply->SetSampleRate(P->Rate());
    if (P->Out.Ply)
        P->Out.Ply->SetSampleRate(P->Rate());
}

void BgmRenderer::SetInterpolation(int mode)
{
    // melonDS AudioInterpolation: None, Linear, Cosine, Cubic, SNESGaussian
    static const SP::Interpolation map[] = {
        SP::INTERPOLATION_NONE, SP::INTERPOLATION_LINEAR, SP::INTERPOLATION_4POINTLEGRANGE,
        SP::INTERPOLATION_4POINTLEGRANGE, SP::INTERPOLATION_6POINTLEGRANGE,
    };
    SP::Interpolation interp = mode >= 0 && mode < 5 ? map[mode] : SP::INTERPOLATION_NONE;
    std::lock_guard<std::mutex> lock(P->Lock);
    P->Interp = interp;
    P->Cur.Ply->interpolation = interp;
    if (P->Out.Ply)
        P->Out.Ply->interpolation = interp;
}

void BgmRenderer::Render(s16* stereo, int frames)
{
    if (frames <= 0)
        return;
    std::lock_guard<std::mutex> lock(P->Lock);
    bool cur = P->Playing && !P->Cur.Ply->Finished();
    if (!cur && !P->OutActive)
    {
        memset(stereo, 0, frames * 2 * sizeof(s16));
        return;
    }

    s16 out[RenderChunk * 2];
    for (int pos = 0; pos < frames; pos += RenderChunk)
    {
        int n = frames - pos < RenderChunk ? frames - pos : RenderChunk;
        s16* buf = stereo + pos * 2;
        if (cur)
            P->Cur.Render(buf, n, P->Rate());
        else
            memset(buf, 0, n * 2 * sizeof(s16));

        if (!P->OutActive)
            continue;
        P->Out.Render(out, n, P->Rate());
        for (int i = 0; i < n * 2; i++)
        {
            s32 v = buf[i] + out[i];
            buf[i] = v < -0x8000 ? -0x8000 : v > 0x7FFF ? 0x7FFF : v;
        }
        // the voice itself is freed by the next Release() or Kill() on the emu thread
        if (P->Out.OutgoingDone())
            P->OutActive = false;
    }
}

}
