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
    RealtimeBGM: host-side SSEQ renderer, wall-clock paced.
    Wraps the vendored SSEQPlayer (src/Sound/SSEQPlayer, WTFPL, CyberBotX/fincs).
    All public methods are thread-safe: control calls come from the emu thread,
    Render() from the audio thread.
*/
#ifndef BGMRENDERER_H
#define BGMRENDERER_H

#include <memory>
#include <vector>
#include "../types.h"

namespace melonDS::Sound
{

class BgmRenderer
{
public:
    BgmRenderer();
    ~BgmRenderer();

    // Load a sequence from raw blobs copied out of emulated main RAM.
    // mml: sequence data (what START_SEQ arg1 points at), mmlLen from the SSEQ header.
    // sbnk: whole SBNK file (what the bank pointer points at). swar[i]: whole SWAR files
    // referenced by the SBNK header (empty if slot unused). Both are parsed in place, not changed.
    // Returns false on parse failure.
    // The playing song is not affected (also on failure); Start() switches to the loaded one.
    bool Load(const u8* mml, u32 mmlLen, std::vector<u8>& sbnk, std::vector<u8> (&swar)[4]);

    // Start playback. If atTick > 0, the sequencer is first advanced silently to that
    // tick (SNDSharedWork tickCounter units: 48 ticks per quarter note) so the host
    // copy lines up with the driver copy that was already playing.
    // A start at atTick > 0 fades in over a few ms, since notes are already sounding there.
    void Start(u32 atTick);
    // Hands the playing song to the outgoing voice, which keeps rendering on top of whatever starts
    // next (crossfade): it plays out its queued fader steps at their 1x pace, then, unless that left
    // it silent, fades out over fadeMs of wall-clock time. Only one outgoing voice is kept.
    void Release(u32 fadeMs);
    void Stop();                 // Release() with the default fade, unload nothing
    void Kill();                 // immediate silence of both voices (reset, savestate load)
    void Pause(bool paused);     // pauses the current voice; the outgoing one plays out
    // Paused songs kept for their resume, like the driver keeps a paused player: Park() moves the
    // current voice (song data, fader glide, driver parameters) aside under key, paused with its notes
    // cut, and leaves no current song. Unpark() makes it current again and resumes it; a playing current
    // song crossfades out as on Release(). Returns false if nothing is parked under key. Parked voices
    // render nothing; the oldest is dropped beyond a few. Kill() keeps them.
    void Park(int key);
    bool Unpark(int key);
    void DropParked(int key);
    void DropAllParked();
    bool Active() const;         // has output: Playing() or the outgoing voice still sounding
    bool Playing() const;        // current song playing (not stopped, not finished)
    u32 Tick() const;            // current song's position, in Start() ticks

    // Driver-level controls, mirrored from sniffed SND commands.
    // SNDPlayer.extFader (PLAYER_PARAM offset 6), 0 = full. frames: emulated frames since the previous
    // value; the host glides there over that many frames of wall-clock time at 1x, so fades keep their
    // 1x shape while fast-forwarding. 0 applies it immediately.
    void SetExtFader(s16 driverDecibel, u32 frames = 0);
    void SetTempoRatio(u16 ratio256);           // SNDPlayer.tempo_ratio, 256 = 1.0
    void SetTrackFader(u16 trackMask, s16 driverDecibel);  // TRACK_PARAM offset 0xA
    void SetTrackPitch(u16 trackMask, s16 pitch);          // TRACK_PARAM offset 0xC
    void SetTrackPan(u16 trackMask, s8 pan);               // TRACK_PARAM offset 9
    // MUTE_TRACK, NitroSDK SNDSeqMute: 0 off, 1 no new notes, 2 also release held notes, 3 also stop them
    void SetTrackMute(u16 trackMask, int mode);
    void SetVariable(u8 index, s16 value);      // 0-15 PLAYER_LOCAL_VAR, 16-31 PLAYER_GLOBAL_VAR
    void SetMasterVolume(u8 vol127);            // MASTER_VOLUME
    // Channels the song's notes may use (the player's ALLOCATABLE_CHANNEL mask), 0xFFFF = all.
    // Applies to new notes; kept across Load().
    void SetChannelMask(u16 mask);

    // Output configuration. Called before Render() from the audio thread as needed.
    void SetOutputRate(double hz);
    // Speed of the emulated audio relative to the DS (TargetFPS / 59.8261 at 1x): the host plays
    // that much faster and higher too, so it lines up with the hardware path. 1.0 = true DS speed.
    void SetOutputSkew(double skew);
    // Sample interpolation, melonDS AudioInterpolation values (0 = none, like the DS).
    void SetInterpolation(int mode);

    // Writes `frames` interleaved stereo s16 frames at the output rate (does not mix).
    // Writes silence when inactive. Never blocks on the emu thread for long.
    void Render(s16* stereo, int frames);

private:
    struct Impl;
    std::unique_ptr<Impl> P;
};

}
#endif
