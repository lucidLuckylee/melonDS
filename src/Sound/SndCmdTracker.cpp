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
    RealtimeBGM: NitroSDK sound command tracker.
    See SndCmdTracker.h and bgm-realtime-feasibility.md sections 3, 5.1 and 6.
    Struct offsets below are from NitroSDK include/nitro/snd/common/{seq,exchannel,work,bank}.h
    with natural ARM alignment.
*/
#include <string.h>
#include "SndCmdTracker.h"
#include "../NDS.h"
#include "../CRC32.h"
#include "../Platform.h"
#include "../Savestate.h"

namespace melonDS::Sound
{

using Platform::Log;
using Platform::LogLevel;

namespace
{

enum : u32
{
    CMD_START_SEQ = 0, CMD_STOP_SEQ = 1, CMD_PREPARE_SEQ = 2, CMD_START_PREPARED_SEQ = 3, CMD_PAUSE_SEQ = 4,
    CMD_SKIP_SEQ = 5, CMD_PLAYER_PARAM = 6, CMD_TRACK_PARAM = 7, CMD_MUTE_TRACK = 8, CMD_ALLOCATABLE_CHANNEL = 9,
    CMD_PLAYER_LOCAL_VAR = 10, CMD_PLAYER_GLOBAL_VAR = 11, CMD_MASTER_VOLUME = 23, CMD_SHARED_WORK = 29, CMD_INVALIDATE_SEQ = 30,
    CMD_INVALIDATE_BANK = 31, CMD_READ_DRIVER_INFO = 33,
};

constexpr u32 MAGIC_SSEQ = 0x51455353, MAGIC_SBNK = 0x4B4E4253, MAGIC_SWAR = 0x52415753;

// SNDWork layout: SNDExChannel channel[16] (84 bytes each), SNDPlayer player[16] (36), SNDTrack track[32] (64),
// SNDAlarm alarm[8]. sizeof(SNDAlarm) depends on the compiler's u64 alignment (72 with 8-byte, 64 with 4-byte),
// so both SNDWork sizes are tried and the ARM7 copy is used to confirm.
constexpr u32 EXCH_SIZE = 84, PLAYER_SIZE = 36, TRACK_SIZE = 64;
constexpr u32 WORK_PLAYER_OFS = 16 * EXCH_SIZE;
constexpr u32 WORK_TRACK_OFS = WORK_PLAYER_OFS + 16 * PLAYER_SIZE;
constexpr u32 WORK_SIZES[2] = {WORK_TRACK_OFS + 32 * TRACK_SIZE + 8 * 72, WORK_TRACK_OFS + 32 * TRACK_SIZE + 8 * 64};
constexpr u32 EXCH_CALLBACK_DATA_OFS = 76;
constexpr u32 PLAYER_TRACKS_OFS = 8;

// Games fade by sending one PLAYER_PARAM extFader per frame; a change after a longer gap is not
// part of a fade and takes one frame.
constexpr u32 MAX_FADER_GAP = 4;

constexpr u32 MAX_MML = 0x100000;
constexpr u32 MAX_BLOB = 0x400000;

// SNDSharedWork.globalVariable[16], after the 0x20-byte header and 16 per-player blocks of 36 bytes
constexpr u32 SHARED_GLOBAL_VAR_OFS = 0x20 + 16 * 36;

constexpr u32 SAVESTATE_VERSION = 1;

bool IsMainRAM(u32 addr) { return (addr >> 24) == 0x02; }

u32 Get32(const u8* p) { u32 v; memcpy(&v, p, 4); return v; }

// SBNK CRC without SNDBankData.waveArcLink[4] (0x18-0x37): NNS rewrites those pointers as banks sharing
// a wave archive are loaded and freed, so they would make one bank look like another
u32 BankCRC32(const std::vector<u8>& sbnk)
{
    return CRC32(sbnk.data() + 0x38, (int)sbnk.size() - 0x38, CRC32(sbnk.data(), 0x18));
}

}

SndCmdTracker::SndCmdTracker(melonDS::NDS& nds) : NDS(nds)
{
    ChanOwner.fill(-1);
}

SndCmdTracker::~SndCmdTracker() = default;

void SndCmdTracker::Reset()
{
    Bgm.Kill();
    Bgm.DropAllParked();
    P = {};
    HostP = -1;
    MutedP = 0;
    NDS.SPU.ClearHostTags();
    CurMuteMask = 0;
    SharedWork = 0;
    DriverInfoAddr = 0;
    DriverInfoReq = 0;
    DriverInfoPending = false;
    DriverInfoLogged = -1;
    ChanOwner.fill(-1);
    ChanOwnerValid = false;
    memset(TrackMuteMode, 0, sizeof(TrackMuteMode));
    memset(TrackFader, 0, sizeof(TrackFader));
    memset(TrackPitch, 0, sizeof(TrackPitch));
    memset(TrackPan, 0, sizeof(TrackPan));
    StartOrder = {};
    StartFrame = {};
    NoHost = {};
    StartCounter = 0;
    MasterVol = 127;
}

void SndCmdTracker::OnCartChanged(const u8* rom, u32 romLen)
{
    Reset();
    Idx.Clear();
    IdxBuilt = false;
    Rom = rom;
    RomLen = romLen;
}

void SndCmdTracker::EnsureIndex()
{
    if (IdxBuilt || !Settings.Enabled || !Rom) return;
    IdxBuilt = true;

    Idx.Build(Rom, RomLen);
    Log(LogLevel::Info, "RealtimeBGM: index has %u sequences in %u SDATs\n",
        (u32)Idx.Sequences().size(), (u32)Idx.SdatNames().size());
    // players started while the feature was disabled
    std::vector<u8> bank;
    for (PlayerState& s : P)
    {
        if (s.MMLLen && (s.Info = Idx.Lookup(s.CRC, s.MMLLen)))
            s.Cls = s.Info->Cls;
        if (s.Active && !s.BankCRC && Eligible(s) && CopyBank(s.Bank, bank))
            s.BankCRC = BankCRC32(bank);
    }
}

u32 SndCmdTracker::RamRead32(u32 addr) const
{
    u32 v = 0;
    for (int i = 0; i < 4; i++)
        v |= (u32)NDS.MainRAM[(addr + i) & NDS.MainRAMMask] << (i * 8);
    return v;
}

bool SndCmdTracker::CopyRAM(u32 addr, u32 len, std::vector<u8>& out) const
{
    if (len == 0 || !IsMainRAM(addr) || !IsMainRAM(addr + len - 1)) return false;
    out.resize(len);
    u32 start = addr & NDS.MainRAMMask;
    if (start + len <= NDS.MainRAMMask + 1)
        memcpy(out.data(), &NDS.MainRAM[start], len);
    else
        for (u32 i = 0; i < len; i++)
            out[i] = NDS.MainRAM[(addr + i) & NDS.MainRAMMask];
    return true;
}

bool SndCmdTracker::CopyBank(u32 addr, std::vector<u8>& out) const
{
    if (!IsMainRAM(addr) || RamRead32(addr) != MAGIC_SBNK) return false;
    u32 size = RamRead32(addr + 8);
    return size >= 0x3C && size <= MAX_BLOB && CopyRAM(addr, size, out);
}

// Copies a loaded SWAR. Wave archives loaded wave-by-wave (NNS "single load") contain only the
// header and offset table, with absolute main RAM addresses for the loaded waves (see
// SND_GetWaveDataAddress); those are rebuilt into a self-contained SWAR with relative offsets.
bool SndCmdTracker::CopySwar(u32 addr, std::vector<u8>& out) const
{
    if (!IsMainRAM(addr) || RamRead32(addr) != MAGIC_SWAR) return false;
    u32 fsize = RamRead32(addr + 8);
    u32 count = RamRead32(addr + 0x38);
    if (fsize < 0x3C || fsize > MAX_BLOB || count > 0x10000) return false;

    bool absolute = false;
    for (u32 i = 0; i < count; i++)
        if (RamRead32(addr + 0x3C + i * 4) >= 0x02000000) absolute = true;
    if (!absolute)
        return CopyRAM(addr, fsize, out);

    u32 hdrLen = 0x3C + count * 4;
    if (!CopyRAM(addr, hdrLen, out)) return false;
    for (u32 i = 0; i < count; i++)
    {
        u32 ofs = RamRead32(addr + 0x3C + i * 4);
        u32 wave = ofs >= 0x02000000 ? ofs : (ofs ? addr + ofs : 0);
        u32 newOfs = 0;
        if (wave)
        {
            // SNDWaveParam: loopstart (u16 @6) and looplen (u32 @8) in words
            if (!IsMainRAM(wave)) return false;
            u32 len = 12 + 4 * ((RamRead32(wave + 4) >> 16) + RamRead32(wave + 8));
            std::vector<u8> w;
            if (len > MAX_BLOB || out.size() + len > MAX_BLOB || !CopyRAM(wave, len, w)) return false;
            newOfs = (u32)out.size();
            out.insert(out.end(), w.begin(), w.end());
        }
        memcpy(&out[0x3C + i * 4], &newOfs, 4);
    }
    u32 total = (u32)out.size();
    u32 blockSize = total - 0x10;
    memcpy(&out[8], &total, 4);
    memcpy(&out[0x14], &blockSize, 4);
    return true;
}

void SndCmdTracker::OnPxiWord(u32 word)
{
    if ((word & 0x1F) != 7) return;
    u32 node = word >> 6;
    if (node < 0x02000000) return;

    struct Cmd { u32 ID, Args[4]; };
    std::array<Cmd, 256> cmds;
    int n = 0;
    for (; n < 256 && IsMainRAM(node); n++)
    {
        cmds[n].ID = NDS.ARM7Read32(node + 4);
        for (int i = 0; i < 4; i++)
            cmds[n].Args[i] = NDS.ARM7Read32(node + 8 + i * 4);
        node = NDS.ARM7Read32(node);
    }

    for (int i = 0; i < n; i++)
    {
        const Cmd& c = cmds[i];
        // NNS sends ALLOCATABLE_CHANNEL (possibly several, for different track sets) right after
        // (PREPARE|START)_SEQ in the same list; apply them first so runtime classification sees
        // this start's channel mask.
        if ((c.ID == CMD_START_SEQ || c.ID == CMD_PREPARE_SEQ) && c.Args[0] < 16)
        {
            u16 mask = 0;
            for (int j = i + 1; j < n; j++)
            {
                if ((cmds[j].ID == CMD_START_SEQ || cmds[j].ID == CMD_PREPARE_SEQ) && cmds[j].Args[0] == c.Args[0]) break;
                if (cmds[j].ID == CMD_ALLOCATABLE_CHANNEL && cmds[j].Args[0] == c.Args[0])
                    mask |= (u16)cmds[j].Args[2];
            }
            if (mask) P[c.Args[0]].ChanMask = mask;
        }
        HandleCommand(c.ID, c.Args[0], c.Args[1], c.Args[2], c.Args[3]);
    }

    // a STOP of the host must not leave the other BGM players unmuted until the end of the frame
    if (Settings.Enabled && HostP < 0)
        PickHost();
}

void SndCmdTracker::HandleCommand(u32 id, u32 a0, u32 a1, u32 a2, u32 a3)
{
    switch (id)
    {
    case CMD_START_SEQ:
    case CMD_PREPARE_SEQ:
        if (a0 < 16) OnStart((int)a0, a1, a2, a3, id == CMD_PREPARE_SEQ);
        break;

    case CMD_START_PREPARED_SEQ:
        if (a0 < 16 && P[a0].Active && P[a0].Prepared)
        {
            P[a0].Prepared = false;
            StartFrame[a0] = FrameCount;
            if (Settings.Enabled && FF && Eligible(P[a0]))
                EnterHostMode((int)a0, true);
            UpdateMuteMask();
        }
        break;

    case CMD_STOP_SEQ:
        if (a0 < 16) OnStop((int)a0);
        break;

    case CMD_PAUSE_SEQ:
        if (a0 < 16)
        {
            int p = (int)a0;
            P[p].Paused = a1 != 0;
            // the driver keeps a paused player as it is (a battle theme plays on another player meanwhile),
            // and so does the host: the voice is parked, not dropped and rebuilt on the resume
            if (p == HostP && a1)
                ParkHost();
            else if (p == HostP)
            {
                Log(LogLevel::Debug, "RealtimeBGM: host player %d resumed\n", HostP);
                Bgm.Pause(false);
            }
            // a parked player resumed while another one is hosted waits for PickHost; without
            // fast-forward its parked voice is only the source while its hardware copy is muted
            else if (!a1 && P[p].Parked && Settings.Enabled && (FF || (MutedP & (1 << p))))
            {
                if (HostP >= 0 && P[HostP].Paused) ParkHost();
                if (HostP < 0) UnparkHost(p);
            }
        }
        break;

    case CMD_SKIP_SEQ:
        // the driver releases the notes and runs the sequence a1 ticks ahead; the host does the same
        // from its own position, which is behind the driver's after fast-forward
        if ((int)a0 == HostP)
        {
            Bgm.Start(Bgm.Tick() + a1);
            if (P[a0].Paused) Bgm.Pause(true);
        }
        // a parked voice cannot follow; the resume re-adopts the player from the driver's tick
        else if (a0 < 16 && P[a0].Parked)
        {
            Bgm.DropParked((int)a0);
            P[a0].Parked = false;
        }
        break;

    case CMD_PLAYER_PARAM:
        if (a0 < 16)
        {
            s32 val = a3 == 1 ? (s32)(s8)a2 : a3 == 2 ? (s32)(s16)a2 : (s32)a2;
            if (a1 == 6)
            {
                // the host replays fader changes at their 1x pace, which keeps fades at their 1x
                // length while fast-forwarding
                u32 gap = FrameCount - P[a0].FaderFrame;
                P[a0].ExtFader = (s16)val;
                P[a0].FaderFrame = FrameCount;
                if ((int)a0 == HostP) Bgm.SetExtFader((s16)val, gap >= 1 && gap <= MAX_FADER_GAP ? gap : 1);
            }
            else if (a1 == 0x1A)
            {
                P[a0].TempoRatio = (u16)val;
                if ((int)a0 == HostP) Bgm.SetTempoRatio((u16)val);
            }
        }
        break;

    case CMD_TRACK_PARAM:
        {
            u32 player = a0 & 0xFFFFFF, size = a0 >> 24;
            if (player >= 16) break;
            s32 val = size == 1 ? (s32)(s8)a3 : size == 2 ? (s32)(s16)a3 : (s32)a3;
            // remembered for every player so a host adopted mid-song starts with the real track levels
            for (int t = 0; t < 16; t++)
            {
                if (!(a1 & (1 << t))) continue;
                if (a2 == 0xA) TrackFader[player][t] = (s16)val;
                else if (a2 == 0xC) TrackPitch[player][t] = (s16)val;
                else if (a2 == 9) TrackPan[player][t] = (s8)val;
            }
            if ((int)player != HostP) break;
            if (a2 == 0xA) Bgm.SetTrackFader((u16)a1, (s16)val);
            else if (a2 == 0xC) Bgm.SetTrackPitch((u16)a1, (s16)val);
            else if (a2 == 9) Bgm.SetTrackPan((u16)a1, (s8)val);
        }
        break;

    case CMD_MUTE_TRACK:
        if (a0 < 16 && a2 <= 3)
        {
            for (int t = 0; t < 16; t++)
                if (a1 & (1 << t)) TrackMuteMode[a0][t] = (u8)a2;
            if ((int)a0 == HostP) Bgm.SetTrackMute((u16)a1, (int)a2);
        }
        break;

    case CMD_ALLOCATABLE_CHANNEL:
        if (a0 < 16)
        {
            P[a0].ChanMask |= (u16)a2;
            if ((int)a0 == HostP) Bgm.SetChannelMask(P[a0].ChanMask);
        }
        break;

    case CMD_PLAYER_LOCAL_VAR:
        if ((int)a0 == HostP && a1 < 16) Bgm.SetVariable((u8)a1, (s16)a2);
        break;

    case CMD_PLAYER_GLOBAL_VAR:
        // shared by all players; the renderer keeps them after the local ones
        if (HostP >= 0 && a0 < 16) Bgm.SetVariable((u8)(16 + a0), (s16)a1);
        break;

    case CMD_MASTER_VOLUME:
        MasterVol = (u8)a0;
        Bgm.SetMasterVolume(MasterVol);
        break;

    case CMD_INVALIDATE_SEQ:
    case CMD_INVALIDATE_BANK:
        // NNS frees a heap without STOP_SEQ; the driver finishes every player whose sequence
        // data (or bank) lies in the freed range
        for (int p = 0; p < 16; p++)
        {
            const PlayerState& s = P[p];
            if (!s.Active) continue;
            bool hit = id == CMD_INVALIDATE_SEQ ? (s.MML <= a1 && s.MML + s.MMLLen >= a0)
                                                : (s.Bank >= a0 && s.Bank <= a1);
            if (hit) OnStop(p);
        }
        break;

    case CMD_SHARED_WORK:
        SharedWork = IsMainRAM(a0) ? a0 : 0;
        break;

    case CMD_READ_DRIVER_INFO:
        // NNS double-buffers driver info and only requests a new snapshot once the previous
        // one completed, so the buffer requested before this one is complete and stable.
        if (a0 != DriverInfoReq)
        {
            DriverInfoAddr = DriverInfoReq;
            DriverInfoReq = a0;
        }
        DriverInfoPending = DriverInfoAddr != 0;
        break;
    }
}

bool SndCmdTracker::Eligible(const PlayerState& s) const
{
    switch (s.Cls)
    {
    case SeqClass::BGM: return true;
    case SeqClass::Jingle: return Settings.JinglesAt1x;
    case SeqClass::Ambient: return Settings.AmbientAt1x;
    default: return false;
    }
}

void SndCmdTracker::OnStart(int player, u32 mml, u32 offset, u32 bank, bool prepareOnly)
{
    if (player == HostP) LeaveHostMode();
    if (P[player].Parked) Bgm.DropParked(player);
    // the driver stops the old sequence; the new one's notes are tagged at key-on. The old one's
    // release tails stay silent: they would sound at the fast-forwarded pitch.
    if (MutedP & (1 << player)) NDS.SPU.RetireHostTags(player);
    MutedP &= ~(1 << player);
    EnsureIndex();

    PlayerState& s = P[player];
    u16 chanMask = s.ChanMask;
    s = PlayerState();
    s.ChanMask = chanMask;
    s.Active = true;
    s.Prepared = prepareOnly;
    s.MML = mml;
    s.Bank = bank;
    memset(TrackMuteMode[player], 0, sizeof(TrackMuteMode[player]));
    memset(TrackFader[player], 0, sizeof(TrackFader[player]));
    memset(TrackPitch[player], 0, sizeof(TrackPitch[player]));
    memset(TrackPan[player], 0, sizeof(TrackPan[player]));
    NoHost[player] = false;
    StartOrder[player] = ++StartCounter;
    StartFrame[player] = FrameCount;

    // NNS passes the sequence data of a whole SSEQ file, so the header sits right before it.
    // A non-zero offset means a sequence inside an SSAR, which the host renderer does not handle.
    u32 hdr = mml - 0x1C;
    if (offset == 0 && IsMainRAM(hdr) && RamRead32(hdr) == MAGIC_SSEQ)
    {
        u32 fsize = RamRead32(hdr + 8);
        if (fsize > 0x1C && fsize - 0x1C <= MAX_MML)
            s.MMLLen = fsize - 0x1C;
    }

    std::vector<u8> buf;
    if (s.MMLLen && CopyRAM(mml, s.MMLLen, buf))
    {
        s.CRC = CRC32(buf.data(), (int)s.MMLLen);
        s.Info = Idx.Lookup(s.CRC, s.MMLLen);
        if (s.Info)
            s.Cls = s.Info->Cls;
        else
            s.Cls = ClassifySeq(AnalyzeMML(buf.data(), s.MMLLen), s.ChanMask);
    }
    else
        s.MMLLen = 0;

    // the bank is what tells apart starts of one sequence with different banks; only a host needs it
    if (Eligible(s) && CopyBank(bank, buf))
        s.BankCRC = BankCRC32(buf);

    Log(LogLevel::Debug, "RealtimeBGM: start player %d %s (%s) crc=%08X len=%u%s\n", player,
        s.Info && !s.Info->Name.empty() ? s.Info->Name.c_str() : "?", SeqClassName(s.Cls), s.CRC, s.MMLLen,
        prepareOnly ? " [prepared]" : "");

    if (!prepareOnly && Settings.Enabled && FF && Eligible(s))
        EnterHostMode(player, true);
    UpdateMuteMask();
}

void SndCmdTracker::OnStop(int player)
{
    P[player].Active = false;
    P[player].Prepared = false;
    if (P[player].Parked)
    {
        Bgm.DropParked(player);
        P[player].Parked = false;
    }
    if (player == HostP)
        LeaveHostMode();
}

bool SndCmdTracker::EnterHostMode(int player, bool fromStart)
{
    PlayerState& s = P[player];
    std::vector<u8> mml, sbnk, swar[4];

    bool ok = s.MMLLen && CopyRAM(s.MML, s.MMLLen, mml) && CRC32(mml.data(), (int)s.MMLLen) == s.CRC;
    // the bank must still be the one the driver plays: the game may be rebuilding its sound heap
    // when a song is adopted mid-play
    ok = ok && CopyBank(s.Bank, sbnk) && BankCRC32(sbnk) == s.BankCRC;
    if (ok)
    {
        // SNDBankData.waveArcLink[4] (8 bytes each: waveArc, next) follows the 0x18-byte headers;
        // SND_AssignWaveArc fills waveArc with the address of the loaded SWAR.
        for (int i = 0; i < 4; i++)
        {
            u32 arc = RamRead32(s.Bank + 0x18 + i * 8);
            if (arc && !CopySwar(arc, swar[i]))
            {
                Log(LogLevel::Info, "RealtimeBGM: player %d wave archive %d at %08X could not be copied\n", player, i, arc);
                ok = false;
            }
        }
    }

    const u8* swarPtr[4];
    u32 swarLen[4];
    for (int i = 0; i < 4; i++)
    {
        swarPtr[i] = swar[i].empty() ? nullptr : swar[i].data();
        swarLen[i] = (u32)swar[i].size();
    }

    // the current host keeps playing if this player cannot be loaded
    if (!ok || !Bgm.Load(mml.data(), s.MMLLen, sbnk.data(), (u32)sbnk.size(), swarPtr, swarLen))
    {
        Log(LogLevel::Warn, "RealtimeBGM: could not load player %d (%s) into the host renderer\n", player,
            s.Info && !s.Info->Name.empty() ? s.Info->Name.c_str() : "?");
        NoHost[player] = true;
        return false;
    }

    if (HostP >= 0) LeaveHostMode();

    // the driver keeps the player's notes on its allocatable channels (all of them until ALLOCATABLE_CHANNEL)
    Bgm.SetChannelMask(s.ChanMask ? s.ChanMask : 0xFFFF);
    ApplyOutputSettings();
    u32 tick = fromStart ? 0 : TickCounter(player);
    Bgm.Start(tick);
    Log(LogLevel::Debug, "RealtimeBGM: host enter player %d tick %u fader %d tempo %u paused %d\n", player, tick, s.ExtFader, s.TempoRatio, s.Paused);
    Bgm.SetMasterVolume(MasterVol);
    Bgm.SetExtFader(s.ExtFader);
    Bgm.SetTempoRatio(s.TempoRatio);
    for (int t = 0; t < 16; t++)
    {
        if (TrackMuteMode[player][t]) Bgm.SetTrackMute(1 << t, TrackMuteMode[player][t]);
        if (TrackFader[player][t]) Bgm.SetTrackFader(1 << t, TrackFader[player][t]);
        if (TrackPitch[player][t]) Bgm.SetTrackPitch(1 << t, TrackPitch[player][t]);
        if (TrackPan[player][t]) Bgm.SetTrackPan(1 << t, TrackPan[player][t]);
    }
    if (!fromStart && SharedWork)
    {
        // the driver's current variables, including those set by the game mid-song
        for (int i = 0; i < 16; i++)
            Bgm.SetVariable((u8)i, (s16)RamRead32(SharedWork + 0x20 + player * 36 + i * 2));
    }
    if (SharedWork)
    {
        // global variables outlive the sequence
        for (int i = 0; i < 16; i++)
            Bgm.SetVariable((u8)(16 + i), (s16)RamRead32(SharedWork + SHARED_GLOBAL_VAR_OFS + i * 2));
    }
    if (s.Paused) Bgm.Pause(true);

    HostP = player;
    s.HostMode = true;
    MutedP |= 1 << player;
    // notes keyed on before this were tagged while the player was not muted
    NDS.SPU.RetagHostNotes();
    UpdateMuteMask();

    Log(LogLevel::Info, "RealtimeBGM: host renderer plays player %d (%s) from tick %u, mute mask %04X\n", player,
        s.Info && !s.Info->Name.empty() ? s.Info->Name.c_str() : "?", tick, CurMuteMask);
    return true;
}

void SndCmdTracker::LeaveHostMode()
{
    Bgm.Stop();
    if (HostP >= 0)
    {
        P[HostP].HostMode = false;
        Log(LogLevel::Info, "RealtimeBGM: host renderer released player %d\n", HostP);
    }
    HostP = -1;
    UpdateMuteMask();
}

void SndCmdTracker::ParkHost()
{
    PlayerState& s = P[HostP];
    Bgm.Park(HostP);
    s.Parked = true;
    s.ParkFader = s.ExtFader;
    s.HostMode = false;
    // the player stays muted: its hardware copy is still ahead of the parked voice
    Log(LogLevel::Info, "RealtimeBGM: host player %d paused, voice parked\n", HostP);
    HostP = -1;
    UpdateMuteMask();
}

bool SndCmdTracker::UnparkHost(int player)
{
    PlayerState& s = P[player];
    s.Parked = false;
    if (!Bgm.Unpark(player))
        return false;

    // what the game changed while the player was paused
    ApplyOutputSettings();
    Bgm.SetMasterVolume(MasterVol);
    Bgm.SetChannelMask(s.ChanMask ? s.ChanMask : 0xFFFF);
    Bgm.SetTempoRatio(s.TempoRatio);
    if (s.ExtFader != s.ParkFader) Bgm.SetExtFader(s.ExtFader);
    for (int t = 0; t < 16; t++)
    {
        Bgm.SetTrackMute(1 << t, TrackMuteMode[player][t]);
        Bgm.SetTrackFader(1 << t, TrackFader[player][t]);
        Bgm.SetTrackPitch(1 << t, TrackPitch[player][t]);
        Bgm.SetTrackPan(1 << t, TrackPan[player][t]);
    }
    if (SharedWork)
    {
        // local variables set while parked were not forwarded
        for (int i = 0; i < 16; i++)
            Bgm.SetVariable((u8)i, (s16)RamRead32(SharedWork + 0x20 + player * 36 + i * 2));
        for (int i = 0; i < 16; i++)
            Bgm.SetVariable((u8)(16 + i), (s16)RamRead32(SharedWork + SHARED_GLOBAL_VAR_OFS + i * 2));
    }

    HostP = player;
    s.HostMode = true;
    MutedP |= 1 << player;
    NDS.SPU.RetagHostNotes();
    UpdateMuteMask();
    Log(LogLevel::Info, "RealtimeBGM: host renderer resumes parked player %d (%s) at tick %u, mute mask %04X\n", player,
        s.Info && !s.Info->Name.empty() ? s.Info->Name.c_str() : "?", Bgm.Tick(), CurMuteMask);
    return true;
}

void SndCmdTracker::PickHost()
{
    // a resumed parked player gets its own voice back (see CMD_PAUSE_SEQ)
    int best = -1;
    for (int p = 0; p < 16; p++)
    {
        const PlayerState& s = P[p];
        if (!s.Parked || s.Paused || !(FF || (MutedP & (1 << p)))) continue;
        if (best < 0 || StartOrder[p] > StartOrder[best]) best = p;
    }
    if (best >= 0 && UnparkHost(best))
        return;
    if (!FF)
        return;

    u32 status = SharedWork ? RamRead32(SharedWork + 4) : 0xFFFF;
    best = -1;
    for (int p = 0; p < 16; p++)
    {
        const PlayerState& s = P[p];
        if (!s.Active || s.Prepared || s.Parked || NoHost[p] || !(status & (1 << p)) || !Eligible(s)) continue;
        if (best < 0 || StartOrder[p] > StartOrder[best]) best = p;
    }
    if (best >= 0)
        EnterHostMode(best, false);
}

void SndCmdTracker::SetFastForward(bool on)
{
    if (on == FF) return;
    FF = on;
    // when fast-forward ends the host stays the audible source until the next START/STOP on
    // that player, since the driver copy is now ahead in the song (design 6.2)
    if (FF && Settings.Enabled && HostP < 0)
        PickHost();
}

u32 SndCmdTracker::TickCounter(int player) const
{
    if (!SharedWork) return 0;
    // SNDSharedWork: 0x20 bytes of header, then per player { s16 variable[16]; u32 tickCounter; }
    return RamRead32(SharedWork + 0x20 + player * 36 + 32);
}

void SndCmdTracker::UpdateMuteMask()
{
    u16 mask = 0;
    if (HostP >= 0 && Settings.Enabled)
    {
        // a player counts while the driver reports it playing (sound effects finish without a STOP),
        // and right after its start, before the driver sets its status bit: a sound effect's first
        // notes must not be pre-muted on channels it may allocate
        u32 status = SharedWork ? RamRead32(SharedWork + 4) : 0xFFFF;
        u16 others = 0;
        for (int q = 0; q < 16; q++)
            if (q != HostP && P[q].Active && ((status & (1 << q)) || FrameCount - StartFrame[q] <= 2))
                others |= P[q].ChanMask;
        u16 exclusive = P[HostP].ChanMask & ~others;

        if (ChanOwnerValid)
        {
            // idle channels only the host may allocate (a note starting there before the next
            // snapshot would otherwise leak through); sounding notes are muted by their tag
            for (int ch = 0; ch < 16; ch++)
                if (ChanOwner[ch] < 0 && (exclusive & (1 << ch)))
                    mask |= 1 << ch;
        }
        else
            mask = exclusive;
    }

    CurMuteMask = mask;
}

bool SndCmdTracker::ParseDriverInfo()
{
    // SNDDriverInfo { SNDWork work; u32 chCtrl[16]; SNDWork* workAddress; u32 lockedChannels; u32 padding[6]; }
    std::vector<u8>& buf = DriverInfoBuf;
    if (!CopyRAM(DriverInfoAddr, WORK_SIZES[0] + 72, buf)) return false;

    for (u32 ch = 0; ch < 16; ch++)
        if (buf[ch * EXCH_SIZE] != ch) return false;

    u32 work = 0, workSize = 0;
    for (u32 size : WORK_SIZES)
    {
        u32 wa = Get32(&buf[size + 64]);
        u32 locked = Get32(&buf[size + 68]);
        if ((wa & 3) || locked > 0xFFFF || (wa >> 24) < 0x02 || (wa >> 24) > 0x03) continue;
        // the live ARM7 copy must look like an SNDWork too
        bool match = true;
        for (u32 ch = 0; ch < 16 && match; ch++)
            match = NDS.ARM7Read8(wa + ch * EXCH_SIZE) == ch;
        if (match) { work = wa; workSize = size; break; }
    }
    if (!work) return false;
    LiveWork = work;

    u32 trackBase = work + WORK_TRACK_OFS;
    for (u32 ch = 0; ch < 16; ch++)
    {
        const u8* c = &buf[ch * EXCH_SIZE];
        s8 owner = -1;
        u32 trk = Get32(c + EXCH_CALLBACK_DATA_OFS);
        if ((c[3] & 1) && trk >= trackBase && trk < trackBase + 32 * TRACK_SIZE && !((trk - trackBase) % TRACK_SIZE))
        {
            u8 t = (u8)((trk - trackBase) / TRACK_SIZE);
            for (int p = 0; p < 16 && owner < 0; p++)
            {
                const u8* pl = &buf[WORK_PLAYER_OFS + p * PLAYER_SIZE];
                if (!(pl[0] & 1)) continue;
                for (int k = 0; k < 16; k++)
                    if (pl[PLAYER_TRACKS_OFS + k] == t) { owner = (s8)p; break; }
            }
        }
        ChanOwner[ch] = owner;
    }

    if (DriverInfoLogged != 1)
    {
        DriverInfoLogged = 1;
        Log(LogLevel::Info, "RealtimeBGM: driver info at %08X parsed (SNDWork at %08X, %u bytes): using exact channel ownership\n",
            DriverInfoAddr, work, workSize);
    }
    return true;
}

int SndCmdTracker::ChannelKeyOnOwner(int ch)
{
    if (!MutedP || !Settings.Enabled || !ChanOwnerValid || !LiveWork) return -1;

    u32 trackBase = LiveWork + WORK_TRACK_OFS;
    u32 trk = NDS.ARM7Read32(LiveWork + ch * EXCH_SIZE + EXCH_CALLBACK_DATA_OFS);
    if (trk < trackBase || trk >= trackBase + 32 * TRACK_SIZE || (trk - trackBase) % TRACK_SIZE) return -1;
    u8 t = (u8)((trk - trackBase) / TRACK_SIZE);

    for (int p = 0; p < 16; p++)
    {
        if (!(MutedP & (1 << p))) continue;
        u32 pl = LiveWork + WORK_PLAYER_OFS + p * PLAYER_SIZE;
        if (!(NDS.ARM7Read8(pl) & 1)) continue;
        for (int k = 0; k < 16; k++)
            if (NDS.ARM7Read8(pl + PLAYER_TRACKS_OFS + k) == t) return p;
    }
    return -1;
}

void SndCmdTracker::ApplyOutputSettings()
{
    if (Settings.Interpolation != AppliedInterp)
    {
        AppliedInterp = Settings.Interpolation;
        Bgm.SetInterpolation(AppliedInterp);
    }
    if (Settings.OutputSkew != AppliedSkew)
    {
        AppliedSkew = Settings.OutputSkew;
        Bgm.SetOutputSkew(AppliedSkew);
    }
}

void SndCmdTracker::OnFrame()
{
    FrameCount++;
    ApplyOutputSettings();
    if (!Settings.Enabled)
    {
        if (HostP >= 0) LeaveHostMode();
        for (int p = 0; p < 16; p++)
        {
            if (!P[p].Parked) continue;
            Bgm.DropParked(p);
            P[p].Parked = false;
        }
        if (MutedP)
        {
            MutedP = 0;
            NDS.SPU.ClearHostTags();
        }
        // driver info is not parsed while disabled
        ChanOwnerValid = false;
    }
    if (HostP >= 0 && !Bgm.Playing() && !P[HostP].Paused)
    {
        // the host copy reached the end of a non-looping sequence
        Log(LogLevel::Info, "RealtimeBGM: host copy of player %d finished\n", HostP);
        NoHost[HostP] = true;
        LeaveHostMode();
    }

    if (DriverInfoPending && Settings.Enabled)
    {
        DriverInfoPending = false;
        bool wasValid = ChanOwnerValid;
        ChanOwnerValid = ParseDriverInfo();
        if (!ChanOwnerValid) ChanOwner.fill(-1);
        // notes can only be tagged with a valid snapshot (savestate load: tags were dropped)
        if (ChanOwnerValid && !wasValid && MutedP) NDS.SPU.RetagHostNotes();
        if (!ChanOwnerValid && DriverInfoLogged != 0)
        {
            DriverInfoLogged = 0;
            Log(LogLevel::Info, "RealtimeBGM: driver info at %08X not recognised: using ALLOCATABLE_CHANNEL masks\n", DriverInfoAddr);
        }
    }

    if (Settings.Enabled && HostP < 0)
        PickHost();

    // A muted player the host does not follow stays silent only while fast-forward may adopt it again
    // or while it is paused with its voice parked. Otherwise the hardware becomes its source again.
    // Stopped players stay muted for their release tails.
    for (int p = 0; p < 16; p++)
    {
        if (p == HostP || !(MutedP & (1 << p)) || !P[p].Active || (FF && !NoHost[p]) || (P[p].Parked && P[p].Paused)) continue;
        if (P[p].Parked)
        {
            Bgm.DropParked(p);
            P[p].Parked = false;
        }
        MutedP &= ~(1 << p);
        Log(LogLevel::Debug, "RealtimeBGM: player %d unmuted, hardware plays it again\n", p);
    }

    UpdateMuteMask();
}

void SndCmdTracker::DoSavestate(melonDS::Savestate* file)
{
    // without usable tracker state: same game, so the driver's work areas do not move
    auto resetKeepingWork = [this]
    {
        u32 sw = SharedWork, dia = DriverInfoAddr, dir = DriverInfoReq;
        Reset();
        SharedWork = sw; DriverInfoAddr = dia; DriverInfoReq = dir;
    };

    if (!file->Saving)
    {
        // Upstream savestates have no RealtimeBGM section, and a missing section is a load error,
        // so look for it first (same walk as Savestate::FindSection).
        const u8* b = (const u8*)file->Buffer();
        bool found = false;
        for (u32 ofs = 0x10; ofs + 8 <= file->BufferLength();)
        {
            if (!memcmp(b + ofs, "NELO", 4)) { found = true; break; }
            u32 len = Get32(b + ofs + 4);
            if (len == 0) break;
            ofs += len;
        }
        if (!found)
        {
            resetKeepingWork();
            return;
        }
    }

    file->Section("NELO");
    u32 version = SAVESTATE_VERSION;
    file->Var32(&version);
    if (version != SAVESTATE_VERSION)
    {
        // the next Section() call finds its section by name, which skips the rest of this one
        Log(LogLevel::Info, "RealtimeBGM: savestate section version %u not supported, tracker state reset\n", version);
        resetKeepingWork();
        return;
    }
    file->Var32(&SharedWork);
    file->Var32(&DriverInfoAddr);
    file->Var32(&DriverInfoReq);
    file->Var8(&MasterVol);
    for (int p = 0; p < 16; p++)
    {
        PlayerState& s = P[p];
        file->Bool32(&s.Active);
        file->Bool32(&s.Prepared);
        file->Bool32(&s.Paused);
        file->Var32(&s.MML);
        file->Var32(&s.MMLLen);
        file->Var32(&s.Bank);
        file->Var32(&s.CRC);
        file->Var32(&s.BankCRC);
        file->Var16(&s.ChanMask);
        file->Var16((u16*)&s.ExtFader);
        file->Var16(&s.TempoRatio);
        file->Var8((u8*)&s.Cls);
        file->Bool32(&s.HostMode);
        file->VarArray(TrackMuteMode[p], sizeof(TrackMuteMode[p]));
        file->Var32(&StartOrder[p]);
    }
    file->Var32(&StartCounter);
    s32 host = HostP;
    file->Var32((u32*)&host);

    if (file->Saving) return;

    // parked voices are not saved: a paused player is adopted again from RAM (PickHost)
    Bgm.Kill();
    Bgm.DropAllParked();
    HostP = -1;
    MutedP = 0;
    NoHost = {};
    ChanOwner.fill(-1);
    ChanOwnerValid = false;
    DriverInfoPending = false;
    EnsureIndex();
    for (PlayerState& s : P)
    {
        s.Info = s.MMLLen ? Idx.Lookup(s.CRC, s.MMLLen) : nullptr;
        s.HostMode = false;
        s.Parked = false;
    }
    if (host >= 0 && host < 16 && FF && Settings.Enabled)
        EnterHostMode(host, false);
    UpdateMuteMask();
}

}
