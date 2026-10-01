# Fast-forward with background music at 1×: feasibility study for melonDS (and mGBA)

Date: 2026-10-01. Sources inspected: melonDS upstream `906e9eb` (2026-08-24), mGBA `c3c8e5e` (0.11 dev), NitroSDK (`ntrtwl/NitroSDK`), NitroSystem, pret `pokeplatinum` / `pokeheartgold` decompilations, `kode54/SSEQPlayer`, `agbplay`, `gba-mus-ripper`, `loveemu-lab/mp2ktool`. Pokémon HeartGold (USA) was used as the local test ROM because no Platinum ROM is on disk; both are Gen 4 and use the same sound library and the same player layout.

## 0. Executive answer

**Feasible on melonDS, and much cleaner than the hardware-level picture suggests.** melonDS itself knows nothing about SDAT/SSEQ, but the ARM9 game code does not talk to sound hardware either. Every Nitro SDK game drives a sound driver running on the ARM7 through a documented command protocol over the IPC FIFO. melonDS emulates that FIFO in one function. Sniffing it gives us, at zero per-instruction cost and in both interpreter and JIT modes:

- every sequence start / prepare / stop / pause, with the player number, a pointer to the MML bytes in main RAM and the bank pointer;
- every volume change and fade (fades are a stream of `PLAYER_PARAM` commands followed by `STOP_SEQ`);
- tempo ratio changes, track mutes, per-track volume/pitch/pan;
- the address of a shared-memory block in which the ARM7 reports which players are active and each player's tick counter.

So the abstraction boundary is **not** lost. It is lost only one step later, inside the ARM7 driver, when the driver turns MML into SPU register writes. We never need to go there.

The proposed design therefore is:

1. At ROM load, find every SDAT in the Nitro filesystem, parse INFO (and SYMB when present), and classify sequences using the PLAYER record plus SSEQ structure. This is only for pre-analysis and names. Classification can also be done purely at runtime from the MML bytes in RAM, so no per-game profile is required.
2. At runtime, sniff PXI tag-7 commands in `NDS::ARM9IOWrite32`. On `START_SEQ`/`PREPARE_SEQ` for a sequence classified BGM, record (player, MML, bank) and feed a host-side SSEQ player.
3. Keep the ARM7 driver playing the BGM exactly as before, so all game-visible state (player status bits, tick counters, handles, fades) stays faithful. Make the ARM7 copy inaudible by muting, in `SPU::Mix`, the SPU channels that the driver has reserved for the BGM player (the `ALLOCATABLE_CHANNEL` masks are also visible on the FIFO, and for Gen 4 Pokémon the BGM channel set is disjoint from the SFX channel set).
4. Render the BGM on the host with a real-time clock, apply fades/stops/tempo from the sniffed commands, and add it to the frontend's output buffer after `SPU::ReadOutput`.

All logic lives in the core (`src/`), which is what both upstream and melonDS-android-lib build. The frontend hook is two calls (set fast-forward state, mix BGM into the output).

A proof-of-concept command sniffer was added to `src/NDS.cpp` in this workspace and run headless against HeartGold. See section 5 for the log.

## 1. melonDS audio path (verified)

Call graph, ARM7 write → PCM:

```
ARM7 code (the Nitro sound driver)         str r0,[0x040004x0]
  → NDS::ARM7IOWrite32  (src/NDS.cpp:4213)   addr in 0x04000400..0x0400051F
    → SPU::Write32      (src/SPU.cpp:1363)    channel = (addr>>4)&0xF
      → SPUChannel::SetCnt/SetSrcAddr/SetTimerReload/SetLoopPos/SetLength (src/SPU.h)
         SetCnt sets KeyOn when bit31 rises

scheduler Event_SPU (registered in SPU ctor, src/SPU.cpp:213) every MixInterval cycles
  → SPU::Mix(spucycles)                      (src/SPU.cpp:859)
      for each of 16 channels: SPUChannel::DoRun → Run<type>(cycles)  (src/SPU.cpp:638)
         KeyOn → SPUChannel::Start (src/SPU.cpp:468) → FIFO_BufferData reads sample data from ARM7 memory
         NextSample_PCM8/PCM16/ADPCM/PSG/Noise; optional interpolation; << VolumeShift; * Volume
      SPUChannel::PanOutput(val, left, right) (src/SPU.cpp:729)   ← per-channel sum into stereo
      capture units, output selector (SOUNDCNT bits 8-11), master volume, bias, 10-bit degrade
      blip_add_delta(BlipLeft/Right, ...)      (blip_buf resampler, 16.76 MHz → output rate)
      every 512*128 cycles → SPU::BufferAudio (src/SPU.cpp:1025) → OutputBuffer ring (mutex)

frontend audio thread
  → EmuInstance::audioCallback (src/frontend/qt_sdl/EmuInstanceAudio.cpp:161)
      SPU::SetOutputSkew(targetFPS/INTERNAL_FRAME_RATE)
      len_in = audioGetNumSamplesOut(len) = len * curFPS/targetFPS     ← fast-forward handling
      SPU::ReadOutput(stream, len_in)       (src/SPU.cpp:1147)
      mute if audioMutedByFastForward; volume scale
```

Fast-forward in melonDS is purely a frontend concept: `EmuThread` (src/frontend/qt_sdl/EmuThread.cpp:341-364) sets `curFPS = fastForwardFPS`, the audio callback then pulls `curFPS/targetFPS` times more samples per period, which is what speeds up and pitches the whole mix. `NDS::RunFrame` (src/NDS.cpp:926) runs ARM9, ARM7, timers, GPU and the SPU event in lockstep on `SysTimestamp`; the core has no notion of wall time. `MuteFastForward` is the only existing audio/fast-forward interaction.

Key data structures: `SPUChannel` (Cnt, SrcAddr, TimerReload, LoopPos, Length, Volume, Pan, KeyOn, Pos, FIFO) and `SPU` (16 `Channels`, 2 `Capture`, `Cnt`, `MasterVolume`, blip buffers, `OutputBuffer`). `SPU::Mix` is the only place where channels become one stereo pair.

## 2. Where SDAT/SSEQ knowledge lives

Nowhere in melonDS: `grep -rn "SDAT\|SSEQ\|SBNK\|SWAR" src` is empty. The game's ARM7 binary contains the Nitro SDK sound driver (`libsnd`), which interprets SSEQ MML and writes the SPU registers. The ARM9 binary contains the NitroSystem layer (`NNS_SndArc*`) which loads SSEQ/SBNK/SWAR files from the SDAT into a per-player heap in main RAM and sends commands to the ARM7.

The protocol (verified in `NitroSDK/include/nitro/snd/common/command.h`, `nitro/pxi/common/fifo.h`, `libraries/snd/src/snd_command.c`):

```c
typedef struct SNDCommand { struct SNDCommand* next; SNDCommandID id; u32 arg[4]; } SNDCommand; // 24 bytes, main RAM
enum SNDCommandID { START_SEQ=0, STOP_SEQ=1, PREPARE_SEQ=2, START_PREPARED_SEQ=3, PAUSE_SEQ=4, SKIP_SEQ=5,
  PLAYER_PARAM=6, TRACK_PARAM=7, MUTE_TRACK=8, ALLOCATABLE_CHANNEL=9, PLAYER_LOCAL_VAR=10, PLAYER_GLOBAL_VAR=11,
  START_TIMER=12, STOP_TIMER=13, SETUP_CHANNEL_PCM=14, SETUP_CHANNEL_PSG=15, SETUP_CHANNEL_NOISE=16, SETUP_CAPTURE=17,
  SETUP_ALARM=18, CHANNEL_TIMER=19, CHANNEL_VOLUME=20, CHANNEL_PAN=21, SURROUND_DECAY=22, MASTER_VOLUME=23,
  MASTER_PAN=24, OUTPUT_SELECTOR=25, LOCK_CHANNEL=26, UNLOCK_CHANNEL=27, STOP_UNLOCKED_CHANNEL=28, SHARED_WORK=29,
  INVALIDATE_SEQ=30, INVALIDATE_BANK=31, INVALIDATE_WAVE=32, READ_DRIVER_INFO=33 };
// transport: SND_FlushCommand → PXI_SendWordByFifo(PXI_FIFO_TAG_SOUND /* =7 */, (u32)listHead, FALSE)
// FIFO word: bits 0-4 tag, bit 5 err, bits 6-31 data (= listHead address; 0 means "process queued commands")
```

Argument meaning (from `snd_interface.c` / `seq.h`):

| id | args |
|---|---|
| START_SEQ / PREPARE_SEQ | player (0-15), MML pointer, offset, `SNDBankData*` |
| START_PREPARED_SEQ, STOP_SEQ | player |
| PAUSE_SEQ | player, flag |
| PLAYER_PARAM | player, byte offset in `SNDPlayer`, value, size. Offset 6 = `extFader` (s16, 0 = full, negative = attenuation in driver decibel units; this is how every NNS volume set and fade reaches the driver). Offset 0x1A = `tempo_ratio` (256 = 1.0). Offset 4 = channel priority. |
| TRACK_PARAM | size<<24 \| player, track bitmask, offset in `SNDTrack`, value. Offset 0xA = track extFader, 0xC = ext pitch, 9 = pan. |
| MUTE_TRACK | player, track mask, mute mode |
| ALLOCATABLE_CHANNEL | player, track bitmask, SPU channel bitmask the player's tracks may use (NNS sends it at every sequence start, value from the SDAT PLAYER record) |
| SHARED_WORK | pointer to `SNDSharedWork` in main RAM: `+4 u32 playerStatus` (bit per active player), `+8 u16 channelStatus`, `+0x20 + 36*player: s16 variable[16]; u32 tickCounter` |

The MML pointer in START_SEQ is the start of the sequence *data*, not the file header. NNS loads the whole SSEQ file (header included) into the player heap and passes `file + baseOffset`, so the SSEQ header is at `arg1 - 0x1C` and still in RAM. It contains the file size, so the exact MML length is known and the sequence can be identified by hashing it against the SDAT.

## 3. Correlating runtime activity back to a sequence

Not needed at the SPU-channel level. Identification happens at the command level:

```
on IPC word with tag 7 and data >= 0x02000000:
  for node in linked list (bounded):
    id, a0..a3 = read main RAM
    switch id:
      START_SEQ / PREPARE_SEQ:
        hdr = a1 - 0x1C; if MainRAM[hdr..] == "SSEQ": len = u32(hdr+8) - 0x1C
        key = CRC32(MainRAM[a1 .. a1+len])
        seq = sdatIndex.lookup(key)            // name, INFO player/bank/vol, static class
        cls = seq ? seq.cls : classifyMML(MainRAM[a1..a1+len], playerMask[a0])   // runtime fallback
        if cls is BGM: BgmTracker.onStart(a0, mml bytes, bank = a3, pending = (id == PREPARE_SEQ))
      START_PREPARED_SEQ: BgmTracker.onStartPrepared(a0)
      STOP_SEQ:           BgmTracker.onStop(a0)
      PAUSE_SEQ:          BgmTracker.onPause(a0, a1)
      PLAYER_PARAM:       if a1 == 6:  BgmTracker.onFader(a0, (s16)a2)
                          if a1 == 0x1A: BgmTracker.onTempoRatio(a0, a2)
      TRACK_PARAM / MUTE_TRACK / PLAYER_LOCAL_VAR: forward
      ALLOCATABLE_CHANNEL: playerMask[a0] = a1
      SHARED_WORK:        sharedWorkAddr = a0
```

Why this is safe to do at FIFO-write time: `SND_FlushCommand` fully builds the list in RAM before sending the word, and the ARM9 does not reuse those nodes until the ARM7 replies. The write path is `NDS::ARM9IOWrite32` case `0x04000188` (src/NDS.cpp:3507); 16-bit writes are forwarded there (src/NDS.cpp:3341), and the JIT routes I/O stores through the same function (src/ARMJIT_Memory.cpp:1471). One branch on `(val & 0x1F) == 7` per FIFO word is the whole runtime cost.

Bank identification: `a3` is the `SNDBankData*` in RAM, which is the loaded SBNK file (magic `SBNK`), so the bank and its up-to-four SWARs can also be matched by content, or simply read from RAM. Reading from RAM is the more robust choice: it also covers games (and Platinum's cries) that swap banks at runtime with `NNS_SndArcPlayerStartSeqEx`.

Game-state polling without any CPU hook: once `SHARED_WORK` has been seen, `playerStatus` and per-player `tickCounter` can be read from main RAM every frame. This gives the exact sequencer position for resync (see 6.5) and lets the tracker confirm a player is really playing.

## 4. Static ROM analysis and classification

`tools/sdat_scan.py` walks the Nitro FNT/FAT, finds every `*.sdat`, parses INFO (SEQ, BANK, WAVEARC, PLAYER records), SYMB (present in retail Gen 4 ROMs) and does a linear decode of each SSEQ to count notes/events and detect backward jumps (loops). Result on HeartGold (`data/sound/gs_sound_data.sdat`, 2379 sequences, 9 players):

| player | SDAT role (from decomp) | chan mask | heap | sequences | heuristic class (no names used) |
|---|---|---|---|---|---|
| 0 | PV (cries) | 0xC000 | 0x5e88 | 7 | 7 cry/trigger |
| 1 | FIELD BGM | 0xA7FE | 0x3c8c | 277 | 269 BGM, rest jingle/SFX (silence placeholders) |
| 2 | ME (fanfares) | 0xA7FE | 0 | 49 | 32 jingle, 1 BGM (looping victory fanfare), 16 SFX-sized |
| 3-6 | SE | 0xD800 | small | 1638 | 894 SFX, 66 cry/trigger, 46 ambient-loop, 0 BGM, 0 jingle |
| 7 | BGM (non-field) | 0xA7FE | 0 | 224 | 139 BGM, 8 jingle, 5 SFX-sized |

Platinum (from `pokeplatinum/res/sound/pl_sound_data.json`): same layout, players 1 (FIELD, 0x07FF) and 7 (BGM, 0x07FF) hold music, 2 is ME, 3-6 SE (0xD800), 0 PV. `Sound_PlayBGM` chooses the player from the INFO record. **BGM rule for Gen 4: the sequence's INFO record says player 1 or 7; fanfares say 2.** (These are SDAT/NNS player numbers; the driver player number seen on the FIFO is allocated dynamically, see 5.1.)

Cries in Gen 4 are SSEQ: one shared two-note sequence (`SEQ_PV001`) started with a per-species SBNK/SWAR via `NNS_SndArcPlayerStartSeqEx`. They land on player 0, are two notes long and never loop, so they classify as SFX by every heuristic and keep running on the emulated clock, as requested.

Classification signals that worked without names, in priority order:

1. PLAYER record: heap size and channel-mask width. Music players get a wide mask and a large heap; SE players get 3-5 channels and tiny heaps. (The mask is also visible at runtime via `ALLOCATABLE_CHANNEL`.)
2. Looping (backward `JUMP 0x94`) plus note count ≥ 150 → BGM.
3. Non-looping, ≥ 30 notes, wide player → jingle/fanfare.
4. ≤ 4 notes → cry/trigger sequence (SE or PV).
5. Looping with few notes on a narrow player → ambient loop (rain, waterfall) → leave on the emulated clock.

The only ambiguous bucket is fanfares; a user-facing policy switch ("fanfares at 1× too: yes/no") handles it. Per-game overrides therefore can be optional: name-pattern (SEQ_SE_, SEQ_ME_, SEQ_PV) or id-list overrides are a refinement, not a requirement. Non-Nitro-SDK games (see section 9) simply get no classification and fall back to normal behaviour.

## 5. Proof-of-concept sniffer (done in this workspace)

`melonDS/src/NDS.cpp` now contains `SndCmdSniff()` called from the `0x04000188` case of `ARM9IOWrite32`. It walks the command list, logs every command and, for START/PREPARE, verifies the `SSEQ` magic at `mml-0x1C` and logs the CRC32 of the MML bytes. The CRC is matched offline against the SDAT by `tools/sdat_scan.py` logic. Results of the headless run are in section 5.1.

### 5.1 Headless run against HeartGold

Command: `QT_QPA_PLATFORM=offscreen SDL_AUDIODRIVER=dummy melonDS/build/melonDS "Pokemon - HeartGold Version (USA).nds"` for 75 s (about 70 s of emulated time, built-in free BIOS, default settings, JIT on). Captured 2124 sound commands:

| command | count |
|---|---|
| READ_DRIVER_INFO | 2095 (one per frame: the NNS layer polls the driver every frame) |
| INVALIDATE_SEQ/BANK/WAVE | 15 (heap frees) |
| PREPARE_SEQ + START_PREPARED_SEQ | 2 + 2 |
| ALLOCATABLE_CHANNEL | 4 (re-sent at every sequence start) |
| PLAYER_PARAM | 4 (offset 4 priority, offset 6 extFader) |
| SHARED_WORK, MASTER_PAN | 1 each |

The two sequence starts, decoded and matched against the SDAT by CRC32 of the MML bytes:

```
[SND 106847755]  PREPARE_SEQ player=0 mml=021C4D1C bank=021BCEE0 magic=SSEQ fsize=17820 crc=4CD3CED4
                 ALLOCATABLE_CHANNEL player=0 tracks=0xFFFF chmask=0xA7FE
                 PLAYER_PARAM player=0 offset=6 (extFader) value=-3
                 START_PREPARED_SEQ player=0
   → gs_sound_data.sdat seq 1004 SEQ_GS_TITLE, INFO player 7 (BGM), fsize 17820, crc 4CD3CED4   ✓
[SND 2369455188] PREPARE_SEQ player=1 mml=021C8BFC bank=021C0DC0 magic=SSEQ fsize=14356 crc=5F22ECA1
                 ALLOCATABLE_CHANNEL player=1 tracks=0xFFFF chmask=0xA7FE
   → gs_sound_data.sdat seq 1008 SEQ_GS_POKEMON_THEME, INFO player 7 (BGM), fsize 14356, crc 5F22ECA1 ✓
```

Both starts identified exactly, in JIT mode, with no CPU hook. Two corrections to the design that this run surfaced:

1. **Driver player numbers are not SDAT PLAYER indices.** The NNS layer owns 32 logical players and allocates one of the 16 driver players per start (here 0 and 1 for what the SDAT calls player 7). The BGM decision must therefore come from the CRC match (→ INFO record) or from the runtime signals (channel mask sent with ALLOCATABLE_CHANNEL at every start, plus MML structure), never from the driver player number alone. The tracker keys its state by driver player number but classifies by sequence.
2. **ALLOCATABLE_CHANNEL is `(player, trackBitMask, chBitMask)`** and NNS re-sends it on every start, so the mute mask per driver player is always fresh.

Also observed: the game issues `READ_DRIVER_INFO` every frame. The ARM7 answers by copying its whole `SNDWork` (16 `SNDExChannel`, 16 `SNDPlayer`, 32 `SNDTrack`) into ARM9 main RAM at `arg0`, together with `workAddress` (the ARM7 address of the live struct). Each `SNDExChannel.callback_data` is the ARM7 address of its owning `SNDTrack` (set in `snd_seq.c:884`), and `SNDPlayer.tracks[16]` lists the track indices per player. With `workAddress` the pointers resolve to indices, giving an **exact per-SPU-channel → driver player map once per frame** from main RAM, no heuristics. This is the preferred way to build the mute mask in games that poll driver info (all NNS games do); the ALLOCATABLE_CHANNEL mask remains the fallback. Caveat: struct layouts differ slightly between SDK versions, so the offsets must be derived from the SDK version string in the ARM9 binary (NitroSDK embeds one) or validated by sanity checks (channel `myNo == index`).

## 6. Removing BGM from the fast-forwarded mix and rendering it at 1×

### 6.1 Interception point

Do **not** suppress the START_SEQ command or patch the MML. Both desynchronise the driver's `playerStatus`/handles, and the NNS layer frees handles when the driver reports the player idle; later fades and stops would never be sent and game code that waits for a sequence to finish would misbehave.

Instead let the ARM7 play the BGM normally and silence it at the mixer. `SPU::Mix` is the single point where channels are summed. Add a 16-bit `ChannelMuteMask` to `SPU` and skip `PanOutput` for masked channels (also skip them as capture/output-selector sources). Three lines in src/SPU.cpp:

```cpp
// SPU.h
u16 ChannelMuteMask = 0;          // channels excluded from the mix (not from emulation)
// SPU.cpp, Mix()
for (int i = 4; i < 16; i++) { s32 v = Channels[i].DoRun(spucycles); if (!(ChannelMuteMask & (1<<i))) Channels[i].PanOutput(v, left, right); }
// same guard for channels 0-3
```

The mask comes from the tracker. Preferred source: the per-frame `READ_DRIVER_INFO` snapshot (section 5.1), which maps every SPU channel to its owning driver player exactly; mask = channels owned by players currently playing a BGM-classified sequence. Fallback when a game does not poll driver info: `mask = OR of chmask[p] for players p playing BGM, minus OR of chmask[q] for non-BGM players q`, using the masks from `ALLOCATABLE_CHANNEL`. For Platinum, BGM players use 0x07FF and SE/PV use 0xD800/0xC000, so the sets are already disjoint. For HeartGold, BGM is 0xA7FE and SE is 0xD800: channel 15 overlaps. Two options, both cheap: (a) accept that one SE channel is occasionally muted during fast-forward; (b) rewrite the `ALLOCATABLE_CHANNEL` argument in RAM when it is sent at init so BGM gets `0x27FE` (the write happens before the ARM7 reads the node, so it is race-free; this is the only RAM patch in the design and it is optional).

Because the driver still runs the sequence, every game-visible effect stays correct: the fade state machine in `SoundSystem_Tick`, `NNS_SndPlayerCountPlayingSeq`, variables, tick counters. Channel muting is invisible to the game.

### 6.2 When to switch sources

Policy that avoids audible jumps:

- Feature enabled, not fast-forwarding: hardware path, mask = 0, host renderer idle.
- Fast-forward engaged while BGM is playing on player p: read `tickCounter[p]` from shared work, start the host renderer at that tick (sequencer run silently up to it), set mask. From now until the next START_SEQ/STOP_SEQ on p the host renderer is the only BGM source, **even after fast-forward ends**, because the ARM7 copy is now ahead in the song. On the next START_SEQ the hardware path takes over again (or the host renderer, if still fast-forwarding).
- STOP_SEQ / fade: forwarded to the host renderer and the mask is cleared when the host renderer finishes the fade.

### 6.3 Host renderer

Reuse `kode54/SSEQPlayer` (C++11, WTFPL, no dependencies, ~6k lines, from fincs' FeOS Sound System via CyberBotX's NCSF player). It already implements: PCM8/PCM16/ADPCM/PSG/noise, hardware channel allocation order, ADSR, modulation, portamento/sweep, all MML opcodes except `MUTE 0xD7`, up to 16 tracks, variables/random/conditionals, loops/calls, tempo and tempo ratio, per-track mutes, master and sequence volume, output at any sample rate with selectable interpolation. Its `SDAT` class only loads one sequence from an SDAT file, so the integration layer is:

```cpp
class BgmRenderer {
public:
  void Load(const u8* mml, u32 len, const u8* sbnk, u32 sbnkLen, const u8* swar[4], u32 swarLen[4]); // from main RAM
  void Start();  void StartAtTick(u32 tick);  void Stop();  void Pause(bool);
  void SetExtFader(s16 driverDb);   // PLAYER_PARAM offset 6 → sseqVol via SDK decibel table
  void SetTempoRatio(u16 r);        // 256 = 1.0
  void SetTrackParam(u16 mask, u8 off, u32 val);  void MuteTracks(u16 mask, u8 mode);  void SetVar(u8 i, s16 v);
  void Render(s16* stereo, int frames, double outRate);   // called from the frontend audio thread, wall-clock paced
  bool Active() const;
private: Player player; Platform::Mutex* lock; ... };
```

Written new: this wrapper (~400 lines), the SDAT index and classifier in C++ (~500 lines, port of `tools/sdat_scan.py`), the command tracker (~300 lines), seeking by tick (SSEQPlayer runs the sequencer in `Player::Run()`; a "run N ticks without mixing" entry point is a small modification), and the decibel mapping (copy the SDK `SND_CalcDecibel` table so host loudness matches the driver).

Correctness of instruments: the SBNK and SWARs are read from main RAM at START_SEQ time (bank pointer `a3`; the bank header lists its wave archives, which NNS has also loaded into the heap). This gives exactly the instruments the driver uses, including runtime bank swaps.

### 6.4 Mixing back

In `EmuInstance::audioCallback`, after `SPU::ReadOutput`, call `nds->SPU.Bgm.Render(tmp, len, outRate)` and add with clamp. The host renderer is paced by the audio device, so BGM is at 1× by construction. For melonDS-android-lib the same two calls go into its audio output path. The frontend also tells the core `nds->SPU.SetFastForward(bool)` from the fast-forward toggle (src/frontend/qt_sdl/EmuThread.cpp:357).

### 6.5 Transitions and savestates

- Area transitions and battle transitions in Pokémon are `Sound_FadeOutBGM` (PLAYER_PARAM fades) then `STOP_SEQ` then `START_SEQ` of the new track, so they map one-to-one onto tracker events.
- Savestate load: the tracker is rebuilt from the sniffed `SHARED_WORK` pointer and `playerStatus` (both are in the saved RAM), but the *identity* of the playing sequence is not. Until the next START_SEQ, BGM is not separated. Fix later by saving the tracker's small state in the savestate (`SPU::DoSavestate`, bump `SAVESTATE_MINOR`).
- Rewind: same as savestate load.

## 7. Existing players that can be reused

| project | language / license | synthesises audio | notes |
|---|---|---|---|
| kode54/SSEQPlayer | C++11 / WTFPL | yes, complete | recommended; needs loader shim and seek-by-tick |
| CyberBotX/NCSF (new) | C# / MIT | yes | not embeddable in C++ |
| CyberBotX/SDATStuff | C++ / none stated | timing only | parsers useful as reference |
| TetraSsky/DualRip | Python / MIT | yes | reference for SSAR handling |
| VGMTrans | C++ / zlib | no (MIDI/SF2 export) | parser reference |
| Nitro Studio 2 | C# / unclear | yes | not embeddable |

No Rust crate exists for SDAT/SSEQ. No existing emulator implements this feature: melonDS issue #1476 asks for it and is open, DeSmuME #183 was closed won't-fix.

## 8. mGBA

Audio path: `GBAIOWrite` (src/gba/io.c:300) → `GBAAudioWriteSOUND*` / `GBAAudioWriteFIFO` (src/gba/audio.c:279); timers 0/1 overflow → `GBAAudioSampleFIFO` (audio.c:300) pops one byte and re-arms FIFO DMA; `GBAAudioSample` (audio.c:361) sums PSG (shared GB core `GBAudioSamplePSG`) and FIFO A/B into one int16 stereo pair; `_sample` (audio.c:409) writes to `audio->psg.buffer` which `_GBACoreGetAudioBuffer` exposes. Fast-forward is frontend-only (`sync.fpsTarget`, `fastForwardVolume`/`Mute`), implemented by resampling that one buffer faster.

MP2K knowledge: none active, but `include/mgba/internal/gba/audio.h:22-253` still carries the complete MP2K RAM layout (`GBAMP2kContext`/SoundInfo with `MP2K_MAGIC 0x68736D53`, `GBAMP2kMusicPlayerInfo`, tracks, channels) left over from the removed 0.8-era HLE mixer. These structs are directly reusable to read the live engine state from WRAM.

Hooks: zero-cost function hooks exist via the cheat mechanism: `GBASetBreakpoint` (src/gba/gba.c:1049) patches a BKPT whose immediate selects a `mCPUComponent`; `GBABreakpoint` (gba.c:922) dispatches and `ARMRunFake` executes the displaced opcode. A new `CPU_COMPONENT_MISC_n` case gives a hook on `m4aSongNumStart`/`MPlayStart`/`m4aMPlayStop`/`m4aMPlayVolumeControl`. Per-frame polling of the MP2K structs via `mCoreCallbacks.videoFrameStarted` costs nothing. Debugger breakpoints/watchpoints exist but drop to per-instruction stepping.

Locating the engine automatically: `agbplay/MP2KScanner.cpp` (LGPL-3) scans for the song table (≥ N consecutive valid 8-byte entries referenced from ROM), finds the player table from the literal pools of the `m4aSongNum*` functions, and reads the sound mode/SoundInfo pointer from the `m4aSoundInit` literal pool. `gba-mus-ripper/sappy_detector.c` and `loveemu mp2ktool` (MIT) use a byte signature of the `m4a_selectsong` Thumb prologue and validate the table; mp2ktool also finds `SoundMain`, `m4aSoundVSync` and the MPlayTable. Either approach yields: song table, player table, SoundInfo address, function entry points. mGBA's `override.XXXX` ini and CRC32-keyed `pokemonTable` (src/gba/overrides.c) are the natural home for cached results.

Separation on GBA is harder than on DS because MP2K is a *software* mixer: BGM and SFX PCM tracks are mixed into the same double-buffered FIFO stream, and CGB (PSG) tracks are also driven by MP2K. The BGM is its own `MusicPlayerInfo` (player 0 in Pokémon; SE players 1-2). Plan: hook `m4aSongNumStart`/`MPlayStart` on the BGM player to learn the song header pointer; mute the BGM player in RAM each frame (write 0 to that player's track volumes, or set its `masterVolume`/fade fields so the engine renders it silent; mGBA's structs give the offsets); render the song on the host with agbplay's player (LGPL-3; or a smaller MP2K sequencer written from the known format). Expect a milestone-level effort similar to the DS one plus the song-format work. Defer until the DS prototype works.

GB/GBC: `src/gb/audio.c` has four PSG channels only; music engines time-slice SFX onto the same channels and there is no engine-level command layer visible from outside. Not worth attempting with this approach.

## 9. Difficult cases

- Music and SFX sharing hardware channels: handled by muting only channels reserved for BGM players; with overlapping masks, optionally rewrite `ALLOCATABLE_CHANNEL`. Worst case is an occasionally muted SFX note during fast-forward.
- Dynamically generated sound: Platinum's reversed/echoed cries and Chatot use `NNS_SndWaveOut*` direct PCM channels. These never go through START_SEQ, stay on the emulated clock, and use channels outside the BGM mask.
- Environmental loops (rain, sea): short looping SSEQ on SE players → classified ambient-loop → emulated clock. Policy switch if a user prefers them at 1×.
- Jingles/fanfares: ME player, non-looping; either clock is defensible. Default: 1× (musical), configurable. Note Pokémon pauses/fades the BGM around fanfares via PLAYER_PARAM; the tracker sees it.
- Tempo changes: `TEMPO 0xE1` inside the MML is handled by the host sequencer; `tempo_ratio` via PLAYER_PARAM is forwarded.
- Songs depending on game state: sequence variables (`PLAYER_LOCAL_VAR`/`GLOBAL_VAR`) are forwarded; `READ_DRIVER_INFO` snapshots can be used for verification. Games that drive music by starting/stopping many short sequences (unusual) degrade gracefully because each start/stop is an event.
- Non-standard engines (games not using NitroSDK libsnd: some Square Enix, Capcom, homebrew): no tag-7 sound commands of this shape appear, the SDAT scan finds nothing, and the feature stays off automatically. Games that use libsnd with streams (`STRM`, played via `NNS_SndStrm*`) for BGM: streams use SETUP_CHANNEL_PCM / DMA to channels and are detectable as a separate case (rare in Pokémon: none in Gen 4; BW/BW2 use SSEQ too).
- DSi mode: the I2S/DSP path is mixed after `SPU::Mix` (src/SPU.cpp:1001), unaffected.
- Savestates/rewind: see 6.5.

## 10. Is automatic classification reliable enough?

Yes for Nitro SDK games, and specifically for all Gen 4/5 Pokémon. The PLAYER record alone separates music from effects with no mistakes on HeartGold's 2379 sequences; SSEQ structure confirms it and distinguishes BGM, jingle and ambient. Runtime classification from RAM (same signals: player channel mask from ALLOCATABLE_CHANNEL, loop detection and note count from the MML bytes at `a1`) does not even need the ROM parse. Profiles become optional overrides for taste (fanfares, ambient loops) and for the rare engine that assigns players unusually.

## 11. Recommended prototype architecture

```
src/
  Sound/SdatIndex.{h,cpp}     ROM → list of SSEQ {crc, len, name?, player, bank, static class}; runs in NDS::SetNDSCart
  Sound/SseqClassify.{h,cpp}  classifyMML(bytes, len, playerMask) → {BGM, Jingle, Ambient, SFX, Cry}
  Sound/SndCmdTracker.{h,cpp} called from NDS::ARM9IOWrite32 (0x04000188); keeps per-player state, shared-work ptr,
                              ALLOCATABLE_CHANNEL masks; emits events to BgmRenderer; computes SPU::ChannelMuteMask
  Sound/BgmRenderer.{h,cpp}   wraps SSEQPlayer (third-party, src/SSEQPlayer/); loads from main RAM; Render() for frontends
  SPU.{h,cpp}                 ChannelMuteMask in Mix(); SetFastForward(); Bgm member; savestate hooks
frontend/qt_sdl
  EmuThread.cpp               nds->SPU.SetFastForward(fastforward)
  EmuInstanceAudio.cpp        mix Bgm.Render() into the callback output; settings toggle
```

Milestones:

1. **Done here:** SDAT scan + classification tool; FIFO sniffer logging START/STOP/PLAYER_PARAM with SSEQ CRC; verified on HeartGold.
2. Port the index/classifier to C++ in the core; match CRCs at START_SEQ; log "BGM started: SEQ_GS_C_KIKYOU on player 1". Also log STOP/fade/tempo/ALLOCATABLE_CHANNEL/SHARED_WORK (pure logging milestone, no behaviour change). Verify in Platinum via the user's Android device or any Platinum ROM.
3. `ChannelMuteMask` in `SPU::Mix`, set from the tracker while fast-forwarding. Verify: at 4× the music is gone, SFX and cries are still there and sped up, game logic unaffected.
4. Vendor SSEQPlayer; `BgmRenderer::Load` from main RAM; render the detected BGM to a WAV file offline as a correctness check against the hardware path at 1×.
5. Wire `Render()` into the Qt audio callback; start at `tickCounter`; apply fades/stops/tempo; the feature works end to end.
6. Policies (fanfares, ambient), savestate persistence of tracker state, optional `ALLOCATABLE_CHANNEL` rewrite for overlapping masks, settings UI.
7. Upstream PR; Android: same core patch plus two lines in melonDS-android-lib's audio output.

Likely failure modes to watch: a game that starts BGM with PREPARE_SEQ + START_PREPARED_SEQ split across frames (handled: pending state); SSEQ loaded without its header (NNS always loads the whole file; homebrew FSS may not → fall back to runtime classification without CRC); tempo drift between host sequencer and driver on long fast-forwards (irrelevant after the host becomes the sole source); loudness mismatch (use the SDK decibel tables); SSEQPlayer's missing `MUTE 0xD7` (add it, trivial).

## 12. Implementation status (2026-10-01, "NelonDS")

Implemented in `melonDS/` (desktop, builds and runs headless) by three parallel agents against fixed interfaces:

- `src/Sound/SdatIndex.{h,cpp}`: Nitro FS walk, SDAT INFO/SYMB/FAT parse, SSAR entries, CRC32 keyed lookup, `AnalyzeMML`/`ClassifySeq`. Fuzzed (ASan/UBSan) on truncated/random ROMs. HeartGold: 1372 + 829 sequences indexed, per-player class counts identical to `tools/sdat_scan.py`.
- `src/Sound/SSEQPlayer/`: vendored kode54/SSEQPlayer (WTFPL) in namespace `melonDS::Sound::SSEQPlayer`, hardened (bounds checks, no sinc tables), extended with extFader/track fader/pitch/pan, MUTE 0xD7, tick counter and seek, NitroSDK volume mapping (extFader is centibels; `SND_CalcChannelVolume`, snd_util.c:303). `src/Sound/BgmRenderer.{h,cpp}` wraps it thread-safely. 25 checks pass incl. TSan; renders SEQ_GS_TITLE from RAM-style blobs at 130x realtime.
- `src/Sound/SndCmdTracker.{h,cpp}`: PXI tag-7 sniffer, per-driver-player state, host-mode policy, exact channel ownership from the per-frame READ_DRIVER_INFO snapshot (validated against the live ARM7 copy; SNDWork is 4480 bytes on HeartGold), implicit stops on INVALIDATE_SEQ/BANK, savestate section "NELO" (upstream states still load).
- Core hooks: `NDS.cpp` (member, Reset, DoSavestate, SetNDSCart, RunFrame, FIFO write), `SPU::Mix` mute mask. Qt frontend: fast-forward → `SetFastForward`, BGM mixed into `audioCallback`, settings `NelonDS.*`, checkbox in Interface settings, window title "NelonDS (melonDS x.y)".
- Patches: `nelonds-core.patch`, `nelonds-qt-frontend.patch`.

Headless verification (HeartGold, JIT): index built, both title sequences identified by CRC, driver-info parsing active, and with fast-forward forced on the host renderer took over each sequence (mute masks 0xA7FE/0xBFFF) and released on INVALIDATE_SEQ.

Not yet verified by ear: actual audio on a real device, mid-song fast-forward entry (tick seek), fades reaching the renderer, savestates. Known gaps: per-track fader/pitch/pan not restored on mid-song entry; SSAR sequences tracked but not rendered; slow-motion not treated as fast-forward.

Android: `android/` holds a nix flake (SDK 37, NDK 28, CMake 3.22.1, JDK 21) that builds melonDS-android (app c42995ca, lib 431ab4bd). Core hook sites are identical in melonDS-android-lib; the port adds `SetFastForward` via the JNI fast-forward toggle, mixes `Renderer().Render()` in `MelonInstance::readAudioOutput`, and a `nelonds_bgm_normal_speed` preference.

### 12.1 On-device results (AYN Thor, 2026-10-01 evening)

- APK built with a new Gradle build type `nelon` in `android/melonDS-android/app/build.gradle.kts`: release optimization and shrinking, package id `me.magnum.melonds.dev` (installs next to the Play Store melonDS), debug-signed, native flags `-O3 -march=armv8.6-a+crypto -mtune=cortex-x3`, `CMAKE_BUILD_TYPE=Release`, `ENABLE_LTO=ON`. The first debug-variant APK ran the core unoptimized and could not exceed 1x. Build: `nix develop --command bash -c 'cd melonDS-android && bash ./gradlew --no-daemon :app:assembleGitHubProdNelon -Pandroid.injected.build.abi=arm64-v8a'`; install with `adb install -r -t`.
- Verified on Volt White 2 Redux: index built (1390 sequences), driver-info parsing active, overworld BGM identified by CRC, host renderer took over at fast-forward start ("from tick 218"), and the user confirmed it works by ear.
- Two fixes made from the device log: (1) players that have started but whose driver status bit is not yet set now count as "others" in the mute mask, so a sound effect's first notes are not pre-muted; (2) `JinglesAt1x` now defaults to false, because a fanfare stole the host from the BGM, played twice (empty mute mask) and was cut off when the fast-forwarded hardware copy ended.
- AYN Thor specifics: the app must run on the top screen (display 0); launched on the bottom screen (display 4, "Screen-2") it finds no secondary display. Saves: melonDS-android stores saves next to the ROM when the ROM comes from a registered folder; the Play Store app had used its private folder because games were launched through the Cocoon frontend.
