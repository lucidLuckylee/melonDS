/*
 * SSEQ Player - Player structure
 * By Naram Qashat (CyberBotX) [cyberbotx@cyberbotx.com]
 * Last modification on 2014-10-18
 *
 * Adapted from source code of FeOS Sound System
 * By fincs
 * https://github.com/fincs/FSS
 *
 * NelonDS changes: namespaced, writes s16 stereo directly, NitroSDK extFader /
 * per-track ext volume/pitch/pan/mute, tick counter, tick-limited/silent run for seeking,
 * allocatable channel mask.
 */

#pragma once

#include <memory>
#include <bitset>
#include <cstdint>
#include "SSEQ.h"
#include "Track.h"
#include "Channel.h"
#include "consts.h"

namespace melonDS::Sound::SSEQPlayer
{

struct Player
{
	uint8_t prio, nTracks;
	uint16_t tempo, tempoCount, tempoRate /* 8.8 fixed point */;
	int16_t masterVol; // MML main volume (0xC2) in centibels

	// Driver-side parameters written by the game (SNDPlayer.extFader, SNDTrack.extFader/ext_pitch/ext_pan, mute_flag)
	int16_t extFader;
	int16_t trackExtFader[FSS_TRACKCOUNT];
	int16_t trackExtPitch[FSS_TRACKCOUNT];
	int8_t trackExtPan[FSS_TRACKCOUNT];
	uint16_t trackMute;
	int outputVol; // SOUNDCNT master volume, 0..128
	uint16_t channelMask; // channels notes may be allocated on (NitroSDK track channel_mask, set by ALLOCATABLE_CHANNEL)

	uint32_t tickCounter; // SNDSharedWork tickCounter equivalent
	bool seqEnded;
	bool paused;
	bool skipNotes; // fast seek: run the sequence without keying notes
	uint32_t loopJumps; // tracks (bit per track id) that jumped back in the last RunTick, for the seek's loop detection

	const SSEQ *sseq;

	uint8_t trackIds[FSS_TRACKCOUNT];
	Track tracks[FSS_MAXTRACKS];
	Channel channels[16];
	int16_t variables[32];

	double sampleRate;
	Interpolation interpolation;

	Player();

	bool Setup(const SSEQ *sseq);
	void ClearState();
	void FreeTracks();
	void Stop(bool bKillSound);
	int ChannelAlloc(int type, int prio);
	int TrackAlloc();
	void RunTick();
	void Run(uint32_t tickLimit = UINT32_MAX);
	void UpdateTracks();
	void Timer(uint32_t tickLimit = UINT32_MAX);
	void SetSampleRate(double rate);
	void FlagTracks(uint16_t trackMask, int flag);
	void SetTrackMute(int trackNum, int mode);
	void SetPaused(bool pause);
	bool Finished() const;

	/* Playback helper */
	double secondsPerSample, secondsIntoPlayback, secondsUntilNextClock;
	std::bitset<16> mutes;
	void GenerateSamples(int16_t *buf, unsigned samples);
};

}
