/*
 * SSEQ Player - Player structure
 * By Naram Qashat (CyberBotX) [cyberbotx@cyberbotx.com]
 * Last modification on 2014-10-23
 *
 * Adapted from source code of FeOS Sound System
 * By fincs
 * https://github.com/fincs/FSS
 */

#include "Player.h"
#include "common.h"

namespace melonDS::Sound::SSEQPlayer
{


Player::Player() : prio(0), nTracks(0), tempo(0), tempoCount(0), tempoRate(0), masterVol(0), extFader(0), trackMute(0), outputVol(128), channelMask(0xFFFF),
	tickCounter(0), seqEnded(false), paused(false), skipNotes(false), loopJumps(0), sseq(nullptr), sampleRate(32768), interpolation(INTERPOLATION_NONE),
	secondsPerSample(1.0 / 32768), secondsIntoPlayback(0), secondsUntilNextClock(SecondsPerClockCycle)
{
	memset(this->trackIds, 0, sizeof(this->trackIds));
	memset(this->trackExtFader, 0, sizeof(this->trackExtFader));
	memset(this->trackExtPitch, 0, sizeof(this->trackExtPitch));
	memset(this->trackExtPan, 0, sizeof(this->trackExtPan));
	for (size_t i = 0; i < 16; ++i)
	{
		this->channels[i].chnId = i;
		this->channels[i].ply = this;
	}
	memset(this->variables, -1, sizeof(this->variables));
}

// Original FSS Function: Player_Setup
bool Player::Setup(const SSEQ *sseqToPlay)
{
	this->sseq = sseqToPlay;

	int firstTrack = this->TrackAlloc();
	if (firstTrack == -1)
		return false;
	this->tracks[firstTrack].Init(firstTrack, this, nullptr, 0);

	this->nTracks = 1;
	this->trackIds[0] = firstTrack;

	this->tracks[firstTrack].startPos = this->tracks[firstTrack].pos = &this->sseq->data[0];

	this->secondsPerSample = 1.0 / this->sampleRate;

	this->ClearState();

	return true;
}

// Original FSS Function: Player_ClearState
void Player::ClearState()
{
	this->tempo = 120;
	this->tempoCount = 240; // SND_BASE_TEMPO: the driver runs the first tick right away
	this->tempoRate = 0x100;
	this->tickCounter = 0;
	this->seqEnded = false;
	this->paused = false;
	this->masterVol = 0; // this is actually the highest level
	memset(this->variables, -1, sizeof(this->variables));
	this->secondsIntoPlayback = 0;
	this->secondsUntilNextClock = SecondsPerClockCycle;
}

// Original FSS Function: Player_FreeTracks
void Player::FreeTracks()
{
	for (uint8_t i = 0; i < this->nTracks; ++i)
		this->tracks[this->trackIds[i]].Free();
	this->nTracks = 0;
}

// Original FSS Function: Player_Stop
void Player::Stop(bool bKillSound)
{
	this->ClearState();
	for (uint8_t i = 0; i < this->nTracks; ++i)
	{
		uint8_t trackId = this->trackIds[i];
		this->tracks[trackId].ClearState();
		for (int j = 0; j < 16; ++j)
		{
			Channel &chn = this->channels[j];
			if (chn.state != CS_NONE && chn.trackId == trackId)
			{
				if (bKillSound)
					chn.Kill();
				else
					chn.Release();
			}
		}
	}
	this->FreeTracks();
}

// Original FSS Function: Chn_Alloc
// Like NitroSDK SND_AllocExChannel, only channels in channelMask are considered.
int Player::ChannelAlloc(int type, int priority)
{
	static const uint8_t pcmChnArray[] = { 4, 5, 6, 7, 2, 0, 3, 1, 8, 9, 10, 11, 14, 12, 15, 13 };
	static const uint8_t psgChnArray[] = { 8, 9, 10, 11, 12, 13 };
	static const uint8_t noiseChnArray[] = { 14, 15 };
	static const uint8_t arraySizes[] = { sizeof(pcmChnArray), sizeof(psgChnArray), sizeof(noiseChnArray) };
	static const uint8_t *const arrayArray[] = { pcmChnArray, psgChnArray, noiseChnArray };

	auto chnArray = arrayArray[type];
	int arraySize = arraySizes[type];

	int curChnNo = -1;
	for (int i = 0; i < arraySize; ++i)
	{
		int thisChnNo = chnArray[i];
		if (!(this->channelMask & (1 << thisChnNo)))
			continue;
		Channel &thisChn = this->channels[thisChnNo];
		if (curChnNo != -1 && thisChn.prio >= this->channels[curChnNo].prio)
		{
			Channel &curChn = this->channels[curChnNo];
			if (thisChn.prio != curChn.prio)
				continue;
			if (curChn.vol <= thisChn.vol)
				continue;
		}
		curChnNo = thisChnNo;
	}

	if (curChnNo == -1 || priority < this->channels[curChnNo].prio)
		return -1;
	this->channels[curChnNo].noteLength = -1;
	this->channels[curChnNo].vol = 0x7FF;
	this->channels[curChnNo].clearHistory();
	return curChnNo;
}

// Original FSS Function: Track_Alloc
int Player::TrackAlloc()
{
	for (int i = 0; i < FSS_MAXTRACKS; ++i)
	{
		Track &thisTrk = this->tracks[i];
		if (!thisTrk.state[TS_ALLOCBIT])
		{
			thisTrk.Zero();
			thisTrk.state.set(TS_ALLOCBIT);
			thisTrk.updateFlags.reset();
			return i;
		}
	}
	return -1;
}

// One sequencer tick (NitroSDK PlayerSeqMain)
void Player::RunTick()
{
	if (this->seqEnded)
		return;
	this->loopJumps = 0;
	bool anyActive = false;
	for (uint8_t i = 0; i < this->nTracks; ++i)
	{
		Track &trk = this->tracks[this->trackIds[i]];
		trk.Run();
		if (!trk.state[TS_END])
			anyActive = true;
	}
	++this->tickCounter;
	if (!anyActive)
		this->seqEnded = true;
}

// Original FSS Function: Player_Run (NitroSDK PlayerTempoMain)
// Stops early once tickCounter reaches tickLimit; the remaining ticks run on the next call.
void Player::Run(uint32_t tickLimit)
{
	while (this->tempoCount >= 240)
	{
		if (this->tickCounter >= tickLimit)
			return;
		this->tempoCount -= 240;
		this->RunTick();
	}
	this->tempoCount += (static_cast<int>(this->tempo) * static_cast<int>(this->tempoRate)) >> 8;
}

void Player::UpdateTracks()
{
	for (int i = 0; i < 16; ++i)
		this->channels[i].UpdateTrack();
	for (int i = 0; i < FSS_MAXTRACKS; ++i)
		this->tracks[i].updateFlags.reset();
}

// Original FSS Function: Snd_Timer
void Player::Timer(uint32_t tickLimit)
{
	this->UpdateTracks();

	for (int i = 0; i < 16; ++i)
		this->channels[i].Update();

	if (!this->paused)
		this->Run(tickLimit);
}

void Player::SetSampleRate(double rate)
{
	if (rate == this->sampleRate)
		return;
	for (int i = 0; i < 16; ++i)
		this->channels[i].reg.sampleIncrease *= this->sampleRate / rate;
	this->sampleRate = rate;
	this->secondsPerSample = 1.0 / rate;
}

void Player::FlagTracks(uint16_t trackMask, int flag)
{
	for (uint8_t i = 0; i < this->nTracks; ++i)
	{
		Track &trk = this->tracks[this->trackIds[i]];
		if (trackMask & (1 << (trk.num & 15)))
			trk.updateFlags.set(flag);
	}
}

// NitroSDK SetTrackMute: 0 = off, 1 = mute (notes keep playing), 2 = mute + release, 3 = mute + stop
void Player::SetTrackMute(int trackNum, int mode)
{
	trackNum &= 15;
	if (mode == 0)
		this->trackMute &= ~(1 << trackNum);
	else
		this->trackMute |= 1 << trackNum;
	if (mode < 2)
		return;
	for (uint8_t i = 0; i < this->nTracks; ++i)
	{
		Track &trk = this->tracks[this->trackIds[i]];
		if ((trk.num & 15) != trackNum)
			continue;
		for (int j = 0; j < 16; ++j)
		{
			Channel &chn = this->channels[j];
			if (chn.state == CS_NONE || chn.trackId != trk.trackId)
				continue;
			// SND_SEQ_MUTE_STOP releases at rate 127 and frees the channel from the track
			if (mode == 3)
			{
				chn.releaseRate = Cnv_Fall(127);
				chn.Release();
				chn.trackId = -1;
			}
			else
				chn.Release();
		}
	}
}

// NitroSDK SND_PauseSeq: stop sequencing and release held notes (release rate 127)
void Player::SetPaused(bool pause)
{
	if (pause && !this->paused)
		for (int i = 0; i < 16; ++i)
			if (this->channels[i].state != CS_NONE)
			{
				this->channels[i].releaseRate = Cnv_Fall(127);
				this->channels[i].Release();
			}
	this->paused = pause;
}

bool Player::Finished() const
{
	if (!this->seqEnded)
		return false;
	for (int i = 0; i < 16; ++i)
		if (this->channels[i].state != CS_NONE)
			return false;
	return true;
}

static inline int32_t muldiv7(int32_t val, uint8_t mul)
{
	return mul == 127 ? val : ((val * mul) >> 7);
}

void Player::GenerateSamples(int16_t *buf, unsigned samples)
{
	unsigned long mute = this->mutes.to_ulong();

	for (unsigned smpl = 0; smpl < samples; ++smpl)
	{
		this->secondsIntoPlayback += this->secondsPerSample;

		int32_t leftChannel = 0, rightChannel = 0;

		// I need to advance the sound channels here
		for (int i = 0; i < 16; ++i)
		{
			Channel &chn = this->channels[i];

			if (chn.state > CS_NONE)
			{
				int32_t sample = chn.GenerateSample();
				chn.IncrementSample();

				if (mute & BIT(i))
					continue;

				uint8_t datashift = chn.reg.volumeDiv;
				if (datashift == 3)
					datashift = 4;
				sample = muldiv7(sample, chn.reg.volumeMul) >> datashift;

				// pan law of the SPU (melonDS SPUChannel::PanOutput): 127 counts as 128
				int32_t pan = chn.reg.panning == 127 ? 128 : chn.reg.panning;
				leftChannel += (sample * (128 - pan)) >> 7;
				rightChannel += (sample * pan) >> 7;
			}
		}

		leftChannel = (leftChannel * this->outputVol) >> 7;
		rightChannel = (rightChannel * this->outputVol) >> 7;
		clamp(leftChannel, -0x8000, 0x7FFF);
		clamp(rightChannel, -0x8000, 0x7FFF);

		*buf++ = leftChannel;
		*buf++ = rightChannel;

		if (this->secondsIntoPlayback > this->secondsUntilNextClock)
		{
			this->Timer();
			this->secondsUntilNextClock += SecondsPerClockCycle;
		}
	}
}

}
