/* Copyright (c) 2013-2015 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "sdl-audio.h"

#include <mgba/core/core.h>
#include <mgba/core/thread.h>

#include <math.h>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

mLOG_DEFINE_CATEGORY(SDL_AUDIO, "SDL Audio", "platform.sdl.audio");

#if SDL_VERSION_ATLEAST(3, 0, 0)
static void _mSDLAudioCallback(void* context, SDL_AudioStream* stream, int additionalLen, int totalLen);
#else
static void _mSDLAudioCallback(void* context, Uint8* data, int len);
#endif

#define STRETCH_IN 65536 // frames held for the stretch: a second at the GBA's highest rate
#define STRETCH_SEGMENT_MAX 8192

// Keeps the pitch when the game runs faster or slower than 1x: the core's audio is time-stretched back to real time
// instead of played fast or slow, the way SoundTouch does it: 80 ms segments played as they are, each started within
// 15 ms past its spot where it best continues the last one, joined by a 10 ms crossfade. (Grains overlapped by half
// smear music with many voices.) src/ds/stretch.ts in the site does the same for the DS.
// shortcut: one stretcher for the page's single audio device; move it into struct mSDLAudio if there are ever two.
static struct {
	bool on;
	bool active;
	unsigned rate;
	long segment;
	long overlap;
	long seek;
	int16_t in[STRETCH_IN * 2];
	long inLen;
	double pos; // where the next segment starts at the nominal rate
	double fill; // frames in out, smoothed
	double taken; // frames taken from the core per callback, smoothed
	bool hasTail;
	int16_t tail[STRETCH_SEGMENT_MAX * 2]; // the last segment's end, to fade from
	struct mAudioBuffer out;
	bool outInit;
} _stretch;

EMSCRIPTEN_KEEPALIVE void setKeepAudioPitch(bool on) {
	_stretch.on = on;
}

static void _stretchReset(unsigned sampleRate) {
	_stretch.rate = sampleRate;
	_stretch.segment = sampleRate * 8 / 100;
	if (_stretch.segment > STRETCH_SEGMENT_MAX) {
		_stretch.segment = STRETCH_SEGMENT_MAX;
	}
	_stretch.overlap = _stretch.segment / 8;
	_stretch.seek = _stretch.segment * 3 / 16;
	_stretch.inLen = 0;
	_stretch.pos = 0;
	_stretch.hasTail = false;
	_stretch.fill = 0;
	_stretch.taken = 0;
	if (!_stretch.outInit) {
		mAudioBufferInit(&_stretch.out, STRETCH_IN, 2);
		_stretch.outInit = true;
	}
	mAudioBufferClear(&_stretch.out);
}

// How much a segment starting at c sounds like the last one's end: normalized cross-correlation, mono.
static double _stretchScore(const int16_t* in, long c, long len, long step) {
	int64_t xy = 0, yy = 1;
	for (long i = 0; i < len; i += step) {
		int32_t x = _stretch.tail[i * 2] + _stretch.tail[i * 2 + 1];
		int32_t y = in[(c + i) * 2] + in[(c + i) * 2 + 1];
		xy += (int64_t) x * y;
		yy += (int64_t) y * y;
	}
	return xy / sqrt((double) yy);
}

// Takes everything the core has made and writes about 1/speed as many frames to _stretch.out.
static void _stretchRun(struct mAudioBuffer* core, double speed) {
	long space = STRETCH_IN - _stretch.inLen;
	long avail = mAudioBufferAvailable(core);
	if (avail > space) {
		mAudioBufferRead(core, NULL, avail - space); // after a stall: the oldest go
	}
	size_t taken = mAudioBufferRead(core, &_stretch.in[_stretch.inLen * 2], space);
	_stretch.inLen += taken;
	_stretch.taken += (taken - _stretch.taken) * 0.02;

	int16_t* in = _stretch.in;
	long segment = _stretch.segment, overlap = _stretch.overlap, seek = _stretch.seek;
	while (true) {
		long start = (long) _stretch.pos;
		if (start + seek + segment > _stretch.inLen) {
			break;
		}
		long best = start;
		if (_stretch.hasTail) {
			// Every 4th spot over every other frame, then each one around the best: the main thread has little to spare
			double bestScore = -INFINITY;
			for (long c = start; c <= start + seek; c += 4) {
				double score = _stretchScore(in, c, overlap, 2);
				if (score > bestScore) {
					bestScore = score;
					best = c;
				}
			}
			long from = best - 3 > start ? best - 3 : start, to = best + 3 < start + seek ? best + 3 : start + seek;
			for (long c = from; c <= to; ++c) {
				double score = _stretchScore(in, c, overlap, 1);
				if (score > bestScore) {
					bestScore = score;
					best = c;
				}
			}
		}
		// Faded from the last segment's end (a copy: below 1x the next segment starts inside this one), then the rest
		int16_t fade[STRETCH_SEGMENT_MAX / 4];
		for (long i = 0; i < overlap * 2; ++i) {
			int32_t k = _stretch.hasTail ? i / 2 * 256 / overlap : 256;
			fade[i] = (_stretch.tail[i] * (256 - k) + in[best * 2 + i] * k) >> 8;
		}
		mAudioBufferWrite(&_stretch.out, fade, overlap);
		mAudioBufferWrite(&_stretch.out, &in[(best + overlap) * 2], segment - 2 * overlap);
		memcpy(_stretch.tail, &in[(best + segment - overlap) * 2], overlap * 2 * sizeof(int16_t));
		_stretch.hasTail = true;
		_stretch.pos += speed * (segment - overlap);

		// The tail is copied out: nothing before the next start is read again (at high speeds it hasn't come yet)
		long drop = (long) _stretch.pos < _stretch.inLen ? (long) _stretch.pos : _stretch.inLen;
		memmove(_stretch.in, &_stretch.in[drop * 2], (_stretch.inLen - drop) * 2 * sizeof(int16_t));
		_stretch.inLen -= drop;
		_stretch.pos -= drop;
	}
}

bool mSDLInitAudio(struct mSDLAudio* context, struct mCoreThread* threadContext) {
#if defined(_WIN32) && SDL_VERSION_ATLEAST(2, 0, 8)
	if (!getenv("SDL_AUDIODRIVER")) {
		_putenv_s("SDL_AUDIODRIVER", "directsound");
	}
#endif
	if (!SDL_OK(SDL_InitSubSystem(SDL_INIT_AUDIO))) {
		mLOG(SDL_AUDIO, ERROR, "Could not initialize SDL sound system: %s", SDL_GetError());
		return false;
	}

	context->desiredSpec.freq = context->sampleRate;
	context->desiredSpec.channels = 2;
#if SDL_VERSION_ATLEAST(3, 0, 0)
	context->desiredSpec.format = SDL_AUDIO_S16;
	char hint[21];
	snprintf(hint, sizeof(hint), "%" PRIz "u", context->samples);
	SDL_SetHint(SDL_HINT_AUDIO_DEVICE_SAMPLE_FRAMES, hint);
	context->stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &context->desiredSpec, _mSDLAudioCallback, context);
	if (!context->stream) {
#else
	context->desiredSpec.callback = _mSDLAudioCallback;
	context->desiredSpec.userdata = context;
	context->desiredSpec.samples = context->samples;
	context->desiredSpec.format = AUDIO_S16SYS;
#if SDL_VERSION_ATLEAST(2, 0, 0)
	context->deviceId =
	    SDL_OpenAudioDevice(0, 0, &context->desiredSpec, &context->obtainedSpec, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
	if (context->deviceId == 0) {
#else
	if (SDL_OpenAudio(&context->desiredSpec, &context->obtainedSpec) < 0) {
#endif
#endif
		mLOG(SDL_AUDIO, ERROR, "Could not open SDL sound system");
		return false;
	}
	context->core = NULL;

#if SDL_VERSION_ATLEAST(3, 0, 0)
	SDL_GetAudioStreamFormat(context->stream, NULL, &context->obtainedSpec);
#endif

	mAudioBufferInit(&context->buffer, context->samples, context->obtainedSpec.channels);
	mAudioResamplerInit(&context->resampler, mINTERPOLATOR_SINC);
	mAudioResamplerSetDestination(&context->resampler, &context->buffer, context->obtainedSpec.freq);
	context->fill = 0;

	if (threadContext) {
		context->core = threadContext->core;
		context->sync = &threadContext->impl->sync;

		mSDLResumeAudio(context);
	}

	return true;
}

void mSDLDeinitAudio(struct mSDLAudio* context) {
	UNUSED(context);
#if SDL_VERSION_ATLEAST(3, 0, 0)
	SDL_DestroyAudioStream(context->stream);
#elif SDL_VERSION_ATLEAST(2, 0, 0)
	SDL_PauseAudioDevice(context->deviceId, 1);
	SDL_CloseAudioDevice(context->deviceId);
#else
	SDL_PauseAudio(1);
	SDL_CloseAudio();
#endif
	mAudioBufferDeinit(&context->buffer);
	mAudioResamplerDeinit(&context->resampler);
	SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

void mSDLPauseAudio(struct mSDLAudio* context) {
#if SDL_VERSION_ATLEAST(3, 0, 0)
	SDL_PauseAudioStreamDevice(context->stream);
#elif SDL_VERSION_ATLEAST(2, 0, 0)
	SDL_PauseAudioDevice(context->deviceId, 1);
#else
	UNUSED(context);
	SDL_PauseAudio(1);
#endif
}

void mSDLResumeAudio(struct mSDLAudio* context) {
#if SDL_VERSION_ATLEAST(3, 0, 0)
	SDL_ResumeAudioStreamDevice(context->stream);
#elif SDL_VERSION_ATLEAST(2, 0, 0)
	SDL_PauseAudioDevice(context->deviceId, 0);
#else
	UNUSED(context);
	SDL_PauseAudio(0);
#endif
}

#if SDL_VERSION_ATLEAST(3, 0, 0)
static void _mSDLAudioCallback(void* context, SDL_AudioStream* stream, int additionalLen, int len) {
	UNUSED(additionalLen);
#else
static void _mSDLAudioCallback(void* context, Uint8* data, int len) {
#endif
	struct mSDLAudio* audioContext = context;
	if (!context || !audioContext->core) {
#if SDL_VERSION_ATLEAST(3, 0, 0)
		return;
#else
		memset(data, 0, len);
		return;
#endif
	}
	struct mAudioBuffer* buffer = NULL;
	unsigned sampleRate = 32768;
	if (audioContext->core) {
		buffer = audioContext->core->getAudioBuffer(audioContext->core);
		sampleRate = audioContext->core->audioSampleRate(audioContext->core);
	}
	double fauxClock = 1;
	if (audioContext->sync) {
		if (audioContext->sync->fpsTarget > 0 && audioContext->core) {
			fauxClock = mCoreCalculateFramerateRatio(audioContext->core, audioContext->sync->fpsTarget);
		}
		mCoreSyncLockAudio(audioContext->sync);
		// One callback's worth of reserve on top: in a browser this callback shares the main thread with the
		// core's frame hook, and without the reserve any short stall there runs the speakers dry (a click).
		audioContext->sync->audioHighWater = 2 * audioContext->samples + audioContext->resampler.highWaterMark +
		    audioContext->resampler.lowWaterMark + (audioContext->samples >> 6);
		audioContext->sync->audioHighWater *= sampleRate / (fauxClock * audioContext->obtainedSpec.freq);
	}

	double rate = sampleRate / fauxClock;
	// fpsTarget is 60 times the game speed (main.ts: 1.5x is 2x at 45 fps)
	double speed = audioContext->sync && audioContext->sync->fpsTarget > 0 ? audioContext->sync->fpsTarget / 60 : 1;
	if (_stretch.on && buffer && fabs(speed - 1) > 0.01) {
		if (!_stretch.active || _stretch.rate != sampleRate) {
			_stretchReset(sampleRate);
		}
		_stretch.active = true;
		// The stretch is what the core makes per callback (measured: its sound doesn't always come at the rate it says)
		// over what the speakers play. On top, hold out near a segment and two callbacks by up to 25% (tempo, not pitch).
		double played = audioContext->samples * rate / speed / audioContext->obtainedSpec.freq;
		if (!_stretch.taken) {
			_stretch.taken = played * speed;
		}
		_stretch.fill += (mAudioBufferAvailable(&_stretch.out) - _stretch.fill) * 0.1;
		double target = 2 * played + _stretch.segment; // it comes a segment at a time
		double error = (_stretch.fill - target) / target;
		_stretchRun(buffer, _stretch.taken / played * (1 + 0.25 * (error > 1 ? 1 : error < -1 ? -1 : error)));
		buffer = &_stretch.out;
		rate /= speed;
	} else {
		_stretch.active = false;
	}
	if (audioContext->sync && !audioContext->sync->audioWait && buffer) {
		// Paced by the system clock (timestepSync), the core and the sound card drift apart: nudge the rate by up to
		// 0.5% (inaudible) to hold the core's buffer near three callbacks, instead of running dry or piling up.
		double target = 3.0 * audioContext->samples * rate / audioContext->obtainedSpec.freq;
		double fill = mAudioBufferAvailable(buffer);
		if (fill > 3 * target) {
			// After a stall (hidden tab): skip ahead rather than play seconds late
			mAudioBufferRead(buffer, NULL, fill - target);
			fill = target;
			audioContext->fill = target;
		}
		audioContext->fill += (fill - audioContext->fill) * 0.05; // the core adds a whole frame's samples at a time
		double error = (audioContext->fill - target) / target;
		rate *= 1 + 0.005 * (error > 1 ? 1 : error < -1 ? -1 : error);
	}
	mAudioResamplerSetSource(&audioContext->resampler, buffer, rate, true);
	mAudioResamplerProcess(&audioContext->resampler);
	if (audioContext->sync) {
		mCoreSyncConsumeAudio(audioContext->sync);
	}
#if SDL_VERSION_ATLEAST(3, 0, 0)
	int16_t data[2048];
	while (len > 0) {
		int thisRead = sizeof(data);
		if (len < thisRead) {
			thisRead = len;
		}
		int available = mAudioBufferRead(&audioContext->buffer, data, thisRead / 4) * 4;
		if (!available) {
			break;
		}
		SDL_PutAudioStreamData(stream, data, available);
		len -= available;
	}
#else
	len /= 2 * audioContext->obtainedSpec.channels;
	int available = mAudioBufferRead(&audioContext->buffer, (int16_t*) data, len);

	if (available < len) {
		memset(((short*) data) + audioContext->obtainedSpec.channels * available, 0,
		       (len - available) * audioContext->obtainedSpec.channels * sizeof(short));
	}
#endif
}
