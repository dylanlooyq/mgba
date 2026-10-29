/* Copyright (c) 2013-2024 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba-util/audio-resampler.h>

#include <mgba-util/audio-buffer.h>

#define MAX_CHANNELS 2

struct mAudioResamplerData {
	struct mAudioResampler* resampler;
	unsigned channel;
	// The source copied out once per process call: peeking the circle buffer for every tap was most of the cost
	const int16_t* samples;
	size_t count;
};

static int16_t _sampleAt(int index, const void* context) {
	const struct mAudioResamplerData* data = context;
	if (index < 0 || (size_t) index >= data->count) {
		return 0;
	}
	return data->samples[index * data->resampler->source->channels + data->channel];
}

void mAudioResamplerInit(struct mAudioResampler* resampler, enum mInterpolatorType interpType) {
	memset(resampler, 0, sizeof(*resampler));
	resampler->interpType = interpType;
	switch (interpType) {
	case mINTERPOLATOR_SINC:
		mInterpolatorSincInit(&resampler->sinc, 0, 0);
		resampler->lowWaterMark = resampler->sinc.width;
		resampler->highWaterMark = resampler->sinc.width;
		break;
	case mINTERPOLATOR_COSINE:
		mInterpolatorCosineInit(&resampler->cosine, 0);
		resampler->lowWaterMark = 0;
		resampler->highWaterMark = 1;
		break;
	}
}

void mAudioResamplerDeinit(struct mAudioResampler* resampler) {
	switch (resampler->interpType) {
	case mINTERPOLATOR_SINC:
		mInterpolatorSincDeinit(&resampler->sinc);
		break;
	case mINTERPOLATOR_COSINE:
		mInterpolatorCosineDeinit(&resampler->cosine);
		break;
	}
	resampler->source = NULL;
	resampler->destination = NULL;
}

void mAudioResamplerSetSource(struct mAudioResampler* resampler, struct mAudioBuffer* source, double rate, bool consume) {
	resampler->source = source;
	resampler->sourceRate = rate;
	resampler->consume = consume;
}

void mAudioResamplerSetDestination(struct mAudioResampler* resampler, struct mAudioBuffer* destination, double rate) {
	resampler->destination = destination;
	resampler->destRate = rate;
}

size_t mAudioResamplerProcess(struct mAudioResampler* resampler) {
	int16_t sampleBuffer[MAX_CHANNELS] = {0};
	double timestep = resampler->sourceRate / resampler->destRate;
	double timestamp = resampler->timestamp;
	struct mInterpolator* interp = &resampler->interp;
	struct mAudioResamplerData context = {
		.resampler = resampler,
	};
	struct mInterpolationData data = {
		.at = _sampleAt,
		.context = &context,
	};

	size_t read = 0;
	mASSERT(resampler->source->channels <= MAX_CHANNELS);
	context.count = mAudioBufferAvailable(resampler->source);
	int16_t* samples = malloc(context.count * resampler->source->channels * sizeof(int16_t) + 1);
	context.count = samples ? mAudioBufferDump(resampler->source, samples, context.count, 0) : 0;
	context.samples = samples;

	while (true) {
		if (timestamp + resampler->highWaterMark >= mAudioBufferAvailable(resampler->source)) {
			break;
		}
		if (mAudioBufferAvailable(resampler->destination) == mAudioBufferCapacity(resampler->destination)) {
			break;
		}

		size_t channel;
		for (channel = 0; channel < resampler->source->channels; ++channel) {
			context.channel = channel;
			sampleBuffer[channel] = interp->interpolate(interp, &data, timestamp, timestep);
		}
		if (!mAudioBufferWrite(resampler->destination, sampleBuffer, 1)) {
			break;
		}
		timestamp += timestep;
		++read;
	}
	free(samples);

	if (resampler->consume && timestamp > resampler->lowWaterMark) {
		size_t drop = timestamp - resampler->lowWaterMark;
		drop = mAudioBufferRead(resampler->source, NULL, drop);
		timestamp -= drop;
	}
	resampler->timestamp = timestamp;
	return read;
}
