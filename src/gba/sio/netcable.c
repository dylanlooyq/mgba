/* See include/mgba/internal/gba/sio/netcable.h */
#include <mgba/internal/gba/sio/netcable.h>

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>
#include <mgba/internal/gba/video.h>

#define MASTER_HANDSHAKE 0x8FFF
#define SLAVE_HANDSHAKE 0xB9A0
#define GROUP 9
#define TRANSFER_GAP (64 * 197) // link.c's timer 3 between a group's transfers

enum { PHASE_IDLE, PHASE_HANDSHAKE, PHASE_DATA };

static bool _handlesMode(struct GBASIODriver* driver, enum GBASIOMode mode) {
	struct GBASIONetCable* cable = (struct GBASIONetCable*) driver;
	return cable->players > 1 && mode == GBA_SIO_MULTI;
}

static int _connectedDevices(struct GBASIODriver* driver) {
	struct GBASIONetCable* cable = (struct GBASIONetCable*) driver;
	return cable->players > 1 ? cable->players - 1 : 0;
}

static int _deviceId(struct GBASIODriver* driver) {
	struct GBASIONetCable* cable = (struct GBASIONetCable*) driver;
	return cable->players > 1 ? cable->seat : 0;
}

static void _idle(struct GBASIONetCable* cable) {
	cable->phase = PHASE_IDLE;
	if (cable->d.p) {
		mTimingDeschedule(&cable->d.p->p->timing, &cable->clock);
	}
}

static void _setMode(struct GBASIODriver* driver, enum GBASIOMode mode) {
	if (mode != GBA_SIO_MULTI) {
		_idle((struct GBASIONetCable*) driver);
	}
}

static uint16_t _writeSIOCNT(struct GBASIODriver* driver, uint16_t value) {
	struct GBASIONetCable* cable = (struct GBASIONetCable*) driver;
	if (!GBASIOMultiplayerIsIrq(value)) {
		_idle(cable); // link.c's DisableSerial
	} else if (cable->phase == PHASE_IDLE) {
		// EnableSerial: the handshake starts over
		cable->phase = PHASE_HANDSHAKE;
		cable->prevCount = 0;
		cable->pos = 0;
		cable->sum = 0;
		if (cable->seat) {
			mTimingDeschedule(&driver->p->p->timing, &cable->clock);
			mTimingSchedule(&driver->p->p->timing, &cable->clock, VIDEO_TOTAL_LENGTH);
		}
	}
	// sio.c keeps bits 2-7 from before (unplugged, this GBA read as a slave): say them afresh
	value = GBASIOMultiplayerSetSlave(value, cable->seat != 0);
	value = GBASIOMultiplayerClearError(value);
	return GBASIOMultiplayerFillReady(value);
}

static bool _start(struct GBASIODriver* driver) {
	UNUSED(driver);
	return true; // the master's transfer: finished by sio.c's event, through _finishMultiplayer
}

static void _record(struct GBASIONetCable* cable, int kind, const uint16_t* words) {
	if (cable->outCount == NETCABLE_OUT_CAP) {
		// shortcut: the page reads these every few ms; if it stalls this long the link is lost anyway
		cable->outHead = (cable->outHead + 1) % NETCABLE_OUT_CAP;
		--cable->outCount;
		++cable->dropped;
	}
	uint16_t* rec = cable->out[(cable->outHead + cable->outCount) % NETCABLE_OUT_CAP];
	rec[0] = kind;
	memset(&rec[1], 0, 8 * sizeof(uint16_t));
	if (words) {
		memcpy(&rec[1], words, 8 * sizeof(uint16_t));
	}
	++cable->outCount;
}

static bool _validHandshake(uint16_t v) {
	return (v & ~3) == SLAVE_HANDSHAKE || v == MASTER_HANDSHAKE;
}

// One transfer: what this GBA sent, and what it hears back.
static void _exchange(struct GBASIONetCable* cable, uint16_t sent, uint16_t data[4]) {
	int i;
	MutexLock(&cable->mutex);
	int n = cable->players;
	if (cable->phase != PHASE_DATA) {
		for (i = 0; i < 4; ++i) {
			if (i >= n) {
				data[i] = 0xFFFF;
			} else if (i == cable->seat) {
				data[i] = sent;
			} else if (i == 0) {
				data[i] = cable->confirmed ? MASTER_HANDSHAKE : SLAVE_HANDSHAKE;
			} else {
				data[i] = SLAVE_HANDSHAKE;
			}
		}
		// link.c's DoHandshake: it finishes on two transfers in a row with the same count, the master's word first
		int count = 0;
		for (i = 0; i < 4; ++i) {
			if (_validHandshake(data[i])) {
				++count;
			} else {
				if (data[i] != 0xFFFF) {
					count = 0;
				}
				break;
			}
		}
		// What this GBA sent says DoHandshake ran on the transfer before (it writes its word there)
		bool ran = cable->seat ? sent == SLAVE_HANDSHAKE : sent == MASTER_HANDSHAKE;
		if (count > 1 && count == cable->prevCount && data[0] == MASTER_HANDSHAKE && ran) {
			cable->phase = PHASE_DATA;
			cable->pos = 0;
			cable->sum = 0;
			if (cable->seat) {
				cable->confirmed = false;
			} else {
				_record(cable, NETCABLE_CONFIRM, NULL);
			}
		}
		cable->prevCount = count;
		MutexUnlock(&cable->mutex);
		return;
	}

	if (cable->pos == 0) {
		// The checksum word: everyone's sum is the one this GBA added up, since everyone heard the same words
		for (i = 0; i < 4; ++i) {
			data[i] = i < n ? cable->sum : 0xFFFF;
		}
		cable->sum = 0;
		if (cable->inCount) {
			memcpy(cable->cur, cable->in[cable->inHead], sizeof(cable->cur));
			cable->inHead = (cable->inHead + 1) % NETCABLE_IN_CAP;
			--cable->inCount;
		} else {
			memset(cable->cur, 0, sizeof(cable->cur)); // received nothing this frame: link.c drops an all-zero group
		}
	} else {
		int k = cable->pos - 1;
		for (i = 0; i < 4; ++i) {
			data[i] = i < n ? cable->cur[i][k] : 0xFFFF;
			if (i < n) {
				cable->sum += data[i];
			}
		}
		cable->sent[k] = sent;
		if (k == 7) {
			bool any = false;
			for (i = 0; i < 8; ++i) {
				any = any || cable->sent[i];
			}
			if (any || !cable->seat) {
				_record(cable, NETCABLE_CMD, cable->sent);
			}
		}
	}
	cable->pos = (cable->pos + 1) % GROUP;
	MutexUnlock(&cable->mutex);
}

static void _finishMultiplayer(struct GBASIODriver* driver, uint16_t data[4]) {
	struct GBASIONetCable* cable = (struct GBASIONetCable*) driver;
	_exchange(cable, driver->p->p->memory.io[GBA_REG(SIOMLT_SEND)], data);
}

// A slave's transfers, at the pace link.c's master keeps: one per frame in the handshake, then a group of 9 per frame.
static void _clock(struct mTiming* timing, void* user, uint32_t cyclesLate) {
	struct GBASIONetCable* cable = user;
	struct GBASIO* sio = cable->d.p;
	if (!sio || cable->players < 2 || !cable->seat || cable->phase == PHASE_IDLE || sio->mode != GBA_SIO_MULTI) {
		return;
	}
	uint16_t data[4];
	_exchange(cable, sio->p->memory.io[GBA_REG(SIOMLT_SEND)], data);
	GBASIOMultiplayerFinishTransfer(sio, data, cyclesLate);
	int32_t next = VIDEO_TOTAL_LENGTH;
	if (cable->phase == PHASE_DATA) {
		next = cable->pos ? TRANSFER_GAP : VIDEO_TOTAL_LENGTH - (GROUP - 1) * TRANSFER_GAP;
	}
	mTimingSchedule(timing, &cable->clock, next);
}

void GBASIONetCableCreate(struct GBASIONetCable* cable) {
	memset(cable, 0, sizeof(*cable));
	MutexInit(&cable->mutex);
	cable->d.handlesMode = _handlesMode;
	cable->d.connectedDevices = _connectedDevices;
	cable->d.deviceId = _deviceId;
	cable->d.setMode = _setMode;
	cable->d.writeSIOCNT = _writeSIOCNT;
	cable->d.start = _start;
	cable->d.finishMultiplayer = _finishMultiplayer;
	cable->clock.context = cable;
	cable->clock.name = "GBA SIO Net Cable";
	cable->clock.callback = _clock;
	cable->clock.priority = 0x80;
}

void GBASIONetCablePlug(struct GBASIONetCable* cable, int seat, int players) {
	_idle(cable);
	MutexLock(&cable->mutex);
	cable->seat = seat;
	cable->players = players > 1 && players <= 4 && seat >= 0 && seat < players ? players : 0;
	cable->confirmed = false;
	cable->prevCount = 0;
	cable->pos = 0;
	cable->sum = 0;
	cable->inHead = cable->inCount = 0;
	cable->outHead = cable->outCount = 0;
	cable->dropped = 0;
	MutexUnlock(&cable->mutex);
}

bool GBASIONetCablePush(struct GBASIONetCable* cable, const uint16_t* entry) {
	MutexLock(&cable->mutex);
	bool ok = cable->inCount < NETCABLE_IN_CAP;
	if (ok) {
		memcpy(cable->in[(cable->inHead + cable->inCount) % NETCABLE_IN_CAP], entry, sizeof(cable->cur));
		++cable->inCount;
	}
	MutexUnlock(&cable->mutex);
	return ok;
}

void GBASIONetCableConfirm(struct GBASIONetCable* cable) {
	MutexLock(&cable->mutex);
	cable->confirmed = true;
	MutexUnlock(&cable->mutex);
}

int GBASIONetCableTake(struct GBASIONetCable* cable, uint16_t* out, int max) {
	MutexLock(&cable->mutex);
	int n = cable->outCount < max ? cable->outCount : max;
	int i;
	for (i = 0; i < n; ++i) {
		memcpy(&out[i * NETCABLE_RECORD], cable->out[cable->outHead], sizeof(cable->out[0]));
		cable->outHead = (cable->outHead + 1) % NETCABLE_OUT_CAP;
	}
	cable->outCount -= n;
	MutexUnlock(&cable->mutex);
	return n;
}

int GBASIONetCableStatus(struct GBASIONetCable* cable) {
	MutexLock(&cable->mutex);
	int status = cable->phase | (cable->inCount << 4);
	MutexUnlock(&cable->mutex);
	return status;
}
