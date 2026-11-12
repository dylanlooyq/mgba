/* A link cable to GBAs in other browsers, for games on Pokémon Gen 3's link layer (link.c).
 *
 * The cable can't carry raw transfers: every transfer's answer depends on the one before (each group of 9 ends in a
 * checksum of what was received), so a network round trip per transfer would crawl. Instead it carries the game's own
 * 8-word commands. Each GBA's group sends one command; seat 0's page puts everyone's commands in one order and hands
 * the same entries to every GBA, which gets them as the received words of its next groups (or nothing, as the game
 * already allows). The checksum word and the handshake are answered here, from what this GBA was given. Slaves are
 * clocked here at the master's pace (one group per frame). */
#ifndef GBA_SIO_NETCABLE_H
#define GBA_SIO_NETCABLE_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include <mgba/core/timing.h>
#include <mgba/internal/gba/sio.h>
#include <mgba-util/threading.h>

#define NETCABLE_IN_CAP 512
#define NETCABLE_OUT_CAP 512
#define NETCABLE_RECORD 9 // kind + 8 words

enum {
	NETCABLE_CMD = 1, // this GBA's group sent these 8 words (seat 0 sends every group's, zeros too: it paces the order)
	NETCABLE_CONFIRM = 2, // seat 0's leader confirmed the players: the slaves' handshakes may finish
};

struct GBASIONetCable {
	struct GBASIODriver d;
	Mutex mutex;
	struct mTimingEvent clock;

	int seat;
	int players; // 0: unplugged
	int phase;
	int prevCount;
	bool confirmed;
	int pos;
	uint16_t sum;
	uint16_t cur[4][8];
	uint16_t sent[8];

	uint16_t in[NETCABLE_IN_CAP][4][8];
	int inHead, inCount;
	uint16_t out[NETCABLE_OUT_CAP][NETCABLE_RECORD];
	int outHead, outCount;
	int dropped;
};

void GBASIONetCableCreate(struct GBASIONetCable*);
// players 0 unplugs. Call with the core thread stopped (it may touch the core's timing).
void GBASIONetCablePlug(struct GBASIONetCable*, int seat, int players);
// One entry: 4 players x 8 words.
bool GBASIONetCablePush(struct GBASIONetCable*, const uint16_t* entry);
void GBASIONetCableConfirm(struct GBASIONetCable*);
// Up to max records of NETCABLE_RECORD words; returns how many.
int GBASIONetCableTake(struct GBASIONetCable*, uint16_t* out, int max);
// phase | entries waiting << 4
int GBASIONetCableStatus(struct GBASIONetCable*);

CXX_GUARD_END

#endif
