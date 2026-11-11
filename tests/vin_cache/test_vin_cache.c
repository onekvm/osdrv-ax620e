// SPDX-License-Identifier: GPL-2.0
/*
 * Host test of ovc_cpu_cache.h against a cache-line model of one capture
 * buffer: the WDMA writes DDR line by line while the CPU may speculate any
 * line in or evict it at any time, and user space reads, writes, maps and
 * unmaps between queue operations. Checks that
 *
 *   - every frame the WDMA completes reaches DQBUF intact (no dirty line was
 *     evicted onto it),
 *   - after DQBUF user space reads what DDR holds, or what it wrote itself,
 *   - a client that never touches the pixels costs no cache operation, and a
 *     reader costs one fault, one invalidate and one clean per frame.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ovc_cpu_cache.h"

#define LINES	16
#define FRAMES	4000

struct line {
	bool present, dirty;
	int val;
};

struct sim {
	struct ovc_cc cc;
	int ddr[LINES];
	struct line cache[LINES];
	int wrote[LINES];	/* user's own value per line, -1 = none */
	bool mapped, ptes, queued;
	bool clean_inval;	/* INVAL as clean+invalidate (allowed on arm64) */
	unsigned long faults, cleans, invals;
	unsigned long long rng;
};

static int failures;

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
		fprintf(stderr, __VA_ARGS__); \
		fputc('\n', stderr); \
		failures++; \
		return; \
	} \
} while (0)

static unsigned int rnd(struct sim *s, unsigned int n)
{
	s->rng ^= s->rng << 13;
	s->rng ^= s->rng >> 7;
	s->rng ^= s->rng << 17;
	return (unsigned int)(s->rng % n);
}

static void apply(struct sim *s, unsigned int act)
{
	int i;

	if (act & OVC_CC_ZAP)
		s->ptes = false;
	if (act & OVC_CC_CLEAN) {
		s->cleans++;
		for (i = 0; i < LINES; i++) {
			if (s->cache[i].present && s->cache[i].dirty)
				s->ddr[i] = s->cache[i].val;
			s->cache[i].dirty = false;
		}
	}
	if (act & OVC_CC_INVAL) {
		s->invals++;
		for (i = 0; i < LINES; i++) {
			if (s->clean_inval && s->cache[i].present &&
			    s->cache[i].dirty)
				s->ddr[i] = s->cache[i].val;
			s->cache[i].present = s->cache[i].dirty = false;
		}
	}
}

/* The CPU may pull any line in through the linear map, or evict it. */
static void background(struct sim *s)
{
	int i = rnd(s, LINES);
	struct line *l = &s->cache[i];

	switch (rnd(s, 4)) {
	case 0:
		if (!l->present)
			*l = (struct line){ .present = true, .val = s->ddr[i] };
		break;
	case 1:
		if (l->present && l->dirty)
			s->ddr[i] = l->val;
		l->present = l->dirty = false;
		break;
	}
}

/* A user access: through the PTEs, or a fault that maps the buffer. */
static bool touch(struct sim *s)
{
	if (!s->mapped)
		return false;
	if (!s->ptes) {
		s->faults++;
		apply(s, ovc_cc_fault(&s->cc));
		s->ptes = true;
	}
	return true;
}

static int user_read(struct sim *s, int i)
{
	struct line *l = &s->cache[i];

	if (!l->present)
		*l = (struct line){ .present = true, .val = s->ddr[i] };
	return l->val;
}

static void user_write(struct sim *s, int i, int val)
{
	s->cache[i] = (struct line){ .present = true, .dirty = true, .val = val };
	s->wrote[i] = val;
}

static void map(struct sim *s)
{
	s->mapped = true;
}

static void unmap(struct sim *s)
{
	if (!s->mapped)
		return;
	s->mapped = s->ptes = false;
	ovc_cc_unmapped(&s->cc);
}

static void qbuf(struct sim *s)
{
	apply(s, ovc_cc_qbuf(&s->cc));
	s->queued = true;
}

static void dqbuf(struct sim *s)
{
	apply(s, ovc_cc_dqbuf(&s->cc));
	s->queued = false;
	memset(s->wrote, -1, sizeof(s->wrote));
}

/* After DQBUF: every line reads as DDR, or as the user's own write. */
static void check_reads(struct sim *s, const char *who)
{
	int i, want, got;

	if (!touch(s))
		return;
	for (i = 0; i < LINES; i++) {
		background(s);
		want = s->wrote[i] >= 0 ? s->wrote[i] : s->ddr[i];
		got = user_read(s, i);
		CHECK(got == want, "%s: line %d reads %d, memory holds %d",
		      who, i, got, want);
	}
}

enum client {
	CORE,		/* maps every buffer, never touches the pixels */
	READER,		/* ffmpeg: DQBUF, read the frame, QBUF */
	WRITER,		/* reads and draws into the dequeued frame */
	EARLY,		/* also reads while the buffer is queued */
	CHURN,		/* maps and unmaps at random points */
	CLIENTS
};

static const char *const names[CLIENTS] = {
	"core", "reader", "writer", "early-reader", "map-churn"
};

static void run(enum client who, bool clean_inval, bool streamoff)
{
	struct sim s = { .rng = 0x9e3779b97f4a7c15ULL ^ (who * 977 + clean_inval),
			 .clean_inval = clean_inval };
	int frame, i, written;

	memset(s.ddr, 0, sizeof(s.ddr));
	memset(s.wrote, -1, sizeof(s.wrote));
	map(&s);
	for (frame = 1; frame <= FRAMES; frame++) {
		/* Dequeued: the client works on the previous frame. */
		if (who != CORE)
			check_reads(&s, names[who]);
		if (failures)
			return;
		if (who == WRITER || (who == CHURN && rnd(&s, 3) == 0)) {
			if (touch(&s))
				for (i = 0; i < LINES; i += 1 + rnd(&s, 3))
					user_write(&s, i, 1000000 + frame);
			check_reads(&s, names[who]);
		}
		if (who == CHURN && rnd(&s, 2)) {
			unmap(&s);
			if (rnd(&s, 2)) {
				map(&s);
				check_reads(&s, names[who]);
			}
		}
		background(&s);

		qbuf(&s);
		if (who == CHURN && !s.mapped && rnd(&s, 2))
			map(&s);

		/* Queued: the WDMA writes the frame, or STREAMOFF cuts it. */
		written = streamoff && rnd(&s, 4) == 0 ? (int)rnd(&s, LINES) :
							 LINES;
		for (i = 0; i < written; i++) {
			background(&s);
			if ((who == EARLY || who == CHURN) && rnd(&s, 3) == 0 &&
			    touch(&s))
				user_read(&s, rnd(&s, LINES));
			s.ddr[i] = frame;
			background(&s);
		}
		if (who == CHURN && rnd(&s, 3) == 0)
			unmap(&s);
		for (i = 0; i < LINES; i++)
			background(&s);

		dqbuf(&s);
		for (i = 0; i < written; i++)
			CHECK(s.ddr[i] == frame,
			      "%s: frame %d line %d holds %d after DQBUF",
			      names[who], frame, i, s.ddr[i]);
		if (who == CHURN && !s.mapped)
			map(&s);
	}

	switch (who) {
	case CORE:
		CHECK(!s.faults && !s.cleans && !s.invals,
		      "core: %lu faults %lu cleans %lu invalidates",
		      s.faults, s.cleans, s.invals);
		break;
	case READER:
		CHECK(s.faults == FRAMES && s.invals == FRAMES &&
		      s.cleans == FRAMES,
		      "reader: %lu faults %lu cleans %lu invalidates over %d frames",
		      s.faults, s.cleans, s.invals, FRAMES);
		break;
	default:
		break;
	}
}

int main(void)
{
	int who, ci, so;

	for (who = 0; who < CLIENTS; who++)
		for (ci = 0; ci < 2; ci++)
			for (so = 0; so < 2; so++)
				run(who, ci, so);
	if (failures)
		return 1;
	puts("vin cpu cache: ok");
	return 0;
}
