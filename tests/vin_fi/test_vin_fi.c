// SPDX-License-Identifier: GPL-2.0
/*
 * Host test for kernel/ovc_frame_interval.h: feeds simulated VIN frame-done
 * stamps (IRQ latency jitter, lost frame-dones, stalls, rate changes) and
 * checks the interval G_PARM would report.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "ovc_frame_interval.h"

static int fails;

struct sim {
	struct ovc_fi fi;
	double t;		/* input frame clock, ns */
	unsigned int rng;
	/* every interval reported while running, for "never shows X" checks */
	struct v4l2_fract seen[8];
	unsigned int nseen;
};

static unsigned int rnd(struct sim *s)
{
	s->rng = s->rng * 1664525u + 1013904223u;
	return s->rng >> 8;
}

static void note(struct sim *s)
{
	unsigned int i;

	for (i = 0; i < s->nseen; i++)
		if (s->seen[i].numerator == s->fi.tpf.numerator &&
		    s->seen[i].denominator == s->fi.tpf.denominator)
			return;
	if (s->nseen < 8)
		s->seen[s->nseen++] = s->fi.tpf;
}

/*
 * @secs of input at @hz. Each frame-done lands up to @jit_ns after its frame
 * ends; every @miss-th one is lost (0: none), or each with probability
 * @loss_ppm / 1e6.
 */
static void run(struct sim *s, double hz, double secs, unsigned int jit_ns,
		unsigned int miss, unsigned int loss_ppm)
{
	double period = 1e9 / hz;
	long n = lround(secs * hz), k;

	for (k = 1; k <= n; k++) {
		s->t += period;
		if (miss && k % miss == 0)
			continue;
		if (loss_ppm && rnd(s) % 1000000 < loss_ppm)
			continue;
		ovc_fi_sample(&s->fi, (u64)s->t + (jit_ns ? rnd(s) % jit_ns : 0));
		note(s);
	}
}

static struct sim *sim_new(void)
{
	static struct sim s;

	s = (struct sim){ .t = 5e9, .rng = 12345 };
	return &s;
}

static void expect(const char *what, struct v4l2_fract got,
		   unsigned int num, unsigned int den)
{
	if (got.numerator == num && got.denominator == den)
		return;
	printf("FAIL %s: %u/%u, want %u/%u\n", what, got.numerator,
	       got.denominator, num, den);
	fails++;
}

static void expect_rate(const char *what, struct v4l2_fract got, double hz)
{
	double r = got.numerator ? (double)got.denominator / got.numerator : 0;

	if (fabs(r / hz - 1) <= OVC_FI_TOL_PPM * 1e-6)
		return;
	printf("FAIL %s: %u/%u = %.4f Hz, want %.4f\n", what, got.numerator,
	       got.denominator, r, hz);
	fails++;
}

/* Only these intervals (and 0/0) may have been reported during the run. */
static void expect_only(const char *what, struct sim *s,
			const struct v4l2_fract *ok, unsigned int nok)
{
	unsigned int i, j;

	for (i = 0; i < s->nseen; i++) {
		if (!s->seen[i].denominator)
			continue;
		for (j = 0; j < nok; j++)
			if (s->seen[i].numerator == ok[j].numerator &&
			    s->seen[i].denominator == ok[j].denominator)
				break;
		if (j == nok) {
			printf("FAIL %s: reported %u/%u\n", what,
			       s->seen[i].numerator, s->seen[i].denominator);
			fails++;
		}
	}
}

static void test_nominal(void)
{
	expect("60.000", ovc_fi_nominal(60000), 1, 60);
	expect("59.940", ovc_fi_nominal(59940), 1001, 60000);
	expect("59.970 tie -> integer", ovc_fi_nominal(59970), 1, 60);
	expect("59.969", ovc_fi_nominal(59969), 1001, 60000);
	expect("29.970", ovc_fi_nominal(29970), 1001, 30000);
	expect("47.500", ovc_fi_nominal(47500), 2, 95);
	expect("1.000", ovc_fi_nominal(1000), 1, 1);
	expect("0.001", ovc_fi_nominal(1), 1000, 1);
}

static void test_rates(void)
{
	static const struct {
		const char *name;
		double hz;
		unsigned int num, den;	/* 0/0: check the rate only */
	} t[] = {
		{ "60", 60, 1, 60 },
		{ "59.94", 60000.0 / 1001, 1001, 60000 },
		{ "CVT-RB 1080p 59.934", 138.5e6 / (2080 * 1111), 1001, 60000 },
		{ "50", 50, 1, 50 },
		{ "30", 30, 1, 30 },
		{ "29.97", 30000.0 / 1001, 1001, 30000 },
		{ "25", 25, 1, 25 },
		{ "24", 24, 1, 24 },
		{ "23.976", 24000.0 / 1001, 1001, 24000 },
		{ "75", 75, 1, 75 },
		{ "120", 120, 1, 120 },
		{ "144", 144, 1, 144 },
		{ "240", 240, 1, 240 },
		{ "47.5", 47.5, 0, 0 },
		{ "59.81", 59.81, 0, 0 },
	};
	char what[96];
	unsigned int i;

	for (i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
		struct sim *s = sim_new();

		/* 200 us IRQ latency jitter, one frame-done in 97 lost */
		run(s, t[i].hz, 10, 200000, 97, 0);
		snprintf(what, sizeof(what), "%s Hz", t[i].name);
		if (t[i].den)
			expect(what, s->fi.tpf, t[i].num, t[i].den);
		else
			expect_rate(what, s->fi.tpf, t[i].hz);
	}
}

static void test_lifecycle(void)
{
	static const struct v4l2_fract r60 = { 1, 60 };
	static const struct v4l2_fract r60_30[] = { { 1, 60 }, { 1, 30 } };
	struct sim *s = sim_new();

	run(s, 60, 3.9, 50000, 0, 0);
	expect("unknown before two windows", s->fi.tpf, 0, 0);
	run(s, 60, 0.3, 50000, 0, 0);
	expect("two windows", s->fi.tpf, 1, 60);

	/* STREAMOFF/STREAMON keeps the last value until a new one agrees */
	ovc_fi_start(&s->fi);
	s->t += 10e9;
	run(s, 30, 3.5, 50000, 0, 0);
	expect("restart keeps the old rate", s->fi.tpf, 1, 60);
	run(s, 30, 1, 50000, 0, 0);
	expect("restart at a new rate", s->fi.tpf, 1, 30);

	/* rate changes inside one stream, both ways */
	s = sim_new();
	run(s, 60, 6, 100000, 0, 0);
	run(s, 30, 12, 100000, 0, 0);
	expect("60 -> 30 in stream", s->fi.tpf, 1, 30);
	run(s, 60, 12, 100000, 0, 0);
	expect("30 -> 60 in stream", s->fi.tpf, 1, 60);
	expect_only("in-stream changes", s, r60_30, 2);

	/* a stall, then the signal back at another phase */
	s = sim_new();
	run(s, 60, 6, 100000, 0, 0);
	s->t += 3e9 + 7.3e6;
	run(s, 60, 6, 100000, 0, 0);
	expect("stall", s->fi.tpf, 1, 60);
	expect_only("stall", s, &r60, 1);

	/* heavy latency, every third frame-done lost, random 1% loss */
	s = sim_new();
	run(s, 60, 20, 400000, 0, 0);
	expect_only("400 us jitter", s, &r60, 1);
	expect("400 us jitter", s->fi.tpf, 1, 60);
	s = sim_new();
	run(s, 60, 20, 200000, 3, 0);
	expect_only("every 3rd lost", s, &r60, 1);
	expect("every 3rd lost", s->fi.tpf, 1, 60);
	s = sim_new();
	run(s, 60, 60, 300000, 0, 10000);
	expect_only("1% lost", s, &r60, 1);
	expect("1% lost", s->fi.tpf, 1, 60);
}

int main(void)
{
	test_nominal();
	test_rates();
	test_lifecycle();
	if (fails) {
		printf("%d failures\n", fails);
		return 1;
	}
	printf("vin frame interval: ok\n");
	return 0;
}
