// SPDX-License-Identifier: GPL-2.0-only
/*
 * Host check of the kernel encoder's rate control (ax630c_venc_rc.h, integer)
 * against the user-space controller it ports (vcenc_rc.h, double; copied
 * from onekvm-video-ax630c third_party/vcenc-ewl at ab7feafd). Both run on
 * the same synthetic frame sizes; every frame QP must match. Frame sizes
 * follow a QP-dependent model with scene changes and a mid-run bitrate cut.
 *
 * The extreme cases only check bounds: there a size can land exactly on a
 * ratio threshold, where the double version rounds below it and the integer
 * cross product does not, and the two runs part from that frame on.
 */
#include <math.h>
#include <stdio.h>
#include "vcenc_rc.h"
#include "ax630c_venc_rc.h"

static uint64_t rng = 88172645463325252ULL;

static double urand(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 7;
	rng ^= rng << 17;
	return (rng >> 11) * (1.0 / 9007199254740992.0);
}

static double frame_bits(int idr, uint32_t qp, double act, uint32_t w, uint32_t h)
{
	double b = (idr ? 0.35 : 0.035) * w * h * act;

	return b * pow(2.0, (32.0 - qp) / 6.0) * (0.9 + 0.2 * urand());
}

struct scenario { uint32_t w, h, fps, gop, mode; double bitrate; int exact; };

static const struct scenario sc[] = {
	{ 1920, 1080, 60, 60, VENC_RC_CBR, 8e6, 1 },
	{ 1920, 1080, 60, 30, VENC_RC_CBR, 2e6, 1 },
	{ 1920, 1080, 60, 0, VENC_RC_CBR, 4e6, 1 },
	{ 1920, 1080, 30, 120, VENC_RC_CBR, 16e6, 1 },
	{ 3840, 2160, 30, 60, VENC_RC_CBR, 20e6, 1 },
	{ 1280, 720, 60, 60, VENC_RC_VBR, 6e6, 1 },
	{ 1920, 1080, 60, 60, VENC_RC_VBR, 2e6, 1 },
	{ 1366, 768, 60, 1, VENC_RC_CBR, 8e6, 1 },
	{ 640, 480, 25, 25, VENC_RC_CBR, 1e6, 1 },
	{ 1920, 1080, 60, 60, VENC_RC_VBR, 16e6, 1 },
	{ 3840, 2160, 30, 0, VENC_RC_VBR, 8e6, 1 },
	{ 3840, 2160, 60, 60, VENC_RC_CBR, 1e5, 0 },
	{ 640, 480, 240, 240, VENC_RC_CBR, 5e7, 0 },
	{ 4096, 2400, 1, 1, VENC_RC_VBR, 1e5, 0 },
	{ 64, 64, 60, 7, VENC_RC_CBR, 1e5, 0 },
	{ 1920, 1080, 120, 120, VENC_RC_VBR, 5e7, 0 },
	{ 2560, 1440, 50, 0, VENC_RC_CBR, 12e6, 0 },
};

int main(void)
{
	int fail = 0, compared = 0;

	for (unsigned s = 0; s < sizeof(sc) / sizeof(sc[0]); s++) {
		for (uint32_t codec = 0; codec < 2; codec++) {
			const struct scenario *c = &sc[s];
			struct vcenc_rc a;
			struct venc_rc b;
			uint32_t gopn = 0;
			double act = 1.0;

			vcenc_rc_init(&a, codec, c->w, c->h, c->fps, c->bitrate, c->gop, c->mode);
			venc_rc_init(&b, codec, c->w, c->h, c->fps, (s64)c->bitrate, c->gop,
				     c->mode, VCENC_QP_MIN, VCENC_QP_MAX);
			for (int f = 0; f < 3000; f++) {
				int idr;
				uint32_t qa, qb;
				double bits;

				if (f % 400 == 0)
					act = 0.05 + 3.0 * urand();
				if (f == 1500) {
					vcenc_rc_set_target(&a, c->bitrate / 2, c->fps);
					venc_rc_set_target(&b, (s64)(c->bitrate / 2), c->fps);
				}
				idr = !gopn || (c->gop && gopn >= c->gop);
				if (idr)
					gopn = 0;
				qa = vcenc_rc_pick_qp(&a, idr);
				qb = venc_rc_pick_qp(&b, idr);
				if (qb < VCENC_QP_MIN || qb > VCENC_QP_MAX) {
					printf("case %u codec %u frame %d: QP %u out of range\n",
					       s, codec, f, qb);
					fail = 1;
					break;
				}
				if (c->exact) {
					compared++;
					if (qa != qb) {
						printf("case %u codec %u frame %d: QP %u, reference %u\n",
						       s, codec, f, qb, qa);
						fail = 1;
						break;
					}
				}
				bits = floor(frame_bits(idr, qa, act, c->w, c->h));
				vcenc_rc_update(&a, bits, qa);
				venc_rc_update(&b, (s64)bits, qb);
				gopn++;
			}
		}
	}
	printf("venc rc: %d frames compared, %s\n", compared, fail ? "FAIL" : "ok");
	return fail;
}
