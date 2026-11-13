// SPDX-License-Identifier: GPL-2.0-only
/* Device test: cacheable MMAP of VIN capture buffers on a lazy-cma device tree.
 * Run with the video service stopped and a limited-range input signal present.
 *
 *   untouched  buffers mapped but never read cost no fault or cache operation;
 *   read       every dequeued frame read through the cached mapping equals the
 *              uncached view of the same buffer (its exported dma-buf);
 *   poison     after reading, the whole buffer is overwritten with a Y=1
 *              pattern (never in a limited-range frame) through the cached
 *              mapping before QBUF; no poison may reach the next frame, in
 *              DDR or in the cache;
 *   cost       one fault, one invalidate and one clean per frame.
 *
 * Also prints how long a memcpy of one frame takes through each mapping.
 * usage: ax620e-vin-cached-mmap-test [device] [frames]
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>

#define NBUF	4
#define POISON	0x04030201u	/* bytes 01 02 03 04: Y=1, below limited range */

struct stats {
	long faults, invalidates, cleans;
};

static int failed;

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s (errno %d)\n", __FILE__, __LINE__, #expr, errno); \
	failed = 1; goto out; } } while (0)

static double now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static long read_long(const char *path)
{
	FILE *f = fopen(path, "r");
	long v = -1;

	if (f) {
		if (fscanf(f, "%ld", &v) != 1)
			v = -1;
		fclose(f);
	}
	return v;
}

static int read_stats(const char *path, struct stats *s)
{
	FILE *f = fopen(path, "r");
	int n;

	if (!f)
		return -1;
	n = fscanf(f, "faults %ld invalidates %ld cleans %ld",
		   &s->faults, &s->invalidates, &s->cleans);
	fclose(f);
	return n == 3 ? 0 : -1;
}

static int dqbuf(int fd, struct v4l2_buffer *b)
{
	struct pollfd p = { .fd = fd, .events = POLLIN };

	if (poll(&p, 1, 2000) != 1)
		return -1;
	memset(b, 0, sizeof(*b));
	b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	b->memory = V4L2_MEMORY_MMAP;
	return ioctl(fd, VIDIOC_DQBUF, b);
}

static int qbuf(int fd, unsigned int index)
{
	struct v4l2_buffer b = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
		.index = index,
	};

	return ioctl(fd, VIDIOC_QBUF, &b);
}

static int cmp_double(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;

	return (x > y) - (x < y);
}

int main(int argc, char **argv)
{
	const char *dev = argc > 1 ? argv[1] : "/dev/onekvm-vin";
	int frames = argc > 2 ? atoi(argv[2]) : 120;
	char node[PATH_MAX], stats_path[PATH_MAX + 64];
	unsigned char *cached[NBUF], *uncached[NBUF], *copy = NULL, *ref = NULL;
	int dmabuf[NBUF], fd = -1, streaming = 0, i, f, n = 0;
	double *t_cached = NULL, *t_uncached = NULL;
	struct stats s0, s1, s2;
	struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
	struct v4l2_requestbuffers req = {
		.count = NBUF,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
	};
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	struct v4l2_buffer b;
	size_t size = 0, w;
	long width, height;

	for (i = 0; i < NBUF; i++) {
		cached[i] = uncached[i] = MAP_FAILED;
		dmabuf[i] = -1;
	}
	CHECK(frames > 0);
	CHECK(realpath(dev, node));
	snprintf(stats_path, sizeof(stats_path),
		 "/sys/class/video4linux/%s/device/cached_mmap_stats",
		 strrchr(node, '/') + 1);
	CHECK(read_stats(stats_path, &s0) == 0);

	/* A format larger than the input never completes a frame. */
	width = read_long("/proc/lt6911_info/width");
	height = read_long("/proc/lt6911_info/height");
	CHECK(width > 0 && height > 0);
	fd = open(dev, O_RDWR | O_NONBLOCK);
	CHECK(fd >= 0);
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_NONE;
	CHECK(ioctl(fd, VIDIOC_S_FMT, &fmt) == 0);
	size = fmt.fmt.pix.sizeimage;
	CHECK(ioctl(fd, VIDIOC_REQBUFS, &req) == 0 && req.count == NBUF);
	for (i = 0; i < NBUF; i++) {
		struct v4l2_exportbuffer exp = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.index = i,
			.flags = O_RDWR,
		};

		memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		b.memory = V4L2_MEMORY_MMAP;
		b.index = i;
		CHECK(ioctl(fd, VIDIOC_QUERYBUF, &b) == 0);
		cached[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE,
				 MAP_SHARED, fd, b.m.offset);
		CHECK(cached[i] != MAP_FAILED);
		CHECK(ioctl(fd, VIDIOC_EXPBUF, &exp) == 0);
		dmabuf[i] = exp.fd;
		uncached[i] = mmap(NULL, b.length, PROT_READ, MAP_SHARED,
				   exp.fd, 0);
		CHECK(uncached[i] != MAP_FAILED);
		CHECK(qbuf(fd, i) == 0);
	}
	copy = malloc(size);
	ref = malloc(size);
	t_cached = calloc(frames, sizeof(double));
	t_uncached = calloc(frames, sizeof(double));
	CHECK(copy && ref && t_cached && t_uncached);
	CHECK(ioctl(fd, VIDIOC_STREAMON, &type) == 0);
	streaming = 1;

	/* untouched: what Core does with its mappings. */
	for (f = 0; f < 60; f++) {
		CHECK(dqbuf(fd, &b) == 0);
		CHECK(qbuf(fd, b.index) == 0);
	}
	CHECK(read_stats(stats_path, &s1) == 0);
	CHECK(s1.faults == s0.faults && s1.invalidates == s0.invalidates &&
	      s1.cleans == s0.cleans);

	for (f = 0; f < frames; f++) {
		double t;

		CHECK(dqbuf(fd, &b) == 0);
		CHECK(b.bytesused == size);
		t = now_ms();
		memcpy(copy, cached[b.index], size);
		t_cached[f] = now_ms() - t;
		t = now_ms();
		memcpy(ref, uncached[b.index], size);
		t_uncached[f] = now_ms() - t;
		CHECK(memcmp(copy, ref, size) == 0);
		for (w = 0; w < size / 4; w++)
			CHECK(((uint32_t *)copy)[w] != POISON);
		for (w = 0; w < size / 4; w++)
			((uint32_t *)cached[b.index])[w] = POISON;
		CHECK(qbuf(fd, b.index) == 0);
		n++;
	}
	CHECK(read_stats(stats_path, &s2) == 0);
	CHECK(s2.faults - s1.faults == frames);
	CHECK(s2.invalidates - s1.invalidates == frames);
	CHECK(s2.cleans - s1.cleans == frames);

	qsort(t_cached, frames, sizeof(double), cmp_double);
	qsort(t_uncached, frames, sizeof(double), cmp_double);
	printf("vin-cached-mmap: %ldx%ld, %d frames: untouched 0 faults, cached == uncached, no poison, 1 fault/invalidate/clean per frame: PASS\n",
	       width, height, frames);
	printf("memcpy %zu B: cached (incl. fault+invalidate) P50 %.2f ms P95 %.2f ms; uncached P50 %.2f ms P95 %.2f ms\n",
	       size, t_cached[frames / 2], t_cached[frames * 95 / 100],
	       t_uncached[frames / 2], t_uncached[frames * 95 / 100]);
out:
	if (failed && n)
		fprintf(stderr, "failed after %d checked frames\n", n);
	if (streaming)
		ioctl(fd, VIDIOC_STREAMOFF, &type);
	for (i = 0; i < NBUF; i++) {
		if (cached[i] != MAP_FAILED)
			munmap(cached[i], size);
		if (uncached[i] != MAP_FAILED)
			munmap(uncached[i], size);
		if (dmabuf[i] >= 0)
			close(dmabuf[i]);
	}
	if (fd >= 0)
		close(fd);
	free(copy);
	free(ref);
	free(t_cached);
	free(t_uncached);
	return failed;
}
