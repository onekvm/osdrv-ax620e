// SPDX-License-Identifier: GPL-2.0-only
/* Device test: a freed ioctl allocation must remain pinned by its user VMA.
 * Run with the video service stopped, on a lazy-dma device tree.
 */
#define _FILE_OFFSET_BITS 64
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "../kernel/vc8000e/framebuf_alloc.h"

#define BYTES (1024UL * 1024)
#define STATS "/sys/devices/platform/onekvm-lazy-dma/stats"
#define RECLAIM "/sys/devices/platform/onekvm-lazy-dma/reclaim"

static unsigned long long used(void)
{
	char key[80];
	unsigned long long value;
	FILE *f = fopen(STATS, "r");

	if (!f)
		return ~0ULL;
	while (fscanf(f, "%79s %llu", key, &value) == 2) {
		if (!strcmp(key, "in_use_bytes")) {
			fclose(f);
			return value;
		}
	}
	fclose(f);
	return ~0ULL;
}

static int reclaim(void)
{
	int fd = open(RECLAIM, O_WRONLY);
	int ret;

	if (fd < 0)
		return -1;
	ret = write(fd, "1", 1) == 1 ? 0 : -1;
	close(fd);
	return ret;
}

int main(void)
{
	struct framebuf_parameter first = {.size = BYTES}, second = {.size = BYTES};
	unsigned long long before = used();
	volatile unsigned char *map = MAP_FAILED;
	void *alien;
	int fd = -1, other = -1, have_first = 0, have_second = 0, ret = 1;
	unsigned long i;

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s (errno %d)\n", __FILE__, __LINE__, #expr, errno); \
	goto out; } } while (0)
	CHECK(before != ~0ULL);
	fd = open("/dev/es_venc", O_RDWR);
	CHECK(fd >= 0);
	CHECK(ioctl(fd, HANTRO_IOCH_ALLOC_FRAMEBUF, &first) == 0);
	have_first = 1;
	map = mmap(NULL, BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, first.bus_addr);
	CHECK(map != MAP_FAILED);
	for (i = 0; i < BYTES; i++)
		CHECK(map[i] == 0);
	for (i = 0; i < BYTES; i++)
		map[i] = (unsigned char)(i * 17 + 3);
	other = open("/dev/es_venc", O_RDWR);
	CHECK(other >= 0);
	alien = mmap(NULL, BYTES, PROT_READ, MAP_SHARED, other, first.bus_addr);
	if (alien != MAP_FAILED)
		munmap(alien, BYTES);
	CHECK(alien == MAP_FAILED && errno == EPERM);
	CHECK(ioctl(fd, HANTRO_IOCH_FREE_FRAMEBUF, &first) == 0);
	have_first = 0;
	CHECK(reclaim() == 0);
	CHECK(used() == before + BYTES);
	CHECK(ioctl(fd, HANTRO_IOCH_ALLOC_FRAMEBUF, &second) == 0);
	have_second = 1;
	CHECK(second.bus_addr != first.bus_addr);
	CHECK(ioctl(fd, HANTRO_IOCH_FREE_FRAMEBUF, &second) == 0);
	have_second = 0;
	close(fd);
	fd = -1;
	CHECK(reclaim() == 0);
	CHECK(used() == before + BYTES);
	for (i = 0; i < BYTES; i++)
		CHECK(map[i] == (unsigned char)(i * 17 + 3));
	CHECK(munmap((void *)map, BYTES) == 0);
	map = MAP_FAILED;
	CHECK(reclaim() == 0 && used() == before);
	puts("dma-lifetime: zeroed allocation, owner check, FREE/close/reclaim with live VMA: PASS");
	ret = 0;
out:
	if (map != MAP_FAILED)
		munmap((void *)map, BYTES);
	if (have_second)
		ioctl(fd, HANTRO_IOCH_FREE_FRAMEBUF, &second);
	if (have_first)
		ioctl(fd, HANTRO_IOCH_FREE_FRAMEBUF, &first);
	if (other >= 0)
		close(other);
	if (fd >= 0)
		close(fd);
	return ret;
}
