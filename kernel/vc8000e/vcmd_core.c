// SPDX-License-Identifier: GPL-2.0-only
/* Adapt the port's mapping call without editing the vendored VCMD source. */
#include <linux/mm.h>
#include "framebuf_alloc.h"

#define remap_pfn_range vcmd_fb_remap_pfn_range
#include "eswin/vc8000_vcmd_driver.c"
