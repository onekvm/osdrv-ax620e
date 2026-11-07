/* Host stand-in for the kernel types ax630c_venc_rc.h uses. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef int64_t s64;
typedef uint64_t u64;
typedef int32_t s32;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
#define clamp(v, lo, hi) ((v) < (lo) ? (lo) : (v) > (hi) ? (hi) : (v))
#define DIV_ROUND_CLOSEST(x, d) (((x) + ((d) / 2)) / (d))
