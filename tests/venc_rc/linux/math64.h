#pragma once
#include <stdint.h>
static inline int64_t div_s64(int64_t a, int32_t b) { return a / b; }
static inline int64_t div64_s64(int64_t a, int64_t b) { return a / b; }
static inline uint64_t div_u64(uint64_t a, uint32_t b) { return a / b; }
