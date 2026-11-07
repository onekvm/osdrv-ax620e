#pragma once
static inline int fls64(unsigned long long x) { return x ? 64 - __builtin_clzll(x) : 0; }
