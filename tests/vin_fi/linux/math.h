#pragma once
#define abs_diff(a, b) ((a) > (b) ? (a) - (b) : (b) - (a))
#define DIV_ROUND_CLOSEST(x, d) (((x) + ((d) / 2)) / (d))
#define DIV_ROUND_CLOSEST_ULL(x, d) DIV_ROUND_CLOSEST((unsigned long long)(x), (d))
