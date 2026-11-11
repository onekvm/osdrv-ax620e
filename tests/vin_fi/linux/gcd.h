#pragma once
static inline unsigned long gcd(unsigned long a, unsigned long b)
{
	while (b) {
		unsigned long t = a % b;

		a = b;
		b = t;
	}
	return a;
}
