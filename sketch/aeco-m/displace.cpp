#include "displace.h"
#include "math.h"

static uint32_t T[12] = {
	   0,   2,   6,  14,   30,
	  62, 126, 254, 509, 1022,
	2042, 4075
};

#define	N		10

static float r[N] = { 0 };
static int start = 0, end = 0;

void push(float in)
{
	r[start++] = in;

	start %= N;
}

/*
 *
 */
int displace(uint32_t r)
{
	float rt = roundf(r / 10.0);

	if (r == 30000) {
		// infinity Rt
		return 55;
	}

	if (rt >= T[0] && rt < T[1])
		return 0;

	if (rt >= T[1] && rt < T[2]-1)
		return 1;

	if (rt >= T[2]-1 && rt < T[3]-1)
		return 2;

	if (rt >= T[3]-1 && rt < T[4]-1)
		return 3;

	if (rt >= T[4]-1 && rt < T[5]-1)
		return 4;

	if (rt >= T[5]-1 && rt < T[6]-2)
		return 5;

	if (rt >= T[6]-2 && rt < T[7]-4)
		return 6;

	if (rt >= T[7]-4 && rt < T[8]-4)
		return 7;

	if (rt >= T[8]-4 && rt < T[9]-8)
		return 8;

	if (rt >= T[9]-8 && rt < T[10]-8)
		return 9;

	if (rt >= T[10]-8 && rt < T[11]-8)
		return 10;

	if (rt >= T[11]-8)
		return 11;
}
