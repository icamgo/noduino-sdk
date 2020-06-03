/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/
#ifndef __PT1000_H__
#define __PT1000_H__

#include "Arduino.h"

static uint32_t PT100_TABLE[100] = {
	1852,2068,2283,2497,2710,2922,3134,3344,3554,3764,				// -200 ~ -155
	3972,4180,4388,4594,4800,5006,5211,5415,5619,5823,				// -150 ~ -105
	6026,6228,6430,6631,6833,7033,7233,7433,7633,7832,				// -100 ~ -55
	8031,8229,8427,8625,8822,9019,9216,9412,9609,9804,				//  -50 ~ -5
	10000,10195,10390,10585,10779,10973,11167,11361,11554,11747,	// 0   ~ 45
	11940,12132,12324,12516,12708,12899,13090,13280,13471,13661,	// 50  ~ 95
	13851,14040,14229,14418,14607,14795,14983,15171,15358,15546,	// 100 ~ 145
	15733,15919,16105,16291,16477,16663,16848,17033,17217,17402,	// 150 ~ 195
	17586,17769,17953,18136,18319,18501,18684,18866,19047,19229,	// 200 ~ 245
	19410,19591,19771,19951,20131,20311,20490,20670,20848,21027,	// 250 ~ 295
/*
	21205,21383,21561,21738,21915,22092,22268,22445,22621,22796,	// 300 ~ 345
	22972,23147,23321,23496,23670,23844,24018,24191,24364,24537,	// 350 ~ 395
	24709,24881,25053,25225,25396,25567,25738,25908,26078,26248,	// 400 ~ 445
	26418,26587,26756,26925,27093,27261,27429,27597,27764,27931,	// 450 ~ 495
	28098,28264,28430,28596,28762,28927,29092,29256,29421,29585,	// 500 ~ 545
	29749,29912,30075,30238,30401,30563,30725,30887,31049,31210,	// 550 ~ 595
	31371,31531,31692,31852,32012,32171,32330,32489,32648,32806,	// 600 ~ 645
	32964,33122,33279,33436,33593,33750,33906,34062,34218,34373,	// 650 ~ 695
	34528,34683,34838,34992,35146,35300,35453,35606,35759,35912,	// 700 ~ 745
	36064,36216,36367,36519,36670,36821,36971,37121,37271,37421,	// 750 ~ 795
	37570,37719,37868,38017,38165,38313,38460,38608,38755,38902,	// 800 ~ 845
	39048															// 850
*/
};

#define PT_1MS			(F_CPU/1000)		/* 14MHz, 1Tick = 1/14 us, 14000 Tick = 1000us */
#define pt_delay(x)		do{for(int i=0;i<x;i++) {asm volatile("nop");}}while(0)

#define	N_TRY			18

void pt1000_init();
uint32_t pt1000_get_rt();
float pt1000_get_temp();

float cal_temp(uint32_t Rt);
#endif
