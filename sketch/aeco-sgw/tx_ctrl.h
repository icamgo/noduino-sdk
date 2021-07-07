#ifndef __TX_CTRL_H__
#define __TX_CTRL_H__

#ifdef EFM32HG110F64
#define	CTRL_FIFO_SIZE		32
#elif EFM32GG230F512
#define	CTRL_FIFO_SIZE		1024
#endif

struct ctrl_t {

	uint64_t devid;
	uint16_t fno;
	uint32_t ts;

};

struct ctrl_fifo {
	struct ctrl_t ctrl[CTRL_FIFO_SIZE];
	int head;
	int tail;
};

bool check_ctrl_fno(struct ctrl_fifo *cfifo, uint8_t *p, int plen)
{
	int a = 0, b = 0;

	uint64_t devid = 0UL;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&devid) + 7 - b) = p[a];
	}

	uint16_t fno = (uint16_t)(p[plen-8] << 8) | p[plen-7];

	for (a = 0; a < CTRL_FIFO_SIZE-1; a++) {
		
		if (cfifo->ctrl[a].devid == devid) {

			if (cfifo->ctrl[a].fno != fno) {

				// update the ctrl structure
				cfifo->ctrl[a].fno = fno;
				cfifo->ctrl[a].ts = seconds();

				return true;

			} else {

				return false;
			}
		}
	}

	// no exist ctrl data, insert a item
	cfifo->ctrl[cfifo->head].devid = devid;
	cfifo->ctrl[cfifo->head].fno = fno;
	cfifo->ctrl[cfifo->head].ts = seconds();

	cfifo->head += 1;
	cfifo->head %= CTRL_FIFO_SIZE;

	return true;
}

void reset_ctrl_ts(struct ctrl_fifo *cfifo, uint32_t ts)
{
	for (int a = 0; a < CTRL_FIFO_SIZE-1; a++) {

		cfifo->ctrl[a].ts = ts;
	}
}

bool is_pkt_in_ctrl(struct ctrl_fifo *cfifo, uint8_t *p, int plen, uint32_t ts)
{
	int a = 0, b = 0;

	uint64_t devid = 0UL;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&devid) + 7 - b) = p[a];
	}

	uint16_t fno = (uint16_t)(p[plen-8] << 8) | p[plen-7];

	for (a = 0; a < CTRL_FIFO_SIZE-1; a++) {

		if (cfifo->ctrl[a].devid == devid) {

			if (cfifo->ctrl[a].fno == fno) {
				// same pkt -> drop the pkt
				return true;
			}

		#if 0
			uint8_t mtype = p[plen-9] & 0xE0;
			if (0x33 == p[2] && 36 == plen && (mtype == 0x20 || mtype == 0x60)) {
				// data down pkt, no limit
				return false;
			}
		#endif

			if ((ts - cfifo->ctrl[a].ts < 20)) {
				// no data up/down flag, < 20s -> drop the pkt
				return true;
			}
		}
	}

	// ok
	return false;
}
#endif
