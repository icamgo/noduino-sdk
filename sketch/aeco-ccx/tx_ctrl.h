#ifndef __TX_CTRL_H__
#define __TX_CTRL_H__

#ifdef EFM32HG110F64
#define	CTRL_FIFO_SIZE		32	
#else
#define	CTRL_FIFO_SIZE		8
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

bool is_pkt_in_ctrl(struct ctrl_fifo *cfifo, uint8_t *p, int plen, uint32_t ts)
{
	int a = 0, b = 0;

	uint64_t devid = 0UL;

	uint8_t mtype = p[15] & 0x60;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&devid) + 7 - b) = p[a];
	}

	uint16_t fno = (uint16_t)(p[plen-8] << 8) | p[plen-7];

	for (a = 0; a < CTRL_FIFO_SIZE-1; a++) {

		if (cfifo->ctrl[a].devid == devid) {

			if (cfifo->ctrl[a].fno == fno) {

				return true;
			}

			if ((mtype == 0x00 || mtype == 0x40) && (ts - cfifo->ctrl[a].ts < 10)) {
				// data up pkt, 10s
				return true;
			}
		}
	}

	return false;
}

bool update_ctrl(uint8_t *p, int plen)
{
	// query the devid and update the fno & ts or insert a new item

}

#if 0
int push_ctrl(struct ctrl_fifo *cfifo, uint8_t *idata, uint16_t fno, uint32_t ts)
{
	if (idata != NULL && cfifo != NULL) {

		cfifo->ctrl[cfifo->head].devid, idata, len);

		cfifo->ctrl[cfifo->head].fno = fno;
		cfifo->ctrl[cfifo->head].ts = ts;

		cfifo->head += 1;

		cfifo->head %= CTRL_FIFO_SIZE;

		return 0;
	} else {
		return 1;
	}
}

int get_ctrl(struct ctrl_fifo *cfifo, struct ctrl_t *odata)
{
	if (odata != NULL && cfifo != NULL &&
		((cfifo->head - cfifo->tail) & (CTRL_FIFO_SIZE-1)) > 0) {
	
		memcpy(odata->data, cfifo->ctrl[cfifo->tail].data, cfifo->ctrl[cfifo->tail].plen);

		odata->fno = cfifo->ctrl[cfifo->tail].fno;
		odata->ts = cfifo->ctrl[cfifo->tail].ts;

		cfifo->tail += 1;
		cfifo->tail %= CTRL_FIFO_SIZE;

		return 0;

	} else {
		return 1;
	}
}
#endif

#endif
