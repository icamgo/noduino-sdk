#ifndef __CIRC_BUF_H__
#define __CIRC_BUF_H__

#define	CIRC_BUF_SIZE		6

struct pkt {
/*
	char head[2];
	uint8_t ver;
	uint64_t did;
	int16_t val;
	int16_t vbat;
*/
	uint8_t data[24];
	int16_t rssi;
};

struct circ_buf {
	struct pkt pkt[CIRC_BUF_SIZE];
	int head;
	int tail;
};

int push_pkt(struct circ_buf *cbuf, uint8_t *idata, int16_t rssi)
{
	if (idata != NULL && cbuf != NULL) {

		memcpy(cbuf->pkt[cbuf->head].data, idata, 24);

		cbuf->pkt[cbuf->head].rssi = rssi;

		cbuf->head += 1;

		cbuf->head %= CIRC_BUF_SIZE;

		return 0;
	} else {
		return 1;
	}
}

int get_pkt(struct circ_buf *cbuf, struct pkt *odata)
{
	if (odata != NULL && cbuf != NULL &&
		((cbuf->head - cbuf->tail) & (CIRC_BUF_SIZE-1)) > 0) {
	
		memcpy(odata->data, cbuf->pkt[cbuf->tail].data, 24);

		odata->rssi = cbuf->pkt[cbuf->tail].rssi;

		cbuf->tail += 1;
		cbuf->tail %= CIRC_BUF_SIZE;

		return 0;

	} else {
		return 1;
	}
}

/* Return count in buffer.  */
#define CIRC_CNT(head,tail,size) (((head) - (tail)) & ((size)-1))

/* Return space available, 0..size-1.  We always leave one free char
   as a completely full buffer has head == tail, which is the same as
   empty.  */
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))

/* Return count up to the end of the buffer.  Carefully avoid
   accessing head and tail more than once, so they can change
   underneath us without returning inconsistent results.  */
#define CIRC_CNT_TO_END(head,tail,size) \
	({int end = (size) - (tail); \
	  int n = ((head) + end) & ((size)-1); \
	  n < end ? n : end;})

/* Return space available up to the end of the buffer.  */
#define CIRC_SPACE_TO_END(head,tail,size) \
	({int end = (size) - 1 - (head); \
	  int n = (end + (tail)) & ((size)-1); \
	  n <= end ? n : end+1;})

#endif
