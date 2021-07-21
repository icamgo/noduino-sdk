#ifndef __CIRC_BUF_H__
#define __CIRC_BUF_H__

/*
 * ZG: 416 B
 * HG: 2496 B
 * GG: 52 KB
 *
*/

#ifdef EFM32HG110F64
#define	CIRC_BUF_SIZE		256

#elif EFM32GG230F512
#define	CIRC_BUF_SIZE		1024

#elif EFM32ZG110F32
#define	CIRC_BUF_SIZE		64
#endif


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

struct point {
	uint32_t ts;
	int16_t data;
	int8_t iT;
	int8_t wl;
};

struct circ_buf {
	struct point point[CIRC_BUF_SIZE];
	int head;
	int tail;
};

int push_point(struct circ_buf *cbuf, uint32_t ts, int16_t dd, int8_t itt, int8_t wl)
{
	if (cbuf != NULL && CIRC_SPACE(cbuf->head, cbuf->tail, CIRC_BUF_SIZE) >= 1) {

		cbuf->point[cbuf->head].ts = ts;
		cbuf->point[cbuf->head].data = dd;
		cbuf->point[cbuf->head].iT = itt;
		cbuf->point[cbuf->head].wl = wl;

		cbuf->head += 1;

		cbuf->head %= CIRC_BUF_SIZE;

		return 0;

	} else {
		/*
		 * cbuf is full
		 * replace a random point
		*/
		randomSeed(millis());
		uint32_t pos = random() % CIRC_BUF_SIZE;

		cbuf->point[pos].ts = ts;
		cbuf->point[pos].data = dd;
		cbuf->point[pos].iT = itt;

		return 1;
	}
}

int pop_point(struct circ_buf *cbuf, struct point *odata)
{
	if (odata != NULL && cbuf != NULL &&
		CIRC_CNT(cbuf->head, cbuf->tail, CIRC_BUF_SIZE) > 0) {
	
		odata->ts = cbuf->point[cbuf->tail].ts;
		odata->data = cbuf->point[cbuf->tail].data;
		odata->iT = cbuf->point[cbuf->tail].iT;
		odata->wl = cbuf->point[cbuf->tail].wl;

		cbuf->tail += 1;
		cbuf->tail %= CIRC_BUF_SIZE;

		return 0;

	} else {
		return 1;
	}
}

int get_1st_point(struct circ_buf *cbuf, struct point *odata)
{
	if (odata != NULL && cbuf != NULL &&
		CIRC_CNT(cbuf->head, cbuf->tail, CIRC_BUF_SIZE) > 0) {
	
		odata->ts = cbuf->point[cbuf->tail].ts;
		odata->data = cbuf->point[cbuf->tail].data;
		odata->iT = cbuf->point[cbuf->tail].iT;
		odata->wl = cbuf->point[cbuf->tail].wl;

		return 0;

	} else {
		return 1;
	}
}

int pop_head_point(struct circ_buf *cbuf, struct point *odata)
{
	if (odata != NULL && cbuf != NULL &&
		CIRC_CNT(cbuf->head, cbuf->tail, CIRC_BUF_SIZE) > 0) {

		int pos = (cbuf->head - 1) & (CIRC_BUF_SIZE - 1);

		odata->ts = cbuf->point[pos].ts;
		odata->data = cbuf->point[pos].data;
		odata->iT = cbuf->point[pos].iT;
		odata->wl = cbuf->point[pos].wl;

		cbuf->head = pos;

		return 0;

	} else {
		return 1;
	}
}

int get_head_point(struct circ_buf *cbuf, struct point *odata)
{
	if (odata != NULL && cbuf != NULL &&
		CIRC_CNT(cbuf->head, cbuf->tail, CIRC_BUF_SIZE) > 0) {

		int pos = (cbuf->head - 1) & (CIRC_BUF_SIZE - 1);

		odata->ts = cbuf->point[pos].ts;
		odata->data = cbuf->point[pos].data;
		odata->iT = cbuf->point[pos].iT;
		odata->wl = cbuf->point[pos].wl;

		return 0;

	} else {
		return 1;
	}
}
#endif
