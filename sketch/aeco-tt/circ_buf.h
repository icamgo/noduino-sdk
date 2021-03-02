#ifndef __CIRC_BUF_H__
#define __CIRC_BUF_H__

/*
 * ZG: 416 B
 * HG: 2496 B
 * GG: 52 KB
 *
*/

#ifdef EFM32HG110F64
#define	CIRC_BUF_SIZE		312

#elif EFM32GG230F512
#define	CIRC_BUF_SIZE		6656

#elif EFM32ZG110F32
#define	CIRC_BUF_SIZE		52
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
	float data;
};

struct circ_buf {
	struct point point[CIRC_BUF_SIZE];
	int head;
	int tail;
};

int push_point(struct circ_buf *cbuf, uint32_t ts, float dd)
{
	if (cbuf != NULL && CIRC_SPACE(cbuf->head, cbuf->tail, CIRC_BUF_SIZE) >= 1) {

		cbuf->point[cbuf->head].ts = ts;
		cbuf->point[cbuf->head].data = dd;

		cbuf->head += 1;

		cbuf->head %= CIRC_BUF_SIZE;

		return 0;
	} else {
		return 1;
	}
}

int pop_point(struct circ_buf *cbuf, struct point *odata)
{
	if (odata != NULL && cbuf != NULL &&
		CIRC_CNT(cbuf->head, cbuf->tail, CIRC_BUF_SIZE) > 0) {
	
		odata->ts = cbuf->point[cbuf->tail].ts;
		odata->data = cbuf->point[cbuf->tail].data;

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

		return 0;

	} else {
		return 1;
	}
}
#endif
