/*
 *  ring_buffer.h
 *
 *  Created on: May 5, 2011
 *      Author: waltj
 */

#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif


//_____ D E F I N I T I O N S ______________________________________________

typedef struct
{
	unsigned char *startPtr;        // Pointer to buffer start (lowest address)
	unsigned char *endPtr;          // Pointer to one past buffer end (buffer start + sizeof(buf)).
	unsigned char *rdPtr;           // Buffer read pointer
	unsigned char *wrPtr;           // Buffer write pointer
	int bufSize;                    // Byte size of buffer
    int wordByteSize;                   // Byte size of a single element in the buffer
} ring_buf_t;


//_____ P R O T O T Y P E S ________________________________________________

void ringBufInit(ring_buf_t *rbuf, unsigned char* buf, int bufSize, int wordSize);
int ringBufUsed(const ring_buf_t *rbuf);
int ringBufFree(const ring_buf_t *rbuf);
int ringBufWrite(ring_buf_t *rbuf, unsigned char *buf, int numBytes);
int ringBufRead(ring_buf_t *rbuf, unsigned char *buf, int len);
int ringBufPeek(const ring_buf_t *rbuf, unsigned char *buf, int len, int offset);
int ringBufReadToChar(ring_buf_t *rbuf, unsigned char *buf, int len, unsigned char character);
int ringBufReadToChar2(ring_buf_t *rbuf, unsigned char *buf, int len, unsigned char character1, unsigned char character2);
int ringBufFind(const ring_buf_t *rbuf, const unsigned char *str, int len);
int ringBufRemove(ring_buf_t *rbuf, int len);
int ringBufClear(ring_buf_t *rbuf);
int ringBufEmpty(const ring_buf_t *rbuf);



#ifdef __cplusplus
}
#endif

#endif  // _RING_BUFFER_H_
