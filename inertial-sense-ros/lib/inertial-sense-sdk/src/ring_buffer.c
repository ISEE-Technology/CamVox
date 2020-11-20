/*
 *  ring_buffer.c
 *
 *  Created on: May 5, 2011
 *      Author: waltj
 */

#include <string.h>

#include "ring_buffer.h"

 //_____ D E F I N I T I O N S ______________________________________________

 //_____ G L O B A L S ______________________________________________________


 //_____ L O C A L   P R O T O T Y P E S ____________________________________

unsigned char* ringfindChar(unsigned char* bufPtr, unsigned char* endPtr, unsigned char character);
unsigned char* ringfindChar2(unsigned char* bufPtr, unsigned char* endPtr, unsigned char character1, unsigned char character2);


//_____ F U N C T I O N S __________________________________________________


/**
 * \brief Initialize ring buffer pointers
 */
void ringBufInit(ring_buf_t *rbuf, unsigned char* buf, int bufSize, int wordByteSize)
{
	rbuf->startPtr = buf;
	rbuf->endPtr = buf + bufSize;
	rbuf->rdPtr = buf;
	rbuf->wrPtr = buf;
	rbuf->bufSize = bufSize;
    rbuf->wordByteSize = wordByteSize;
}


/**
 * \brief This function returns the number of bytes currently in ring buffer.
 */
int ringBufUsed(const ring_buf_t *rbuf)
{
	int bytesUsed;

	bytesUsed = (int)(rbuf->wrPtr - rbuf->rdPtr);

	// Handle wrapping
	if (bytesUsed < 0)
		bytesUsed += rbuf->bufSize;

	return bytesUsed;
}


/**
 * \brief This function returns the number of bytes free in UART Rx buffer.  
 * Important: Buffer size is one less than init buffer size so pointers don't wrap.
 */
int ringBufFree(const ring_buf_t *rbuf)
{
    return (rbuf->bufSize - rbuf->wordByteSize - ringBufUsed(rbuf));
}


/**
 * \brief This function writes data to the ring buffer.
 *
 * \param rbuf  Ring buffer struct pointer.
 * \param buf   Buffer to copy data from.
 * \param len   Length of data to copy.
 *
 * \return 0 on success, 1 on overflow.
 */
int ringBufWrite(ring_buf_t *rb, unsigned char *buf, int numBytes)
{
	if (numBytes <= 0)
		return 0;

	int overflow = numBytes > ringBufFree(rb);

	// Copy data into buffer
	while (numBytes>0)
	{
		// Bytes before wrap is needed
		int bytesToEnd = (int)(rb->endPtr - rb->wrPtr);

		if (numBytes >= bytesToEnd)
		{   // Wrap needed

			// Copy data to end of buffer
			memcpy((void *)rb->wrPtr, (void *)buf, bytesToEnd);

			// Update pointer
			rb->wrPtr = rb->startPtr;

			numBytes -= bytesToEnd;
		}
		else
		{   // No wrap

			// Copy data
			memcpy((void *)rb->wrPtr, (void *)buf, numBytes);

			// Update pointer
			rb->wrPtr += numBytes;

			numBytes = 0;
		}
	}

	if (overflow)
	{	// Move read pointer for overflow
        rb->rdPtr = rb->wrPtr + rb->wordByteSize;

		if (rb->rdPtr >= rb->endPtr)
		{	// Handle wrap
			rb->rdPtr = rb->startPtr;
		}
	}

	return overflow;
}


/**
 * \brief This function reads data from the ring buffer.
 *
 * \param rbuf  Ring buffer struct pointer.
 * \param buf   Buffer to copy data to.
 * \param len   Length of data to attempt reading.
 *
 * \return Actual number of bytes read.
 */
int ringBufRead(ring_buf_t *rbuf, unsigned char *buf, int len)
{
	int bytesToRead, bytesRead1, bytesRead2;

	if ((bytesToRead = ringBufUsed(rbuf)) <= 0)
		return 0;   // Buffer empty

	if (len < bytesToRead)
		bytesToRead = len;

	// Will Data Read Need to Wrap?
	// If ( used part of Rx buffer wraps  &&  data to be read is more than to buffer's end )
	if (rbuf->rdPtr > rbuf->wrPtr && bytesToRead > rbuf->endPtr - rbuf->rdPtr)
	{   // Yes Wrapping
		// Bytes until end of buffer
		bytesRead1 = (int)(rbuf->endPtr - rbuf->rdPtr);
		bytesRead2 = (int)(bytesToRead - bytesRead1);

		// Copy Data into Rx buffer
		memcpy((void *)buf, (void *)rbuf->rdPtr, bytesRead1);						// Read to end
		memcpy((void *)&buf[bytesRead1], (void *)rbuf->startPtr, bytesRead2);	// Read starting at beginning

		// Update pointer
		rbuf->rdPtr = rbuf->startPtr + bytesRead2;
	}
	else
	{   // No Wrapping
		// Read data out of Rx buffer
		memcpy(buf, (void *)rbuf->rdPtr, bytesToRead);

		// Update pointer
		rbuf->rdPtr += bytesToRead;

		// Reset read pointer to buffer start (in case it reached end)
		if (rbuf->rdPtr >= rbuf->endPtr)
			rbuf->rdPtr = rbuf->startPtr;
	}

	return bytesToRead;
}


/**
 * \brief This function reads data from the ring buffer without removing
 *        any data.  Same as ringBufPop without moving pointers.
 *
 * \param rbuf  Ring buffer struct pointer.
 * \param buf   Buffer to copy data to.
 * \param len   Length of data to attempt reading.
 *
 * \return Actual number of bytes read.
 */
int ringBufPeek(const ring_buf_t *rbuf, unsigned char *buf, int len, int offset)
{
	int bytesToRead, bytesRead1, bytesRead2;
	int bytesUsed = ringBufUsed(rbuf);

	if ((bytesToRead = bytesUsed) <= 0)
		return 0;   // Buffer empty

	// Validate byte offset
	if (offset >= bytesUsed)
		return 0;

	if (len < bytesToRead)
		bytesToRead = len;

	unsigned char *rdPtr = rbuf->rdPtr + offset;
    // Handle read pointer wrapping
    if (rdPtr >= rbuf->endPtr) {
        rdPtr -= rbuf->bufSize;
    }

	// Will Data Read Need to Wrap?
	// If ( used part of Rx buffer wraps  &&  data to be read is more than to buffer's end )
	if (rdPtr > rbuf->wrPtr && bytesToRead > rbuf->endPtr - rdPtr)
	{   // Yes Wrapping
		// Bytes until end of buffer
		bytesRead1 = (int)(rbuf->endPtr - rdPtr);
		bytesRead2 = (int)(bytesToRead - bytesRead1);

		// Copy Data into Rx buffer
		memcpy((void *)buf, (void *)rdPtr, bytesRead1);							// Read to end
		memcpy((void *)&buf[bytesRead1], (void *)rbuf->startPtr, bytesRead2);	// Read starting at beginning
	}
	else
	{   // No Wrapping
		// Read data out of Rx buffer
		memcpy(buf, (void *)rdPtr, bytesToRead);
	}

	return bytesToRead;
}


/**
 * \brief This function returns a pointer to one past the first occurrence of the character in the string.
 *
 * \param bufPtr    Pointer to start of string.
 * \param endPtr    Pointer to one past end of string to be searched.
 * \param val       Value to be located.
 *
 * \return Pointer to one past the first occurrence of character in string.  NULL if not found.
 */
unsigned char* ringfindChar(unsigned char* bufPtr, unsigned char* endPtr, unsigned char character)
{
	// Search for character
	for (; bufPtr < endPtr; bufPtr++)
	{
		// Check for match
		if (*bufPtr == character)
			return bufPtr + 1;
	}

	return 0;
}


/**
 * \brief This function returns a pointer to one past the first occurrence of the character in the string.
 *
 * \param bufPtr    Pointer to start of string.
 * \param endPtr    Pointer to one past end of string to be searched.
 * \param val       Value to be located.
 *
 * \return Pointer to one past the first occurrence of character in string.  NULL if not found.
 */
unsigned char* ringfindChar2(unsigned char* bufPtr, unsigned char* endPtr, unsigned char character1, unsigned char character2)
{
	// Search for character
	for (; bufPtr < endPtr; bufPtr++)
	{
		// Check for match
		if (*bufPtr == character1 || *bufPtr == character2)
			return bufPtr + 1;
	}

	return 0;
}


/**
 * \brief This function returns everything up to and including the first occurrence of a character.
 *        If the character is not found, then nothing (zero) is returned.
 *
 * \param rbuf  Ring buffer struct pointer.
 * \param buf   Buffer to copy data to.
 * \param len   Length of data to attempt reading.
 * \param character Value to search for.
 *
 * \return Number of bytes read.
 */
int ringBufReadToChar(ring_buf_t *rbuf, unsigned char *buf, int len, unsigned char character)
{
	return ringBufReadToChar2(rbuf, buf, len, character, character);
}


/**
 * \brief This function returns everything up to and including the first occurrence of a character.
 *        If the character is not found, then nothing (zero) is returned.
 *
 * \param rbuf  Ring buffer struct pointer.
 * \param buf   Buffer to copy data to.
 * \param len   Length of data to attempt reading.
 * \param character1 Value to search for.
 * \param character2 Value to search for.
 *
 * \return Number of bytes read.
 */
int ringBufReadToChar2(ring_buf_t *rbuf, unsigned char *buf, int len, unsigned char character1, unsigned char character2)
{
	int bytesToRead, bytesRead1, bytesRead2;
	unsigned char* fndPtr;

	if ((bytesToRead = ringBufUsed(rbuf)) <= 0)
		return 0;   // Buffer empty

	if (len < bytesToRead)
		bytesToRead = len;

	// Will Data Search and Read Need to Wrap?
	// If ( used part of Rx buffer wraps  &&  data to be read is more than to buffer's end )
	if (rbuf->rdPtr > rbuf->wrPtr && bytesToRead > rbuf->endPtr - rbuf->rdPtr)
	{
		// Yes, Wrapping in Search

		// Search up to buffer end (before wrapping)
		if ((fndPtr = ringfindChar2(rbuf->rdPtr, rbuf->endPtr, character1, character2)) <= (unsigned char*)0)
		{
			// Not found - Search from start
			// Bytes until end of buffer
			bytesRead1 = (int)(rbuf->endPtr - rbuf->rdPtr);
			bytesRead2 = (int)(bytesToRead - bytesRead1);

			if ((fndPtr = ringfindChar2(rbuf->startPtr, rbuf->startPtr + bytesRead2, character1, character2)) <= (unsigned char*)0)
			{
				// Character NOT found
				return 0;
			}
			else
			{
				// Found character - Wrapping Needed
				bytesRead2 = (int)(fndPtr - rbuf->startPtr);

				// Copy Data into Rx buffer
				memcpy((void *)buf, (void *)rbuf->rdPtr, bytesRead1);                   // Read to end
				memcpy((void *)&buf[bytesRead1], (void *)rbuf->startPtr, bytesRead2);  // Read starting at beginning

				// Update pointer
				rbuf->rdPtr = rbuf->startPtr + bytesRead2;

				return bytesRead1 + bytesRead2;
			}
		}
	}
	else
	{   // No Wrapping in Search
		if ((fndPtr = ringfindChar2(rbuf->rdPtr, rbuf->rdPtr + bytesToRead, character1, character2)) <= (unsigned char*)0)
		{   // Character NOT found
			return 0;
		}
	}

	// Found character - No Wrapping
	bytesToRead = (int)(fndPtr - rbuf->rdPtr);

	// Read data out of Rx buffer
	memcpy(buf, rbuf->rdPtr, bytesToRead);

	// Update pointer
	rbuf->rdPtr += bytesToRead;

	// Reset read pointer to buffer start (in case it reached end)
	if (rbuf->rdPtr >= rbuf->endPtr)
		rbuf->rdPtr = rbuf->startPtr;

	return bytesToRead;
}


/**
 * \brief This function finds the index of the first matching string in the ring buffer.
 *  Returns -1 if not found.
 *
 * \param rbuf  Ring buffer struct pointer.
 * \param len   Length of data to attempt reading.
 * \param str	Character string to search for.
 *
 * \return Index of the first matching string.  -1 if not found.
 */
int ringBufFind(const ring_buf_t *rbuf, const unsigned char *str, int len)
{
	int i;
	int used = ringBufUsed(rbuf);
	// 	int bytesToEnd = rbuf->endPtr - rbuf->rdPtr;

	unsigned char *buf = (unsigned char*)(rbuf->rdPtr);

	for (i = 0; i < used; i++)
	{
		if (buf + len >= rbuf->endPtr)
		{
			// Handle wrapping
			int len1 = (int)(buf + len - rbuf->endPtr);
			int len2 = len - len1;

			if (!strncmp((const char*)buf, (const char*)str, len1) && !strncmp((const char*)buf, (const char*)rbuf->startPtr, len2))
				return i;
		}
		else
		{
			// Not Wrapping
			if (!strncmp((const char*)buf, (const char*)str, len))
				return i;
		}

		buf++;

		// Handling wrapping
		if (buf >= rbuf->endPtr)
			buf = rbuf->startPtr;
	}

	return -1;
}


/**
 * \brief This function removes data from the ring buffer.
 *
 * \param rbuf  Ring buffer struct pointer.
 * \param len   Number of bytes to remove.
 *
 * \return Actual number of bytes removed.
 */
int ringBufRemove(ring_buf_t *rbuf, int len)
{
	int bytesToRemove;

	if ((bytesToRemove = ringBufUsed(rbuf)) <= 0)
	{
		return 0;   // Buffer empty
	}

	if (len >= bytesToRemove)
	{   // Empty buffer
		rbuf->rdPtr = rbuf->wrPtr;

		// Return number bytes removed
		return bytesToRemove;
	}
	else
	{   // Leave some data in buffer
		rbuf->rdPtr += len;

		// Handle wrapping
		if (rbuf->rdPtr >= rbuf->endPtr)
			rbuf->rdPtr -= rbuf->bufSize;

		// Return number bytes removed
		return len;
	}
}


/**
 * \brief Clear the entire buffer. 
 * 
 * \return Number of bytes removed.
 */
int ringBufClear(ring_buf_t *rbuf)
{
	int used = ringBufUsed(rbuf);

	// Clear all
	rbuf->rdPtr = rbuf->wrPtr = rbuf->startPtr;

	return used;
}


/**
 * \brief This function returns 1 if the buffer is empty, 0 if not empty.
 */
int ringBufEmpty(const ring_buf_t *rbuf)
{
	return ringBufUsed(rbuf) == 0;
}
