/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _D_USARTDMA_H_
#define _D_USARTDMA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../ASF/sam/drivers/usart/usart.h"
#include "conf_d_usartDMA.h"

typedef struct
{
	uint32_t                timeMsLast;
	uint32_t                txByteCount;
	uint32_t                rxByteCount;
	uint32_t                txBytesPerS;
	uint32_t                rxBytesPerS;
	uint32_t                status;
} port_monitor_helper_t;

extern port_monitor_helper_t g_portMonitorHelper[MAX_NUMBER_SERIAL_PORTS];


//_____ P R O T O T Y P E S ________________________________________________

bool d_usartDMA_callback_cdc_enable(void);
void d_usartDMA_callback_cdc_disable(void);
void d_usartDMA_callback_cdc_tx_empty_notify(void);

/** 
 * \brief Write data on USART.  Returns number of bytes written.
 */
int serWrite( int serialNum, const unsigned char *buf, int size );

/**
 * \brief Read data on USART.  Returns number of bytes read.  With use of the PDCA ring buffer,
 *  buffer overrun may occur if bytes are received is greater than bytes read plus buffer size.
 */
// Buffer overrun can be detected when serRxUsed decreases outside of a serRead(), because
// the Rx PDCA (DMA) wrote past the read pointer.
int serRead( int serialNum, unsigned char *buf, int size );

/**
 * \brief Removes removes data from USART Rx buffer.  Returns number of bytes removed.  Length 
 *  of -1 removes all data from USART buffer
 */
int serRxClear( int serialNum, int len );

/** 
 * \brief Returns number of bytes used in Rx buffer.
 */
int serRxUsed( int serialNum );

/** 
 * \brief Clear the entire Tx buffer, returns how much was cleared
 */
int serTxClear( int serialNum );

/** 
 * \brief Returns number of bytes used in Tx buffer.
 */
int serTxUsed( int serialNum );

/** 
 * \brief Returns number of bytes available in Rx buffer.
 */
int serRxFree( int serialNum );

/** 
 * \brief Returns number of bytes available in Tx buffer.
 */
int serTxFree( int serialNum );

/**
 * \brief Returns number of characters in buffer before specified character.
 */
int serFindCharacter( int serialNum, uint8_t ch);

/**
 * \brief Returns -1 if the baudrate is not a standard baudrate.
 */
int validateBaudRate(unsigned int baudRate);

/**
 * \brief Change USART baudrate.  0 on success, -1 on failure.
 */
int serSetBaudRate( int serialNum, int baudrate );

/**
 * \brief Read USART baudrate.  Return value is the baudrate or -1 on failure.
 */
int serGetBaudRate( int serialNum );

/**
 * \brief Initialize serial port with specific USART/UART and DMA settings.  If not NULL, the overrun status will have bits HDW_STATUS_ERR_COM_TX_LIMITED and HDW_STATUS_ERR_COM_RX_OVERRUN set during buffer limitation.  
 */
int serInit( int serialNum, uint32_t baudRate, sam_usart_opt_t *options, uint32_t* overrunStatus );

#ifdef __cplusplus
}
#endif

#endif  // _D_USARTDMA_H_

