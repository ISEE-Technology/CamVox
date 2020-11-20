/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <asf.h>
#include <string.h>
#include <stdio.h>

#include "d_usartDMA.h"

#ifdef CONF_BOARD_SPI_UINS
	#include "spiTouINS.h"
#endif

#include "d_dma.h"
#include "usart.h"
#include "../../src/ISComm.h"
#include "board_opt.h"
#include "globals.h"


//Structures for driver
#define DMA_LLD_COUNT	32	//Must be 2^x in size
#define DMA_LLD_MASK	(DMA_LLD_COUNT - 1)

typedef struct
{
	volatile XdmacChid		*dmaChId;					// Pointer to XDMA channel id
	volatile uint8_t		dmaChNumber;				// DMA channel number
	volatile uint32_t		size;						// DMA total buffer size

	// DMA buffer pointers:
	volatile uint8_t		*buf;						// start
	volatile uint8_t		*end;						// end
	volatile uint8_t		*ptr;						// current location

	//Pointers for the lld_view0 array
	volatile uint32_t lld_fptr;
	volatile uint32_t lld_bptr;

	// DMA linked list descriptor used for reload
	COMPILER_WORD_ALIGNED volatile lld_view0 lld[DMA_LLD_COUNT];

	// Items for SPI mode
	COMPILER_WORD_ALIGNED volatile lld_view0 lld_spi_noData[DMA_LLD_COUNT];

} dmaBuffer_tx_t;

typedef struct
{
	volatile XdmacChid		*dmaChId;					// Pointer to XDMA channel id
	volatile uint8_t		dmaChNumber;				// DMA channel number
	volatile uint32_t		size;						// DMA total buffer size
	volatile uint32_t		lastUsedRx;					// Track last used rx
	volatile uint32_t		timestampRx;				// Timestamp of last data read

	// DMA buffer pointers:
	volatile uint8_t		*buf;						// start
	volatile uint8_t		*end;						// end
	volatile uint8_t		*ptr;						// current location

	// DMA linked list descriptor used for reload
	COMPILER_WORD_ALIGNED volatile lld_view0 lld;
} dmaBuffer_rx_t;

typedef struct
{
	uint32_t ul_id;
	uint32_t usartTxTHR;
	uint32_t usartRxRHR;
	uint32_t xdmacUsartTxPerId; // XDMAC channel "HW Interface Number (XDMAC_CC.PERID)".  Refer to datasheet.
	uint32_t xdmacUsartRxPerId;
	uint8_t isUsartNotUart;
	uint8_t isSpiUsart;
} usart_info_t;

typedef struct
{
	volatile void *usart;
	sam_usart_opt_t usart_options;
	dmaBuffer_tx_t	dmaTx;
	dmaBuffer_rx_t	dmaRx;
	usart_info_t uinfo;
} usartDMA_t;

//Helper macros for port settings
#define CONCAT(A,B)         A ## B
#define ARGN(N, LIST)       CONCAT(ARG_, N) LIST
#define ARG_0(A0, ...)					A0
#define ARG_1(A0, A1, ...)				A1
#define ARG_2(A0, A1, A2, ...)			A2
#define ARG_3(A0, A1, A2, A3, ...)		A3
#define ARG_4(A0, A1, A2, A3, A4, ...)	A4

//Buffers for serial ports
#if MAX_NUMBER_SERIAL_PORTS >= 1
	COMPILER_ALIGNED(32) static uint8_t g_serTxDmaBuf_port0[ARGN(2, PORT0_CONFIG)];
	COMPILER_ALIGNED(32) static uint8_t g_serRxDmaBuf_port0[ARGN(2, PORT0_CONFIG)];
#endif

#if MAX_NUMBER_SERIAL_PORTS >= 2
	COMPILER_ALIGNED(32) static uint8_t g_serTxDmaBuf_port1[ARGN(2, PORT1_CONFIG)];
	COMPILER_ALIGNED(32) static uint8_t g_serRxDmaBuf_port1[ARGN(4, PORT1_CONFIG)];
#endif

#if MAX_NUMBER_SERIAL_PORTS >= 3
	COMPILER_ALIGNED(32) static uint8_t g_serTxDmaBuf_port2[ARGN(2, PORT2_CONFIG)];
	COMPILER_ALIGNED(32) static uint8_t g_serRxDmaBuf_port2[ARGN(4, PORT2_CONFIG)];
#endif

#if MAX_NUMBER_SERIAL_PORTS >= 4
	COMPILER_ALIGNED(32) static uint8_t g_serTxDmaBuf_port3[ARGN(2, PORT3_CONFIG)];
	COMPILER_ALIGNED(32) static uint8_t g_serRxDmaBuf_port3[ARGN(4, PORT3_CONFIG)];
#endif

#if MAX_NUMBER_SERIAL_PORTS >= 5
	COMPILER_ALIGNED(32) static uint8_t g_serTxDmaBuf_port4[ARGN(2, PORT4_CONFIG)];
	COMPILER_ALIGNED(32) static uint8_t g_serRxDmaBuf_port4[ARGN(4, PORT4_CONFIG)];
#endif

#if MAX_NUMBER_SERIAL_PORTS >= 6
	COMPILER_ALIGNED(32) static uint8_t g_serTxDmaBuf_port5[ARGN(2, PORT5_CONFIG)];
	COMPILER_ALIGNED(32) static uint8_t g_serRxDmaBuf_port5[ARGN(4, PORT5_CONFIG)];
#endif

#if MAX_NUMBER_SERIAL_PORTS >= 7
	COMPILER_ALIGNED(32) static uint8_t g_serTxDmaBuf_port6[ARGN(2, PORT6_CONFIG)];
	COMPILER_ALIGNED(32) static uint8_t g_serRxDmaBuf_port6[ARGN(4, PORT6_CONFIG)];
#endif

#if MAX_NUMBER_SERIAL_PORTS >= 8
	COMPILER_ALIGNED(32) static uint8_t g_serTxDmaBuf_port7[ARGN(2, PORT7_CONFIG)];
	COMPILER_ALIGNED(32) static uint8_t g_serRxDmaBuf_port7[ARGN(4, PORT7_CONFIG)];
#endif

#if MAX_NUMBER_SERIAL_PORTS >= 9
	#error "Need to add more support"
#endif

//Variables for general use
COMPILER_ALIGNED(32) static uint8_t noData_indicator[1] = {0};
static volatile usartDMA_t g_usartDMA[MAX_NUMBER_SERIAL_PORTS];
static uint32_t	*s_overrunStatus=NULLPTR;
port_monitor_helper_t g_portMonitorHelper[MAX_NUMBER_SERIAL_PORTS] = {0};



/*********************************       USB CDC       ********************************************************************************************************/
static bool usb_cdc_open = false;

bool d_usartDMA_callback_cdc_enable(void)
{
	usb_cdc_open = true;
	return true;
}

void d_usartDMA_callback_cdc_disable(void)
{
	usb_cdc_open = false;
}

void d_usartDMA_callback_cdc_tx_empty_notify(void)
{
#ifdef USB_PORT_NUM	
	dmaBuffer_tx_t *dma = (dmaBuffer_tx_t*)&g_usartDMA[USB_PORT_NUM].dmaTx;
	
	if(dma->ptr != dma->end)
	{
		//Get how much data is buffered
		int data_avail;
		if(dma->ptr >= dma->end)
			data_avail = dma->ptr - dma->end;
		else
			data_avail = dma->ptr - dma->end + dma->size;
		
		//Prevent race conditions with interrupt
		NVIC_DisableIRQ((IRQn_Type) ID_USBHS);
							
		//Get how much USB can take
		int usb_avail = udi_cdc_get_free_tx_buffer();
		if(usb_avail)
		{
			int bytesToEnd = dma->buf + dma->size - dma->end;
			int bytesToSend = Min(data_avail, usb_avail);
		
			if(bytesToSend < bytesToEnd)
			{
				udi_cdc_write_buf((void*)dma->end, bytesToSend);
				dma->end += bytesToSend;
			}
			else
			{
				udi_cdc_write_buf((void*)dma->end, bytesToEnd);
				dma->end = dma->buf;
			}
		}
		
		NVIC_EnableIRQ((IRQn_Type) ID_USBHS);
	}
#endif
}


/********************************* INTERNAL BUFFER FUCTIONS ***************************************************************************************************/
static inline const uint8_t* getCurrentTxDmaAddress( dmaBuffer_tx_t *dma )
{
	//Check to see if we are using the no data array
	if(dma->dmaChId->XDMAC_CSA >= (uint32_t)noData_indicator && dma->dmaChId->XDMAC_CSA <= ((uint32_t)noData_indicator+sizeof(noData_indicator)))
	{
		//We are not sending data. Send back pointer location.
		return (const uint8_t*)dma->ptr;
	}

	return (const uint8_t*)_CLAMP(dma->dmaChId->XDMAC_CSA, (uint32_t)dma->buf, (uint32_t)dma->end);
}

static inline const uint8_t* getCurrentRxDmaAddress( dmaBuffer_rx_t *dma )
{
	return (const uint8_t*)_CLAMP(dma->dmaChId->XDMAC_CDA, (uint32_t)dma->buf, (uint32_t)dma->end);
}

static inline dmaBuffer_tx_t * getTxDma( uint32_t serialNum )
{
	// Get the DMA buffer pointer
	if (serialNum >= MAX_NUMBER_SERIAL_PORTS || !g_usartDMA[serialNum].usart)
		return 0;
	
	return (dmaBuffer_tx_t*)&(g_usartDMA[serialNum].dmaTx);
}

static inline dmaBuffer_rx_t * getRxDma( uint32_t serialNum )
{
	// Get the DMA buffer pointer
	if (serialNum >= MAX_NUMBER_SERIAL_PORTS || !g_usartDMA[serialNum].usart)
		return 0;
	
	return (dmaBuffer_rx_t*)&(g_usartDMA[serialNum].dmaRx);
}

static inline int serTxUsedDma( dmaBuffer_tx_t *dma)
{
	const uint8_t* curDmaAddr = getCurrentTxDmaAddress(dma);

	// not using int with abs value because we want full 32 bit memory addresses
	if (dma->ptr < curDmaAddr)
	{
		// handle wrap
		return (int)(dma->size - (curDmaAddr - dma->ptr));
	}
	return (int)(dma->ptr - curDmaAddr);
}

static inline int serRxUsedDma( dmaBuffer_rx_t *dma )
{
	const uint8_t* curDmaAddr = getCurrentRxDmaAddress(dma);
	
	// not using int with abs value because we want full 32 bit memory addresses
	if (curDmaAddr < dma->ptr)
	{
		// handle wrap
		return (int)(dma->size - (dma->ptr - curDmaAddr));
	}
	return (int)(curDmaAddr - dma->ptr);
}

static inline int serTxFreeDma( dmaBuffer_tx_t *dma )
{
	return dma->size - serTxUsedDma(dma);
}

static inline int serRxFreeDma( dmaBuffer_rx_t *dma )
{
	return dma->size - serRxUsedDma(dma);
}

/********************************* EXTERNAL FUCTIONS ***************************************************************************************************/
/** 
 * \brief Returns number of bytes used in Tx buffer.
 */
int serTxUsed( int serialNum )
{
	// Get the DMA buffer pointer
	dmaBuffer_tx_t	*dma = getTxDma(serialNum);
	if( !dma ) return 0;
	
	return serTxUsedDma(dma);
}

/** 
 * \brief Returns number of bytes used in Rx buffer.
 */
int serRxUsed( int serialNum )
{
	// Get the DMA buffer pointer
	dmaBuffer_rx_t	*dma = getRxDma(serialNum);
	if( !dma ) return 0;
	
	return serRxUsedDma(dma);
}

/** 
 * \brief Returns number of bytes available in Tx buffer.
 */
int serTxFree( int serialNum )
{
#ifdef USB_PORT_NUM
	if(serialNum == USB_PORT_NUM)
	{
		return udi_cdc_get_free_tx_buffer();
	}
#endif
	
	// Get the DMA buffer pointer
	dmaBuffer_tx_t	*dma = getTxDma(serialNum);
	if( !dma ) return 0;

	return serTxFreeDma(dma);
}

/** 
 * \brief Returns number of bytes available in Rx buffer.
 */
int serRxFree( int serialNum )
{
	// Get the DMA buffer pointer
	dmaBuffer_rx_t	*dma = getRxDma(serialNum);
	if( !dma ) return 0;
	
	return serRxFreeDma(dma);
}

/** 
 * \brief Clear the entire Tx buffer
 */
int serTxClear( int serialNum )
{
	// Get the DMA buffer pointer
	dmaBuffer_tx_t *dma = getTxDma(serialNum);
	if( !dma ) return 0;

	// disable channel
	xdmac_disable_interrupt(XDMAC, dma->dmaChNumber);
	xdmac_channel_disable(XDMAC, dma->dmaChNumber);

	// save how much data we are discarding to return from function
	int used = serTxUsedDma(dma);

	// Clear all
	dma->ptr = dma->buf;
	dma->lld_fptr = 0;
	dma->lld_bptr = 0;

	//Check to see if we are in SPI mode
	usartDMA_t *ser = (usartDMA_t*)&g_usartDMA[serialNum];
	if (ser->uinfo.isSpiUsart)
	{
		xdmac_channel_config_t cfg = {0};
		cfg.mbr_sa = (uint32_t)noData_indicator;
		cfg.mbr_da = (uint32_t)ser->uinfo.usartTxTHR;
		cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM |
			XDMAC_CC_PERID(ser->uinfo.xdmacUsartTxPerId);
		cfg.mbr_ubc = 0;
		xdmac_configure_transfer(XDMAC, ser->dmaTx.dmaChNumber, &cfg);

		ser->dmaTx.lld_spi_noData[0].mbr_nda = (uint32_t)&(ser->dmaTx.lld_spi_noData[0]);

		xdmac_channel_set_descriptor_control(XDMAC, ser->dmaTx.dmaChNumber, XDMAC_CNDC_NDVIEW_NDV0 | XDMAC_CNDC_NDE_DSCR_FETCH_EN | XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED);
		xdmac_channel_set_descriptor_addr(XDMAC, ser->dmaTx.dmaChNumber, (uint32_t)&(ser->dmaTx.lld_spi_noData[0]), 0);
		dma->lld_fptr = 1;

		// Flush cached memory changes out to main memory
		SCB_CLEAN_DCACHE_BY_ADDR_32BYTE_ALIGNED(dma->lld_spi_noData, sizeof(dma->lld_spi_noData));

		// Re-enable DMA
		xdmac_channel_enable_no_cache(XDMAC, dma->dmaChNumber);
	}

	return used;
}

int serRxClear( int serialNum, int size)
{
	// Get the DMA buffer pointer
	dmaBuffer_rx_t	*dma = getRxDma(serialNum);
	if( !dma ) return 0;

	int used = serRxUsedDma(dma);

	// Flush FIFO, content of the FIFO is written to memory
	xdmac_channel_software_flush_request(XDMAC, dma->dmaChNumber);

	// Disable transfer
	xdmac_channel_disable(XDMAC, dma->dmaChNumber);

	// Reset
	dma->ptr = dma->buf;

	// Configure Rx linked list descriptor (only uses one)
	dma->lld.mbr_ta = (uint32_t)dma->buf;
	dma->lld.mbr_nda = (uint32_t)&(dma->lld); // point to self
	dma->lld.mbr_ubc = 
		XDMAC_UBC_NVIEW_NDV0 |
		XDMAC_UBC_NDE_FETCH_EN |
		XDMAC_UBC_NDEN_UPDATED |
		dma->size;

	xdmac_channel_set_descriptor_control(XDMAC, dma->dmaChNumber,
		XDMAC_CNDC_NDVIEW_NDV0 |
		XDMAC_CNDC_NDE_DSCR_FETCH_EN |
		XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED |
		XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED );

	xdmac_channel_set_descriptor_addr(XDMAC, dma->dmaChNumber, (uint32_t)&(dma->lld), 0);

// 	DBGPIO_START(DBG_TX_DCACHE_CLEAN_PIN);
	SCB_CLEAN_DCACHE_BY_ADDR_32BYTE_ALIGNED(dma, sizeof(dmaBuffer_rx_t));
// 	DBGPIO_END(DBG_TX_DCACHE_CLEAN_PIN);

	xdmac_channel_enable_no_cache(XDMAC, dma->dmaChNumber);

	return used;
}

// We assume worst case scenario, USART is writing data...
// Add new data to buffer that is not being written from.
int serWrite(int serialNum, const unsigned char *buf, int size)
{	
	if (size <= 0 || serialNum < 0 || serialNum >= MAX_NUMBER_SERIAL_PORTS)
	{
		return 0;
	}

#ifdef USB_PORT_NUM
	if(serialNum == USB_PORT_NUM)
	{
		if (usb_cdc_open)
		{
			dmaBuffer_tx_t *dma = (dmaBuffer_tx_t*)&g_usartDMA[USB_PORT_NUM].dmaTx;

			//NOTE: We are not preventing overflow at this time, it will just happen and data will be lost/corrupted
			//      (It shouldn't happen as USB moves data pretty fast.)

			//Prevent loading more data than buffer size
			if ((uint32_t)size > dma->size)
            { 
		        if (s_overrunStatus)
		        {	// Buffer overrun
					*s_overrunStatus |= HDW_STATUS_ERR_COM_TX_LIMITED;
		        }
                return 0;
            }                
			
			taskENTER_CRITICAL();

			//Check for data in buffer, if nothing buffered, USB is ready, and data fits, don't double buffer
			if(dma->ptr == dma->end && ((uint32_t)size <= udi_cdc_get_free_tx_buffer()))
			{
				udi_cdc_write_buf(buf, size);
			}
			else //We are sending more data than USB buffer will hold or there is already data in external buffer.
			{
				//Store data in the external buffer
				int bytesToEnd = dma->buf + dma->size - dma->ptr;
				if (size <= bytesToEnd)
				{
					// Don't need to wrap
					memcpy((void*)dma->ptr, buf, size);

					// Increment buffer pointer
					dma->ptr += size;
				}
				else	// Need to wrap
				{
					// Bytes to write at buffer start
					int bytesWrapped = size - bytesToEnd;
		
					//Write out data needed to fill the end of the buffer
					if (bytesToEnd)
					{
						// Not already at buffer end.  Fill buffer to end.
						memcpy((void*)dma->ptr, buf, bytesToEnd);
					}
		
					// Copy data into DMA buffer start & update pointer
					memcpy((void*)dma->buf, buf + bytesToEnd, bytesWrapped);

					// Increment buffer pointer
					dma->ptr = dma->buf + bytesWrapped;
				}

				//Send what fits to USB immediately (also makes sure external buffer didn't empty while we were in here)
				d_usartDMA_callback_cdc_tx_empty_notify();
			}
			
			taskEXIT_CRITICAL();
			
			g_portMonitorHelper[serialNum].txByteCount += size;
			return size;
		}
		return 0;
	}
#endif // #ifdef USB_PORT_NUM

	// Get the DMA buffer pointer
	dmaBuffer_tx_t	*dma = getTxDma(serialNum);
	if (!dma || (uint32_t)size > dma->size)	return 0;

	// Bytes free in DMA buffer
	int dmaFreeBytes = serTxFreeDma(dma);

	// Limit size
	if (size > dmaFreeBytes || ((dma->lld_fptr + 1) & DMA_LLD_MASK) == dma->lld_bptr )
	{
		// tx overrun
		serTxClear(serialNum);
		if (s_overrunStatus)
		{
			*s_overrunStatus |= HDW_STATUS_ERR_COM_TX_LIMITED;
		}
#ifndef __INERTIAL_SENSE_EVB_2__
		g_internal_diagnostic.txOverflowCount[serialNum]++;
#endif
	}

	//////////////////////////////////////////////////////////////////////////
	// Copy into DMA buffer

	// Get ser struct to see if we are in SPI mode
	usartDMA_t *ser = (usartDMA_t*)&g_usartDMA[serialNum];

	taskENTER_CRITICAL();

	// Bytes before end of DMA buffer
	int bytesToEnd = dma->end - dma->ptr;

	// Add data to ring buffer
	if (size <= bytesToEnd)
	{	
		// Don't need to wrap
		// Copy data into DMA buffer
		MEMCPY_DCACHE_CLEAN(dma->ptr, buf, size);

		// Setup DMA
		dma->lld[dma->lld_fptr].mbr_ta = (uint32_t)dma->ptr;
		if (ser->uinfo.isSpiUsart)
		{
			dma->lld[dma->lld_fptr].mbr_ubc = XDMAC_UBC_NVIEW_NDV0 | XDMAC_UBC_NDE_FETCH_EN | XDMAC_UBC_NSEN_UPDATED | XDMAC_UBC_UBLEN(size);
			dma->lld[dma->lld_fptr].mbr_nda = (uint32_t)&(ser->dmaTx.lld_spi_noData[dma->lld_fptr]);	//Data lld points to noData lld
			dma->lld_spi_noData[dma->lld_fptr].mbr_nda = (uint32_t)&(ser->dmaTx.lld_spi_noData[dma->lld_fptr]);	//noData lld wraps around on itself
		}
		else
		{
			dma->lld[dma->lld_fptr].mbr_ubc = XDMAC_UBC_UBLEN(size);
		}

		// Increment buffer pointer
		dma->ptr = dma->ptr + size;

		// Move to next lld
		dma->lld_fptr = (dma->lld_fptr + 1) & DMA_LLD_MASK;
	}
	else	// Need to wrap
	{	
		// Bytes to write at buffer start
		int bytesWrapped = size - bytesToEnd;
		uint32_t bufptr = dma->lld_fptr;
		int count = 1;
		
		//Write out data needed to fill the end of the buffer
		if (bytesToEnd)
		{	
			// Not already at buffer end.  Fill buffer to end.
			// Copy data into DMA buffer & update transfer size
			MEMCPY_DCACHE_CLEAN(dma->ptr, buf, bytesToEnd);

			// Setup DMA for end
			dma->lld[bufptr].mbr_ta = (uint32_t)dma->ptr;
			if (ser->uinfo.isSpiUsart)
			{
				dma->lld[bufptr].mbr_ubc = XDMAC_UBC_NVIEW_NDV0 | XDMAC_UBC_NDE_FETCH_EN | XDMAC_UBC_NSEN_UPDATED | XDMAC_UBC_UBLEN(bytesToEnd);
				dma->lld[bufptr].mbr_nda = (uint32_t)&(ser->dmaTx.lld[(bufptr + 1) & DMA_LLD_MASK]);	//Data lld points to next data lld
			}
			else
			{
				dma->lld[bufptr].mbr_ubc = XDMAC_UBC_UBLEN(bytesToEnd);
			}
			
			// Move to next lld
			bufptr = (bufptr + 1) & DMA_LLD_MASK;
			count++;
		}
			
		// Copy data into DMA buffer start & update pointer
		MEMCPY_DCACHE_CLEAN(dma->buf, buf + bytesToEnd, bytesWrapped);

		// Setup DMA for beginning
		dma->lld[bufptr].mbr_ta = (uint32_t)dma->buf;
		if (ser->uinfo.isSpiUsart)
		{
			dma->lld[bufptr].mbr_ubc = XDMAC_UBC_NVIEW_NDV0 | XDMAC_UBC_NDE_FETCH_EN | XDMAC_UBC_NSEN_UPDATED | XDMAC_UBC_UBLEN(bytesWrapped);
			dma->lld[bufptr].mbr_nda = (uint32_t)&(ser->dmaTx.lld_spi_noData[bufptr]);	//Data lld points to noData lld
			dma->lld_spi_noData[bufptr].mbr_nda = (uint32_t)&(ser->dmaTx.lld_spi_noData[bufptr]);	//noData lld wraps around on itself
		}
		else
		{
			dma->lld[bufptr].mbr_ubc = XDMAC_UBC_UBLEN(bytesWrapped);
		}

		// Increment buffer pointer
		dma->ptr = dma->buf + bytesWrapped;
		
		// Move to next lld
		dma->lld_fptr = (dma->lld_fptr + count) & DMA_LLD_MASK;
	}

	//////////////////////////////////////////////////////////////////////////
	// Transfer out of DMA buffer

	// Indicate Transmit
	SER_INDICATE_TX();

	if (ser->uinfo.isSpiUsart)
	{
#if CONF_BOARD_USART_SPI_DATAREADY_ENABLE == 1
		//Block interrupt handler from running while we add data so there isn't a race condition on the data ready pin
		NVIC_DisableIRQ(XDMAC_IRQn);
#endif

		//Connect up the new lld nodes
		uint32_t newLld = (dma->lld_bptr + 1) & DMA_LLD_MASK;
		ser->dmaTx.lld_spi_noData[dma->lld_bptr].mbr_nda = (uint32_t)&(ser->dmaTx.lld[newLld]);

		//Move back pointer manually in SPI mode
		dma->lld_bptr = (dma->lld_fptr - 1) & DMA_LLD_MASK;

		//Write data out to main memory
		SCB_CLEAN_DCACHE_BY_ADDR_32BYTE_ALIGNED(dma->lld, sizeof(dma->lld));
		SCB_CLEAN_DCACHE_BY_ADDR_32BYTE_ALIGNED(dma->lld_spi_noData, sizeof(dma->lld_spi_noData));

#if CONF_BOARD_USART_SPI_DATAREADY_ENABLE == 1
		// Enable DMA interrupt
		xdmac_enable_interrupt(XDMAC, dma->dmaChNumber);
		xdmac_channel_enable_interrupt(XDMAC, dma->dmaChNumber, XDMAC_CIE_BIE);

		//Set data ready pin for SPI mode
		ioport_set_pin_level(SPI_DATAREADY_GPIO, IOPORT_PIN_LEVEL_HIGH);

		//Enable interrupt on completion of linked list node so we can watch for the end of the data
		NVIC_EnableIRQ(XDMAC_IRQn);
#endif
	}
	else
	{
		//Block interrupt handler from running and causing a double configure
		NVIC_DisableIRQ(XDMAC_IRQn);

		//Enable DMA if it isn't running
		if((XDMAC->XDMAC_GS & (XDMAC_GS_ST0 << (dma->dmaChNumber))) == 0)
		{
			if(dma->lld_bptr != dma->lld_fptr)
			{
				//Setup TX DMA
				xdmac_channel_set_source_addr(XDMAC, dma->dmaChNumber, dma->lld[dma->lld_bptr].mbr_ta);
				xdmac_channel_set_microblock_control(XDMAC, dma->dmaChNumber, dma->lld[dma->lld_bptr].mbr_ubc);

				//Move pointer
				dma->lld_bptr = (dma->lld_bptr + 1) & DMA_LLD_MASK;

				// Enable DMA interrupt
				xdmac_enable_interrupt(XDMAC, dma->dmaChNumber);
				xdmac_channel_enable_interrupt(XDMAC, dma->dmaChNumber, XDMAC_CIE_BIE);

				//Enable - cache should have already been cleaned
				XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << (dma->dmaChNumber));
			}
		}

		NVIC_EnableIRQ(XDMAC_IRQn);
	}

	taskEXIT_CRITICAL();

	g_portMonitorHelper[serialNum].txByteCount += size;
	return size;
}

void XDMAC_Handler(void)
{
#ifdef CONF_BOARD_SPI_UINS	
	//Forward for spiTouINS
	XDMAC_spiTouINS_Handler();
#endif	

	static volatile int xdmacSPICount = 0;
	
	for(int i=0;i<MAX_NUMBER_SERIAL_PORTS;i++)
	{
		//Make sure port is valid
		if(g_usartDMA[i].usart == 0)
			continue;

		volatile dmaBuffer_tx_t	*dma = &(g_usartDMA[i].dmaTx);

		//See if interrupt is active on this channel
		if ((dma->dmaChId->XDMAC_CIS & XDMAC_CIS_BIS) && (dma->dmaChId->XDMAC_CIM & XDMAC_CIM_BIM))
		{
#if CONF_BOARD_USART_SPI_DATAREADY_ENABLE == 1
			//See if we are in SPI mode on this USART
			if (g_usartDMA[i].uinfo.isSpiUsart)
			{
				//See if we are at the end of the list
				if((uint32_t)(dma->lld_spi_noData[dma->lld_bptr].mbr_nda) == dma->dmaChId->XDMAC_CNDA)
				{
					//See if we are transmitting the no data indicator
					if(dma->dmaChId->XDMAC_CSA >= (uint32_t)noData_indicator && dma->dmaChId->XDMAC_CSA <= (uint32_t)(noData_indicator+sizeof(noData_indicator)))
					{
						//Delay number of "no data" transmissions (characters) to get the last bytes out of the port before lowering the data ready indicator.
						if(2 <= xdmacSPICount)
						{
							ioport_set_pin_level(SPI_DATAREADY_GPIO, IOPORT_PIN_LEVEL_LOW);
							xdmac_channel_disable_interrupt(XDMAC, dma->dmaChNumber, XDMAC_CIE_BIE);
						}
						else
						{
							xdmacSPICount++;
						}
					}
					else
					{
						xdmacSPICount = 0;
					}
				}
				else
				{
					xdmacSPICount = 0;
				}
			}
			else  //This "else" makes the following "if" an "else if"
#endif
			//Check to see if DMA is finished (as indicated by channel becoming disabled
			if ((XDMAC->XDMAC_GS & (XDMAC_GS_ST0 << (dma->dmaChNumber))) == 0)
			{
				if(dma->lld_bptr != dma->lld_fptr)
				{
					//Setup TX DMA
					xdmac_channel_set_source_addr(XDMAC, dma->dmaChNumber, dma->lld[dma->lld_bptr].mbr_ta);
					xdmac_channel_set_microblock_control(XDMAC, dma->dmaChNumber, dma->lld[dma->lld_bptr].mbr_ubc);

					//Move pointer
					dma->lld_bptr = (dma->lld_bptr + 1) & DMA_LLD_MASK;

					//Enable - We don't need to clean cache as DMA data is already flushed to main memory
					XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << (dma->dmaChNumber));
				}
				else
				{
					//Turn off interrupt when done
					xdmac_channel_disable_interrupt(XDMAC, dma->dmaChNumber, XDMAC_CIE_BIE);
				}
			}
		}
	}
}

int serRead(int serialNum, unsigned char *buf, int size)
{
	if (size <= 0 || serialNum < 0 || serialNum >= MAX_NUMBER_SERIAL_PORTS)
	{
		return 0;
	}

#ifdef USB_PORT_NUM
	if(serialNum == USB_PORT_NUM)
	{
		if(usb_cdc_open)
		{
			return udi_cdc_read_no_polling(buf, size);
		}
		else
		{
			return 0;
		}
	}
#endif

	// Get the DMA buffer pointer
	dmaBuffer_rx_t	*dma = getRxDma(serialNum);
	if (!dma) return 0;
		
	// Flush FIFO, content of the FIFO is written to memory
	xdmac_channel_software_flush_request(XDMAC, dma->dmaChNumber);

	// Bytes in DMA buffer
	uint32_t dmaUsed = serRxUsedDma(dma);
	
	if (dmaUsed < dma->lastUsedRx)
	{
		// rx overrun
		serRxClear(serialNum, -1);
		dma->lastUsedRx = 0;
		if (s_overrunStatus)
		{
			*s_overrunStatus |= HDW_STATUS_ERR_COM_RX_OVERRUN;
		}
#ifndef __INERTIAL_SENSE_EVB_2__
		g_internal_diagnostic.rxOverflowCount[serialNum]++;
#endif
		size = 0;
	}
	else
	{
		// Limit to bytes available
		size = Min((uint32_t)size, dmaUsed);

		if (size > 0)
		{
			// Bytes before end of DMA buffer
			int bytesToEnd = dma->end - dma->ptr;
		
			if (size >= bytesToEnd)
			{	
				// Handle wrapping
				// Copy data into DMA buffer & update transfer size
				DCACHE_CLEAN_INVALIDATE_MEMCPY(buf, dma->ptr, bytesToEnd);
		
				int bytesWrapped = size - bytesToEnd;
				if (bytesWrapped)
				{
					// Copy data into DMA buffer
					DCACHE_CLEAN_INVALIDATE_MEMCPY(&buf[bytesToEnd], dma->buf, bytesWrapped);
				}
				dma->ptr = dma->buf + bytesWrapped;
			}
			else
			{
				// Not wrapping
				// Copy data into DMA buffer
				DCACHE_CLEAN_INVALIDATE_MEMCPY(buf, dma->ptr, size);
				dma->ptr += size;
			}		

			// Indicate Read
			SER_INDICATE_RX();
		}
	
		dma->lastUsedRx = dmaUsed - size;
	}
	
#ifndef __INERTIAL_SENSE_EVB_2__
	if (size > 0)
	{
		uint32_t currentTime = time_msec();
		uint32_t gap = UINT32_TIME_DIFF(currentTime, dma->timestampRx);
		if (gap > 500)
		{
			g_internal_diagnostic.gapCountSerialDriver[serialNum]++;
		}
		dma->timestampRx = currentTime;
	}
#endif	

	// Increment port monitor count
	g_portMonitorHelper[serialNum].rxByteCount += size;
    return size;
}

int serFindCharacter( int serialNum, uint8_t ch)
{
#ifdef USB_PORT_NUM

	//No support for USB port at this time
	if(serialNum == USB_PORT_NUM)
		return 0;

#endif

	// Get the DMA buffer pointer
	dmaBuffer_rx_t	*dma = getRxDma(serialNum);
	if (!dma) return 0;

	// Bytes in DMA buffer
	uint32_t dmaUsed = serRxUsedDma(dma);

	// Return if no data is available
	if(dmaUsed == 0) return 0;
		
	// Flush FIFO, content of the FIFO is written to memory
	xdmac_channel_software_flush_request(XDMAC, dma->dmaChNumber);
	
	// Clean & invalidate memory so we can see what is there
	SCB_CLEANINVALIDATE_DCACHE_BY_ADDR_32BYTE_ALIGNED(dma->buf, dma->size);

	//Look for character
	uint32_t chCount = 0;
	volatile uint8_t *ptr = dma->ptr;
	while(++chCount <= dmaUsed)
	{
		if(*ptr == ch)
			return (int)chCount;		
			
		//move pointer
		ptr++;
		if(ptr >= dma->end)	//Roll over to beginning if we reach the end
			ptr = dma->buf;
	}
	
	//Not found, return zero
	return 0;
}

static int serEnable(int serialNum)
{
	usartDMA_t *ser = (usartDMA_t*)&g_usartDMA[serialNum];

	// Re-init UART
	if (ser->uinfo.isUsartNotUart)
	{	// Initialize USART
		if (ser->uinfo.isSpiUsart)
		{
			// Initialize the USART in SPI slave mode.
			usart_spi_opt_t opt = {
				.baudrate     = 3000000,  // ignored when configuring for slave mode
				.char_length  = US_MR_CHRL_8_BIT,
				.spi_mode     = SPI_MODE_3,
				.channel_mode = US_MR_CHMODE_NORMAL
			};
			usart_init_spi_slave((Usart*)ser->usart, &opt);
		}
		else
		{
			// Initialize the USART in RS232 mode.
			usart_init_rs232((Usart*)ser->usart, &(ser->usart_options), sysclk_get_peripheral_hz());
		}
		
		// Enable the receiver and transmitter.
		usart_enable_tx((Usart*)ser->usart);
		usart_enable_rx((Usart*)ser->usart);
	}
	else
	{	// Initialize UART
		sam_uart_opt_t p_uart_opt =
		{
			.ul_mck = sysclk_get_peripheral_hz(),
			.ul_baudrate = ser->usart_options.baudrate,			// baudrate = sysclk / (16 * UART_BRGR).   Example: 150000000 Hz / (16 * 10) = 937500 bps   
			.ul_mode =				             // Mode register:
				UART_MR_PAR_NO |				     // No parity.
				UART_MR_FILTER_DISABLED |	   // Don't filter Rx line.
				UART_MR_BRSRCCK_PERIPH_CLK | // Use peripheral clock.  Not PMC clock.
				UART_MR_CHMODE_NORMAL		     // Normal channel mode.  No loopback.
		};
		
		// Initialize the UART in normal mode.
		uart_init((Uart*)ser->usart, &p_uart_opt);
		
		// Enable the receiver and transmitter.
		uart_enable((Uart*)ser->usart);
	}

	return 0;
}

#ifndef __INERTIAL_SENSE_EVB_2__
int validateBaudRate(unsigned int baudRate)
{
	// Valid baudrates for InertialSense hardware
	for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(g_validBaudRates); i++)
	{
		if (g_validBaudRates[i] == baudRate)
		{
			return 0;
		}
	}
	return -1;
}
#endif

// 0 on success, -1 on failure
int serSetBaudRate( int serialNum, int baudrate )
{
	usartDMA_t *ser = (usartDMA_t*)&g_usartDMA[serialNum];
	if( !ser->usart ) return -1;

	// If we are not SPI, update baudrate
	if (ser->uinfo.isSpiUsart == 0)
	{
		// Update baudrate setting
#ifndef __INERTIAL_SENSE_EVB_2__
		if (0 == validateBaudRate(baudrate))
#endif
			ser->usart_options.baudrate = baudrate;
	}

	return serEnable(serialNum);
}

/**
 * \brief Read USART baudrate.  Return value is the baudrate or -1 on failure.
 */
int serGetBaudRate( int serialNum )
{
	usartDMA_t *ser = (usartDMA_t*)&g_usartDMA[serialNum];
	if( !ser ) return -1;
	
	// Baudrate setting
	return ser->usart_options.baudrate;
}

/********************************* INITIALIZATION ROUTINES ***************************************************************************************************/
//The optimizer seems to allow the use of the lld array before it is completely configured. No optimization prevents that from happening.
__attribute__((optimize("O0")))
static void serDmaRxInit(usartDMA_t *ser)
{
	xdmac_channel_config_t cfg = {0};
	cfg.mbr_sa = (uint32_t)ser->uinfo.usartRxRHR;
	cfg.mbr_da = (uint32_t)ser->dmaRx.buf;
	cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MEMSET_NORMAL_MODE |
		XDMAC_CC_DSYNC_PER2MEM |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DWIDTH_BYTE |
		XDMAC_CC_SIF_AHB_IF1 |
		XDMAC_CC_DIF_AHB_IF1 |
		XDMAC_CC_SAM_FIXED_AM |
		XDMAC_CC_DAM_INCREMENTED_AM |
		XDMAC_CC_PERID(ser->uinfo.xdmacUsartRxPerId);
	cfg.mbr_ubc = ser->dmaRx.size;
	xdmac_configure_transfer(XDMAC, ser->dmaRx.dmaChNumber, &cfg);

	// Configure linked list descriptor (only uses one)
	ser->dmaRx.lld.mbr_ta = (uint32_t)ser->dmaRx.buf;
	ser->dmaRx.lld.mbr_nda = (uint32_t)&(ser->dmaRx.lld); // point to self
	ser->dmaRx.lld.mbr_ubc = 
		XDMAC_UBC_NVIEW_NDV0 |
		XDMAC_UBC_NDE_FETCH_EN |
		XDMAC_UBC_NDEN_UPDATED |
		ser->dmaRx.size;

	// Set initial descriptor control
	xdmac_channel_set_descriptor_control(XDMAC, ser->dmaRx.dmaChNumber,	//Updates CNDC register
		XDMAC_CNDC_NDVIEW_NDV0 |
		XDMAC_CNDC_NDE_DSCR_FETCH_EN |
		XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED);
	xdmac_channel_set_descriptor_addr(XDMAC, ser->dmaRx.dmaChNumber, (uint32_t)&(ser->dmaRx.lld), 0);	//Updates CNDA register
}

//The optimizer seems to allow the use of the lld array before it is completely configured. No optimization prevents that from happening.
__attribute__((optimize("O0")))
static void serDmaTxInit(usartDMA_t *ser)
{
	xdmac_channel_config_t cfg = {0};
	cfg.mbr_sa = (uint32_t)noData_indicator;
	cfg.mbr_da = (uint32_t)ser->uinfo.usartTxTHR;
	cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MEMSET_NORMAL_MODE |
		XDMAC_CC_DSYNC_MEM2PER |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DWIDTH_BYTE |
		XDMAC_CC_SIF_AHB_IF1 |
		XDMAC_CC_DIF_AHB_IF1 |
		XDMAC_CC_SAM_INCREMENTED_AM |
		XDMAC_CC_DAM_FIXED_AM |
		XDMAC_CC_PERID(ser->uinfo.xdmacUsartTxPerId);
	cfg.mbr_ubc = 0;
	xdmac_configure_transfer(XDMAC, ser->dmaTx.dmaChNumber, &cfg);
	
	//Setup pointers to lld array
	ser->dmaTx.lld_fptr = 0;
	ser->dmaTx.lld_bptr = 0;

	if (ser->uinfo.isSpiUsart)
	{
		// The datasheet says that when a transfer happens and there is no data in the TX buffer, it will send a 0xFF. Its true right after reset, it will send 0xFF.
		// But that appears to be not true after data has been sent. If the last bit sent was high, the line stays high and will send a 0xFF, if the last bit was low, it will stay low and send 0x00.
		// But... can we guarantee that will hold true if they do a chip rev? Maybe not, so we will just guarantee it by keeping the TX DMA running
		// This is done by having the lld loop back on itself with 0x00 data.

		for(int i=0; i<DMA_LLD_COUNT; i++)
		{
			// Data nodes point to empty indicator so when they finish sending the empty indicator gets sent
			ser->dmaTx.lld[i].mbr_nda = (uint32_t)&(ser->dmaTx.lld_spi_noData[i]);

			// Configure the DMA descriptors that handle transmission when there is no data to be sent. These will be configured to loop back on themselves.
			ser->dmaTx.lld_spi_noData[i].mbr_nda = (uint32_t)&(ser->dmaTx.lld_spi_noData[i]);

			// The data to send comes from the noData_indicator array
			ser->dmaTx.lld_spi_noData[i].mbr_ta = (uint32_t)noData_indicator;

			//Setup microblock control member - this will never change for the no data indicators.
			ser->dmaTx.lld_spi_noData[i].mbr_ubc = XDMAC_UBC_NVIEW_NDV0 | XDMAC_UBC_NDE_FETCH_EN | XDMAC_UBC_NSEN_UPDATED | sizeof(noData_indicator);
		}

		// Set initial descriptor control
		xdmac_channel_set_descriptor_control(XDMAC, ser->dmaTx.dmaChNumber, XDMAC_CNDC_NDVIEW_NDV0 | XDMAC_CNDC_NDE_DSCR_FETCH_EN | XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED);
		xdmac_channel_set_descriptor_addr(XDMAC, ser->dmaTx.dmaChNumber, (uint32_t)&(ser->dmaTx.lld_spi_noData[0]), 0);
		ser->dmaTx.lld_fptr = 1;

#if CONF_BOARD_USART_SPI_DATAREADY_ENABLE == 1
		//Setup interrupts for data ready pin for SPI mode
		NVIC_ClearPendingIRQ(XDMAC_IRQn);
		NVIC_SetPriority(XDMAC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);	//Must not be higher than RTOS allows
		NVIC_EnableIRQ(XDMAC_IRQn);
#endif
	}
	else
	{
		//Setup interrupts for DMA for UART mode (gets multiple setups as this routine is called for multiple USARTs)
		NVIC_ClearPendingIRQ(XDMAC_IRQn);
		NVIC_SetPriority(XDMAC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);	//Must not be higher than RTOS allows
		NVIC_EnableIRQ(XDMAC_IRQn);
	}
}


#if (ENABLE_COMM_LOOPBACK_DRIVER > 0)
static void serLoopback(int portNum)
{
	if (CONF_LOOPBACK_PORT != portNum)
		return;

#define RX_BUF_SIZE    256
	static unsigned char rxBuf[RX_BUF_SIZE] = {0};
	int bytesRead = 0;
	int timeout = 0;

#if ENABLE_COMM_LOOPBACK_DRIVER==1 // Loop back test - simple
	while (1)
	{
		// Read data
//		bytesRead = serRead(portNum, rxBuf, 1, 0);
		bytesRead = serRead(portNum, rxBuf, RX_BUF_SIZE, 0);
			
		// Write data
		if (bytesRead)
		{
			if (g_usartDMA[portNum].uinfo.isSpiUsart)
			{
				//we need to omit zero data for the test
				int i, cnt = 0;
				for(i=0;i<bytesRead;i++)
					if(rxBuf[i] != 0)
						rxBuf[cnt++] = rxBuf[i];

				serWrite(portNum, rxBuf, cnt, 0);
			}
			else
			{
				serWrite(portNum, rxBuf, bytesRead, 0);
			}
            LEDS_ALL_TOGGLE();
		}

#ifndef __INERTIAL_SENSE_EVB_2__
		watchdog_maintenance_force(); // ensure watch dog does not kill us
#endif
	}
#endif // ENABLE_COMM_LOOPBACK_DRIVER==1

#if ENABLE_COMM_LOOPBACK_DRIVER==2 // Loop back test - multiple reads before multiple writes
    LEDS_ALL_OFF();

	while (1)
	{
		// Aggregate data
		if (serRead(portNum, &(rxBuf[bytesRead]), 1, 0))
		{	
			if (g_usartDMA[portNum].uinfo.isSpiUsart)
			{
				if(rxBuf[bytesRead] != 0)
					++bytesRead;
			}
			else
			{
				++bytesRead;
			}

            // Toggle LED for Rx
            LED_TOGGLE(LED_GRN);

			timeout = 0;
		}

		// Wait until we have number of bytes to read or we timeout
		if (bytesRead>=10 || ++timeout >= 100000)
		{
            if(bytesRead)
            {
                int bytesSend1 = bytesRead/2;               // split transmission into two serWrite calls
                int bytesSend2 = bytesRead - bytesSend1;
			    serWrite(portNum, rxBuf, bytesSend1, 0);
			    serWrite(portNum, &rxBuf[bytesSend1], bytesSend2, 0);
    			bytesRead = 0;            

                // Toggle LED for Tx
                LED_TOGGLE(LED_RED);
            }

			timeout = 0;
		}
		
		watchdog_maintenance_force(); // ensure watch dog does not kill us
	}
#endif // ENABLE_COMM_LOOPBACK_DRIVER==2

#if ENABLE_COMM_LOOPBACK_DRIVER == 3	// Raw read/write test
	//Turn off DMA
	usartDMA_t *ser = (usartDMA_t*)&g_usartDMA[portNum];
	xdmac_channel_disable(XDMAC, ser->dmaTx.dmaChNumber);
	xdmac_channel_disable(XDMAC, ser->dmaRx.dmaChNumber);

	// For SPI mode, preload data for transmit
	if (ser->uinfo.isSpiUsart)
	{
		((Usart*)ser->usart)->US_THR = 0;
	}

	while(1)
	{
		uint32_t status = usart_get_status((Usart*)ser->usart);

		if(status & US_CSR_RXRDY && status & US_CSR_TXRDY)
		{
			((Usart*)ser->usart)->US_THR = ((Usart*)ser->usart)->US_RHR;
		}

		watchdog_maintenance_force(); // ensure watch dog does not kill us
	}
#endif // ENABLE_COMM_LOOPBACK_DRIVER==3

#if ENABLE_COMM_LOOPBACK_DRIVER==4	//Assembles a packet before sending back
    LEDS_ALL_OFF();
	((Usart*)g_usartDMA[portNum].usart)->US_THR = 0;
	
	while (1)
	{
		bool inpacket = false;
		bytesRead = 0;
		
		while(1)
		{
			if (serRead(portNum, &(rxBuf[bytesRead]), 1, 0))
			{	
				if(inpacket)
				{
					++bytesRead;
					
					if(0xFE == rxBuf[bytesRead - 1])
						break;
				}
				else
				{
					if( 0xFF == rxBuf[bytesRead])
					{
						++bytesRead;
						inpacket = true;	
					}
				}

				// Toggle LED for Rx
				LED_TOGGLE(LED_GRN);
			}

			watchdog_maintenance_force(); // ensure watch dog does not kill us
		}

        if(bytesRead)
        {
			for(int j=2;j<bytesRead-1;j++)
			{
				if(rxBuf[j-1] + 1 != rxBuf[j])
					rxBuf[j] = 200;
			}
			serWrite(portNum, rxBuf, bytesRead, 0);
			bytesRead = 0;

            // Toggle LED for Tx
            LED_TOGGLE(LED_RED);
        }
	}
#endif // ENABLE_COMM_LOOPBACK_DRIVER==4

} // serLoopback
#endif // ENABLE_COMM_LOOPBACK_DRIVER > 0

static int setup_usart_info(usartDMA_t *ser, int serialNumber, uint32_t baudRate, sam_usart_opt_t *options)
{
	// Setup Baud Rate
	if(baudRate == 0)
	{
		ser->usart_options.baudrate = DEFAULT_BAUDRATE;
	}
	else if(options != NULL)
	{
		if(options->baudrate == 0)
			ser->usart_options.baudrate = DEFAULT_BAUDRATE;
		else
			ser->usart_options.baudrate = options->baudrate;
	}
	else
	{
		ser->usart_options.baudrate = baudRate;
	}
			
	// Setup other port options
	if(options == NULL)
	{
		//Use defaults
		ser->usart_options.char_length    = US_MR_CHRL_8_BIT;
		ser->usart_options.parity_type    = US_MR_PAR_NO;
		ser->usart_options.stop_bits      = US_MR_NBSTOP_1_BIT;
		ser->usart_options.channel_mode   = US_MR_CHMODE_NORMAL;
	}
	else
	{
		//Use custom settings
		ser->usart_options.char_length    = options->char_length;
		ser->usart_options.parity_type    = options->parity_type;
		ser->usart_options.stop_bits      = options->stop_bits;
		ser->usart_options.channel_mode   = options->channel_mode;
	}
	
	return 0;
}

static int serBufferInit(usartDMA_t *ser, int serialNumber)
{
	//Configure DMA & Buffer
	switch(serialNumber)
	{
		default:
			break;
#if MAX_NUMBER_SERIAL_PORTS >= 1
		case 0:
			ser->usart = ARGN(0, PORT0_CONFIG);
			ser->dmaTx.size			= ARGN(2, PORT0_CONFIG);
			ser->dmaRx.size			= ARGN(4, PORT0_CONFIG);
			ser->dmaTx.dmaChNumber  = ARGN(1, PORT0_CONFIG);
			ser->dmaRx.dmaChNumber  = ARGN(3, PORT0_CONFIG);
			ser->dmaTx.buf = g_serTxDmaBuf_port0;
			ser->dmaRx.buf = g_serRxDmaBuf_port0;
			break;
#endif
#if MAX_NUMBER_SERIAL_PORTS >= 2
		case 1:
			ser->usart = ARGN(0, PORT1_CONFIG);
			ser->dmaTx.size			= ARGN(2, PORT1_CONFIG);
			ser->dmaRx.size			= ARGN(4, PORT1_CONFIG);
			ser->dmaTx.dmaChNumber  = ARGN(1, PORT1_CONFIG);
			ser->dmaRx.dmaChNumber  = ARGN(3, PORT1_CONFIG);
			ser->dmaTx.buf = g_serTxDmaBuf_port1;
			ser->dmaRx.buf = g_serRxDmaBuf_port1;
			break;
#endif
#if MAX_NUMBER_SERIAL_PORTS >= 3
		case 2:
			ser->usart = ARGN(0, PORT2_CONFIG);
			ser->dmaTx.size			= ARGN(2, PORT2_CONFIG);
			ser->dmaRx.size			= ARGN(4, PORT2_CONFIG);
			ser->dmaTx.dmaChNumber  = ARGN(1, PORT2_CONFIG);
			ser->dmaRx.dmaChNumber  = ARGN(3, PORT2_CONFIG);
			ser->dmaTx.buf = g_serTxDmaBuf_port2;
			ser->dmaRx.buf = g_serRxDmaBuf_port2;
			break;
#endif
#if MAX_NUMBER_SERIAL_PORTS >= 4
		case 3:
			ser->usart = ARGN(0, PORT3_CONFIG);
			ser->dmaTx.size			= ARGN(2, PORT3_CONFIG);
			ser->dmaRx.size			= ARGN(4, PORT3_CONFIG);
			ser->dmaTx.dmaChNumber  = ARGN(1, PORT3_CONFIG);
			ser->dmaRx.dmaChNumber  = ARGN(3, PORT3_CONFIG);
			ser->dmaTx.buf = g_serTxDmaBuf_port3;
			ser->dmaRx.buf = g_serRxDmaBuf_port3;
			break;
#endif
#if MAX_NUMBER_SERIAL_PORTS >= 5
		case 4:
			ser->usart = ARGN(0, PORT4_CONFIG);
			ser->dmaTx.size			= ARGN(2, PORT4_CONFIG);
			ser->dmaRx.size			= ARGN(4, PORT4_CONFIG);
			ser->dmaTx.dmaChNumber  = ARGN(1, PORT4_CONFIG);
			ser->dmaRx.dmaChNumber  = ARGN(3, PORT4_CONFIG);
			ser->dmaTx.buf = g_serTxDmaBuf_port4;
			ser->dmaRx.buf = g_serRxDmaBuf_port4;
			break;
#endif
#if MAX_NUMBER_SERIAL_PORTS >= 6
		case 5:
			ser->usart = ARGN(0, PORT5_CONFIG);
			ser->dmaTx.size			= ARGN(2, PORT5_CONFIG);
			ser->dmaRx.size			= ARGN(4, PORT5_CONFIG);
			ser->dmaTx.dmaChNumber  = ARGN(1, PORT5_CONFIG);
			ser->dmaRx.dmaChNumber  = ARGN(3, PORT5_CONFIG);
			ser->dmaTx.buf = g_serTxDmaBuf_port5;
			ser->dmaRx.buf = g_serRxDmaBuf_port5;
			break;
#endif
#if MAX_NUMBER_SERIAL_PORTS >= 7
		case 6:
			ser->usart = ARGN(0, PORT6_CONFIG);
			ser->dmaTx.size			= ARGN(2, PORT6_CONFIG);
			ser->dmaRx.size			= ARGN(4, PORT6_CONFIG);
			ser->dmaTx.dmaChNumber  = ARGN(1, PORT6_CONFIG);
			ser->dmaRx.dmaChNumber  = ARGN(3, PORT6_CONFIG);
			ser->dmaTx.buf = g_serTxDmaBuf_port6;
			ser->dmaRx.buf = g_serRxDmaBuf_port6;
			break;
#endif
#if MAX_NUMBER_SERIAL_PORTS >= 8
		case 7:
			ser->usart = ARGN(0, PORT7_CONFIG);
			ser->dmaTx.size			= ARGN(2, PORT7_CONFIG);
			ser->dmaRx.size			= ARGN(4, PORT7_CONFIG);
			ser->dmaTx.dmaChNumber  = ARGN(1, PORT7_CONFIG);
			ser->dmaRx.dmaChNumber  = ARGN(3, PORT7_CONFIG);
			ser->dmaTx.buf = g_serTxDmaBuf_port7;
			ser->dmaRx.buf = g_serRxDmaBuf_port7;
			break;
#endif
	}

	// Setup hardware specific items
	if( ser->usart == USART0 )
	{
		ser->uinfo.ul_id             = ID_USART0;
		ser->uinfo.usartTxTHR        = (uint32_t)&(USART0->US_THR);
		ser->uinfo.usartRxRHR        = (uint32_t)&(USART0->US_RHR);
		ser->uinfo.xdmacUsartTxPerId = XDMAC_PERID_USART0_TX;
		ser->uinfo.xdmacUsartRxPerId = XDMAC_PERID_USART0_RX;
		ser->uinfo.isUsartNotUart    = true;
		ser->uinfo.isSpiUsart		 = false;
	}
	else if( ser->usart == USART1 )
	{
		ser->uinfo.ul_id             = ID_USART1;
		ser->uinfo.usartTxTHR        = (uint32_t)&(USART1->US_THR);
		ser->uinfo.usartRxRHR        = (uint32_t)&(USART1->US_RHR);
		ser->uinfo.xdmacUsartTxPerId = XDMAC_PERID_USART1_TX;
		ser->uinfo.xdmacUsartRxPerId = XDMAC_PERID_USART1_RX;
		ser->uinfo.isUsartNotUart    = true;
		ser->uinfo.isSpiUsart		 = false;
	}
	else if( ser->usart == USART2 )
	{
		ser->uinfo.ul_id             = ID_USART2;
		ser->uinfo.usartTxTHR        = (uint32_t)&(USART2->US_THR);
		ser->uinfo.usartRxRHR        = (uint32_t)&(USART2->US_RHR);
		ser->uinfo.xdmacUsartTxPerId = XDMAC_PERID_USART2_TX;
		ser->uinfo.xdmacUsartRxPerId = XDMAC_PERID_USART2_RX;
		ser->uinfo.isUsartNotUart    = true;
#if CONF_BOARD_USART_SPI == 1
		ser->uinfo.isSpiUsart        = g_spi_comm_select;
#else
		ser->uinfo.isSpiUsart		 = false;
#endif
	}
	else if( ser->usart == UART0 )
	{
		ser->uinfo.ul_id             = ID_UART0;
		ser->uinfo.usartTxTHR        = (uint32_t)&(UART0->UART_THR);
		ser->uinfo.usartRxRHR        = (uint32_t)&(UART0->UART_RHR);
		ser->uinfo.xdmacUsartTxPerId = XDMAC_PERID_UART0_TX;
		ser->uinfo.xdmacUsartRxPerId = XDMAC_PERID_UART0_RX;
		ser->uinfo.isUsartNotUart	 = false;
		ser->uinfo.isSpiUsart		 = false;
	}
	else if( ser->usart == UART1 )
	{
		ser->uinfo.ul_id             = ID_UART1;
		ser->uinfo.usartTxTHR        = (uint32_t)&(UART1->UART_THR);
		ser->uinfo.usartRxRHR        = (uint32_t)&(UART1->UART_RHR);
		ser->uinfo.xdmacUsartTxPerId = XDMAC_PERID_UART1_TX;
		ser->uinfo.xdmacUsartRxPerId = XDMAC_PERID_UART1_RX;
		ser->uinfo.isUsartNotUart	 = false;
		ser->uinfo.isSpiUsart		 = false;
	}
	else if( ser->usart == UART2 )
	{
		ser->uinfo.ul_id             = ID_UART2;
		ser->uinfo.usartTxTHR        = (uint32_t)&(UART2->UART_THR);
		ser->uinfo.usartRxRHR        = (uint32_t)&(UART2->UART_RHR);
		ser->uinfo.xdmacUsartTxPerId = XDMAC_PERID_UART2_TX;
		ser->uinfo.xdmacUsartRxPerId = XDMAC_PERID_UART2_RX;
		ser->uinfo.isUsartNotUart	 = false;
		ser->uinfo.isSpiUsart		 = false;
	}
	else if( ser->usart == UART3 )
	{
		ser->uinfo.ul_id             = ID_UART3;
		ser->uinfo.usartTxTHR        = (uint32_t)&(UART3->UART_THR);
		ser->uinfo.usartRxRHR        = (uint32_t)&(UART3->UART_RHR);
		ser->uinfo.xdmacUsartTxPerId = XDMAC_PERID_UART3_TX;
		ser->uinfo.xdmacUsartRxPerId = XDMAC_PERID_UART3_RX;
		ser->uinfo.isUsartNotUart    = false;
		ser->uinfo.isSpiUsart        = false;
	}
	else if( ser->usart == UART4 )
	{
		ser->uinfo.ul_id             = ID_UART4;
		ser->uinfo.usartTxTHR        = (uint32_t)&(UART4->UART_THR);
		ser->uinfo.usartRxRHR        = (uint32_t)&(UART4->UART_RHR);
		ser->uinfo.xdmacUsartTxPerId = XDMAC_PERID_UART4_TX;
		ser->uinfo.xdmacUsartRxPerId = XDMAC_PERID_UART4_RX;
		ser->uinfo.isUsartNotUart    = false;
		ser->uinfo.isSpiUsart        = false;
	}
	else
	{
#ifdef USB_PORT_NUM
		if( serialNumber != USB_PORT_NUM )
#endif		
		{
			// "Invalid USART selected!!!"
			return -1;
		}
	}        

	if (ser->dmaTx.buf == NULL || ser->dmaRx.buf == NULL || !IS_32B_ALIGNED(ser->dmaTx.buf) || !IS_32B_ALIGNED(ser->dmaRx.buf))
	{
// #ifndef __INERTIAL_SENSE_EVB_2__
// extern void soft_reset_backup_register(uint32_t value, uint32_t subValue);
// 		soft_reset_backup_register(CRASH_INFO_INVALID_CODE_OPERATION, 0);
// #endif
	}

	// Set pointer to start of buffer
	ser->dmaTx.ptr = ser->dmaTx.buf;
	ser->dmaRx.ptr = ser->dmaRx.buf;
	
	// Set pointer to end of buffer
	ser->dmaTx.end = ser->dmaTx.buf + ser->dmaTx.size;
	ser->dmaRx.end = ser->dmaRx.buf + ser->dmaRx.size;

#ifdef USB_PORT_NUM
	if( serialNumber == USB_PORT_NUM )
	{
		ser->dmaTx.end = ser->dmaTx.ptr;
	}
#endif

	return 0;
}

int serInit(int serialNum, uint32_t baudRate, sam_usart_opt_t *options, uint32_t* overrunStatus)
{
#ifdef USB_PORT_NUM
	if(serialNum == USB_PORT_NUM)
	{
		// Setup pointers for additional USB buffer
		if (serBufferInit((usartDMA_t *)&g_usartDMA[USB_PORT_NUM], serialNum) != 0)
			return -1;

		udc_start();

		#if (ENABLE_COMM_LOOPBACK_DRIVER==1 || ENABLE_COMM_LOOPBACK_DRIVER==2)
			serLoopback(serialNum);
		#endif // ENABLE_COMM_LOOPBACK_DRIVER == 1 || 2
	
		return 0;
	}
#endif

	if(overrunStatus)
	{	// Set buffer overrun status pointer
		s_overrunStatus = overrunStatus;
	}
	
	// Validate port number
	while( serialNum >= MAX_NUMBER_SERIAL_PORTS ) { /* Invalid port number */ }

	usartDMA_t *ser = (usartDMA_t *)&g_usartDMA[serialNum];

	// Setup UART
	if (setup_usart_info(ser, serialNum, baudRate, options) != 0)
		while(1);

	// Setup buffers
	if (serBufferInit(ser, serialNum) != 0)
		return -1;

	// Enable the peripheral clock in the PMC
	sysclk_enable_peripheral_clock(ser->uinfo.ul_id);

	// Re-init UART
	serSetBaudRate(serialNum, ser->usart_options.baudrate);

	// Enable interrupt for errors
	usart_enable_interrupt((Usart *)ser->usart, UART_IER_OVRE | UART_IER_FRAME | UART_IER_PARE);
	NVIC_EnableIRQ(ser->uinfo.ul_id);
	
	/* Initialize and enable DMA controller */
	pmc_enable_periph_clk(ID_XDMAC);

	/* Get pointer to XDMA channel id */
	Assert(XDMAC);
	Assert(ser->dmaTx.dmaChNumber < XDMACCHID_NUMBER);
	Assert(ser->dmaRx.dmaChNumber < XDMACCHID_NUMBER);
	ser->dmaTx.dmaChId = &(XDMAC->XDMAC_CHID[ser->dmaTx.dmaChNumber]);
	ser->dmaRx.dmaChId = &(XDMAC->XDMAC_CHID[ser->dmaRx.dmaChNumber]);

	serDmaTxInit(ser);
	serDmaRxInit(ser);

	// Enable Rx, only enable Tx when ready to transmit data.
	// flush descriptors, etc. so that dma enable reads fresh values
#if CONF_BOARD_ENABLE_DCACHE == 1
	SCB_CleanInvalidateDCache();
#endif
	XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << (ser->dmaRx.dmaChNumber));

	// For SPI mode, TX DMA needs to run all the time
	if (ser->uinfo.isSpiUsart)
	{
		XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << (ser->dmaTx.dmaChNumber));
	}

#if (ENABLE_COMM_LOOPBACK_DRIVER > 0)
	serLoopback(serialNum);
#endif

	return 0;
}

//Interrupts get enabled for errors. Clear error in interrupt.
void UART0_Handler(void) { UART0->UART_CR = UART_CR_RSTSTA; }
void UART1_Handler(void) { UART1->UART_CR = UART_CR_RSTSTA; }
void UART2_Handler(void) { UART2->UART_CR = UART_CR_RSTSTA; }
void UART3_Handler(void) { UART3->UART_CR = UART_CR_RSTSTA; }
void UART4_Handler(void) { UART4->UART_CR = UART_CR_RSTSTA; }
void USART0_Handler(void) { USART0->US_CR = US_CR_RSTSTA; }
void USART1_Handler(void) { USART1->US_CR = US_CR_RSTSTA; }
void USART2_Handler(void) { USART2->US_CR = US_CR_RSTSTA; }
