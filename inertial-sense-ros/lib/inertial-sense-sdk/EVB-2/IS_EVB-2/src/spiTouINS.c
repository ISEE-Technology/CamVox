/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "spiTouINS.h"
#include "board_opt.h"
#include "drivers/d_dma.h"
#include "../../../hw-libs/misc/rtos.h"

#define SPI_INS_BAUDRATE	10000000UL

// SPI Channel
#define SPI_INS_BASE			SPI0
#define SPI_INS_CHIP_SEL		2
#define SPI_spiTouINS_Handler	SPI0_Handler
#define SPI_IRQn				SPI0_IRQn
#define SPI_XDMAC_TX_CH_NUM		1
#define SPI_XDMAC_RX_CH_NUM		2

// Clock polarity & phase. Set to mode 3.
#define SPI_INS_CLK_POLARITY		1
#define SPI_INS_CLK_PHASE			0

// Delay before SPCK.
#define SPI_DLYBS 0x10

// Delay between consecutive transfers.
#define SPI_DLYBCT 0x03

#define TX_BUFFER_SIZE	512
#define RX_BUFFER_SIZE	2048		//Needs to be a power of 2 (2^x)
#define RX_INT_BUFFER_SIZE	1024	//Needs to be a power of 2 (2^x)

#define READ_SIZE	100
#define READ_ADDITIONAL_SIZE	20

COMPILER_ALIGNED(32) static volatile uint8_t TxBuf[TX_BUFFER_SIZE];
COMPILER_ALIGNED(32) static volatile uint8_t TxBlank[READ_SIZE];
COMPILER_ALIGNED(32) static volatile uint8_t RxBuf[RX_BUFFER_SIZE];
COMPILER_ALIGNED(32) static volatile uint8_t RxBufInternal[RX_INT_BUFFER_SIZE];
static volatile uint32_t rxfptr = 0, rxbptr = 0, rxintptr = 0;
static volatile uint8_t *txfptr = TxBuf;

//Pointers for the lld_view0 array
volatile uint32_t lld_fptr;
volatile uint32_t lld_bptr;

// DMA linked list descriptor used for reload
#define DMA_LLD_COUNT	32	//Must be 2^x in size
#define DMA_LLD_MASK	(DMA_LLD_COUNT - 1)
COMPILER_WORD_ALIGNED volatile lld_view0 lld[DMA_LLD_COUNT];
COMPILER_WORD_ALIGNED volatile lld_view0 rx_lld;
	
static void sendMoreData(int len);

//Task stuff
#define TASK_SPI_TO_UINS_STACK_SIZE				(512/sizeof(portSTACK_TYPE))
#define TASK_SPI_TO_UINS_PRIORITY				(configMAX_PRIORITIES - 2)  // We want this to be the highest priority - Only timer task higher
#define TASK_SPI_TO_UINS_PERIOD_MS				1
static TaskHandle_t xHandlingTask;

void XDMAC_spiTouINS_Handler(void)
{
	if( (xdmac_channel_get_interrupt_status(XDMAC, DMA_CH_SPI_INS_TX) & XDMAC_CIS_BIS) && (xdmac_channel_get_interrupt_mask(XDMAC, DMA_CH_SPI_INS_TX) & XDMAC_CIE_BIE) )
	{
		//Check to see if DMA is finished (as indicated by channel becoming disabled)
		if ((XDMAC->XDMAC_GS & (XDMAC_GS_ST0 << DMA_CH_SPI_INS_TX)) == 0)
		{
			if(lld_bptr != lld_fptr)	//Not equal means there is data ready to send
			{
				//Setup TX DMA
				xdmac_channel_set_source_addr(XDMAC, DMA_CH_SPI_INS_TX, lld[lld_bptr].mbr_ta);
				xdmac_channel_set_microblock_control(XDMAC, DMA_CH_SPI_INS_TX, lld[lld_bptr].mbr_ubc);

				//Move pointer
				lld_bptr = (lld_bptr + 1) & DMA_LLD_MASK;

				//Enable - We don't need to clean cache as DMA data is already flushed to main memory
				XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << DMA_CH_SPI_INS_TX);
			}
			else
			{
				//We are done with the list
				xdmac_channel_disable_interrupt(XDMAC, DMA_CH_SPI_INS_TX, XDMAC_CIE_BIE);
		
				//See if we need to receive more or lower CS line		
				if(ioport_get_pin_level(INS_DATA_RDY_PIN_IDX))
				{
					//Keep receiving
					sendMoreData(READ_ADDITIONAL_SIZE);
				}
				else
				{
					spi_enable_interrupt(SPI_INS_BASE, SPI_IER_TDRE);
				}				
			}
			
			//Notify task to process data
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xTaskNotifyFromISR(g_rtos.task[EVB_TASK_SPI_UINS_COM].handle, 1, eSetBits, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}		
}

void SPI_spiTouINS_Handler(void)
{
	static volatile bool inDataPacket = false;
	
	uint32_t spi_statusReg;

//	PIOC->PIO_SODR = 1U << 20;

	if((SPI_INS_BASE->SPI_IMR & SPI_IMR_TDRE) && (SPI_INS_BASE->SPI_SR & SPI_SR_TXEMPTY))
	{
		//Turn off interrupt as we are either done or sending more data
		spi_disable_interrupt(SPI_INS_BASE, SPI_IER_TDRE);

		if(inDataPacket || ioport_get_pin_level(INS_DATA_RDY_PIN_IDX))
		{
			//Keep receiving
			sendMoreData(READ_ADDITIONAL_SIZE);
		}
		else
		{
			//Deactivate CS line
			ioport_set_pin_level(SPI_INS_CS_PIN, IOPORT_PIN_LEVEL_HIGH);

			//Enable Data Ready interrupt
			pio_get_interrupt_status(INS_DATA_RDY_PIN_PIO);
			pio_enable_interrupt(INS_DATA_RDY_PIN_PIO, INS_DATA_RDY_PIN_MASK);
		}
	}
	
//	PIOC->PIO_CODR = 1U << 20;
}

static void PIO_DataReady_Handler(uint32_t id, uint32_t mask)
{
	//Clear interrupt by reading status
	pio_get_interrupt_status(INS_DATA_RDY_PIN_PIO);

	if (INS_DATA_RDY_PIN_ID == id && (INS_DATA_RDY_PIN_MASK & mask))
	{
		pio_disable_interrupt(INS_DATA_RDY_PIN_PIO, INS_DATA_RDY_PIN_MASK);

		//Start transmission
		sendMoreData(READ_SIZE);
	}
}

static void spiTouINS_task(void *pvParameters)
{
	UNUSED(pvParameters);
	static volatile bool inDataPacket = false;

	while(1)
	{
//		PIOC->PIO_CODR = 1U << 20;

		//Wait to be notified by interrupt
		xTaskNotifyWait( pdFALSE, 0xFFFFFFFFUL, 0, pdMS_TO_TICKS(1000));		

//		PIOC->PIO_SODR = 1U << 20;
		
		//Check and process any data in the internal buffer into the external buffer.

		//Flush FIFO, content of the FIFO is written to memory
		xdmac_channel_software_flush_request(XDMAC, DMA_CH_SPI_INS_RX);
		
		//Find how much data we have
		uint32_t bytesReady;
		uint32_t curDmaAddr = (uint32_t)XDMAC->XDMAC_CHID[DMA_CH_SPI_INS_RX].XDMAC_CDA - (uint32_t)RxBufInternal;
		if (curDmaAddr < rxintptr)
			bytesReady = RX_INT_BUFFER_SIZE - rxintptr + curDmaAddr;
		else
			bytesReady = curDmaAddr - rxintptr;
	
		for(uint32_t i=0; i<bytesReady; i++)
		{
			if(inDataPacket)
			{
				//Store data in buffer
				RxBuf[rxfptr] = RxBufInternal[rxintptr];

				//Check to see if it is the end of packet
				if(0xFE == RxBufInternal[rxintptr])
				{
					inDataPacket = false;
				}
				
				//Move to next location
				rxfptr = (rxfptr + 1) & (RX_BUFFER_SIZE - 1);
			}
			else
			{
				//Watch for start of packet
				if(0xFF == RxBufInternal[rxintptr])
				{
					inDataPacket = true;

					//Store data in buffer
					RxBuf[rxfptr] = RxBufInternal[rxintptr];
					rxfptr = (rxfptr + 1) & (RX_BUFFER_SIZE - 1);
				}
			}	
			
			//Move internal buffer pointer
			rxintptr = (rxintptr + 1) & (RX_INT_BUFFER_SIZE - 1);
		}
	}
}

//The optimizer seems to allow the use of the lld array before it is completely configured. No optimization prevents that from happening.
__attribute__((optimize("O0")))
void spiTouINS_init(void)
{    
//Setup SPI Port
	spi_enable_clock(SPI_INS_BASE);
	spi_disable(SPI_INS_BASE);
	spi_reset(SPI_INS_BASE);
	
	spi_set_lastxfer(SPI_INS_BASE);
	spi_set_master_mode(SPI_INS_BASE);
	spi_disable_mode_fault_detect(SPI_INS_BASE);
	
	spi_set_peripheral_chip_select_value(SPI_INS_BASE, spi_get_pcs(SPI_INS_CHIP_SEL));
	
	spi_set_clock_polarity(SPI_INS_BASE, SPI_INS_CHIP_SEL, SPI_INS_CLK_POLARITY);
	spi_set_clock_phase(SPI_INS_BASE, SPI_INS_CHIP_SEL, SPI_INS_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_INS_BASE, SPI_INS_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	
	spi_set_baudrate_div(SPI_INS_BASE, SPI_INS_CHIP_SEL, (sysclk_get_peripheral_hz() / SPI_INS_BAUDRATE));
	spi_set_transfer_delay(SPI_INS_BASE, SPI_INS_CHIP_SEL, SPI_DLYBS, SPI_DLYBCT);

	spi_enable(SPI_INS_BASE);

	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(SPI_IRQn);

//DMA for incoming
	xdmac_channel_config_t cfg = {0};
	cfg.mbr_sa = (uint32_t)&(SPI_INS_BASE->SPI_RDR);
	cfg.mbr_da = (uint32_t)RxBufInternal;
	cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
	XDMAC_CC_MEMSET_NORMAL_MODE |
	XDMAC_CC_DSYNC_PER2MEM |
	XDMAC_CC_MBSIZE_SINGLE |
	XDMAC_CC_DWIDTH_BYTE |
	XDMAC_CC_SIF_AHB_IF1 |
	XDMAC_CC_DIF_AHB_IF1 |
	XDMAC_CC_SAM_FIXED_AM |
	XDMAC_CC_DAM_INCREMENTED_AM |
	XDMAC_CC_PERID(SPI_XDMAC_RX_CH_NUM);
	cfg.mbr_ubc = RX_INT_BUFFER_SIZE;
	xdmac_configure_transfer(XDMAC, DMA_CH_SPI_INS_RX, &cfg);

	// Configure linked list descriptor (only uses one)
	rx_lld.mbr_ta = (uint32_t)RxBufInternal;
	rx_lld.mbr_nda = (uint32_t)&rx_lld; // point to self
	rx_lld.mbr_ubc =
		XDMAC_UBC_NVIEW_NDV0 |
		XDMAC_UBC_NDE_FETCH_EN |
		XDMAC_UBC_NDEN_UPDATED |
		RX_INT_BUFFER_SIZE;

	// Set initial descriptor control
	xdmac_channel_set_descriptor_control(XDMAC, DMA_CH_SPI_INS_RX,	//Updates CNDC register
	XDMAC_CNDC_NDVIEW_NDV0 |
	XDMAC_CNDC_NDE_DSCR_FETCH_EN |
	XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED);
	xdmac_channel_set_descriptor_addr(XDMAC, DMA_CH_SPI_INS_RX, (uint32_t)&rx_lld, 0);	//Updates CNDA register
	
	rxintptr = 0;
	SCB_CLEAN_DCACHE_BY_ADDR_32BYTE_ALIGNED(&rx_lld, sizeof(rx_lld));
	XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << DMA_CH_SPI_INS_RX);
	
//Configure task for processing incoming data
	createTask(EVB_TASK_SPI_UINS_COM, spiTouINS_task,  "SPI_UINS",  TASK_SPI_TO_UINS_STACK_SIZE,  NULL, TASK_SPI_TO_UINS_PRIORITY,  TASK_SPI_TO_UINS_PERIOD_MS);

//DMA for outgoing
	cfg.mbr_sa = (uint32_t)TxBuf;
	cfg.mbr_da = (uint32_t)&(SPI_INS_BASE->SPI_TDR);
	cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
	XDMAC_CC_MEMSET_NORMAL_MODE | 
	XDMAC_CC_DSYNC_MEM2PER |
	XDMAC_CC_MBSIZE_SINGLE |
	XDMAC_CC_CSIZE_CHK_1 |
	XDMAC_CC_DWIDTH_BYTE |
	XDMAC_CC_SIF_AHB_IF1 |
	XDMAC_CC_DIF_AHB_IF1 |
	XDMAC_CC_SAM_INCREMENTED_AM |
	XDMAC_CC_DAM_FIXED_AM |
	XDMAC_CC_PERID(SPI_XDMAC_TX_CH_NUM);
	cfg.mbr_ubc = 0;
	xdmac_configure_transfer(XDMAC, DMA_CH_SPI_INS_TX, &cfg);
	
	//Setup pointers to lld array
	lld_fptr = 0;
	lld_bptr = 0;

	//Setup interrupts for DMA for UART mode (gets multiple setups as this routine is called for multiple USARTs)
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);	//Must not be higher than RTOS allows
	NVIC_EnableIRQ(XDMAC_IRQn);

//Setup data ready pin
	pio_handler_set(INS_DATA_RDY_PIN_PIO, INS_DATA_RDY_PIN_ID, INS_DATA_RDY_PIN_MASK, PIO_DEBOUNCE | PIO_IT_HIGH_LEVEL, PIO_DataReady_Handler);
	NVIC_EnableIRQ((IRQn_Type)INS_DATA_RDY_PIN_ID);

	//Enable Data Ready interrupt
	pio_handler_set_priority(INS_DATA_RDY_PIN_PIO, (IRQn_Type) INS_DATA_RDY_PIN_ID, 3);	
	pio_get_interrupt_status(INS_DATA_RDY_PIN_PIO);
	pio_enable_interrupt(INS_DATA_RDY_PIN_PIO, INS_DATA_RDY_PIN_MASK);    
	
	//Haven't figured out why, but the first packet out is garbage so send one now
	//Maybe it has something to do with the mode we are in, because before we send the data/clock lines are not in their default states
	spi_write_single(SPI_INS_BASE, 0);
}

//This routine is called by interrupts to read data out of the uINS
//Don't use outside this module.
static void sendMoreData(int len)
{
	pio_disable_interrupt(INS_DATA_RDY_PIN_PIO, INS_DATA_RDY_PIN_MASK);
	spi_disable_interrupt(SPI_INS_BASE, SPI_IER_TDRE);

	//Check for sending overflow
	if(((lld_fptr + 1) & DMA_LLD_MASK) == lld_bptr )
		return;	//This isn't great

	// Setup DMA to transfer some more data
	lld[lld_fptr].mbr_ta = (uint32_t)TxBlank;
	lld[lld_fptr].mbr_ubc = XDMAC_UBC_UBLEN(len);

	// Move to next lld
	lld_fptr = (lld_fptr + 1) & DMA_LLD_MASK;	

	//Block interrupt handler from running and causing a double configure
	NVIC_DisableIRQ(XDMAC_IRQn);

	//Enable DMA if it isn't running
	if((XDMAC->XDMAC_GS & (XDMAC_GS_ST0 << DMA_CH_SPI_INS_TX)) == 0)
	{
		if(lld_bptr != lld_fptr)
		{
			//Activate CS line
			ioport_set_pin_level(SPI_INS_CS_PIN, IOPORT_PIN_LEVEL_LOW);
		
			//Setup TX DMA
			xdmac_channel_set_source_addr(XDMAC, DMA_CH_SPI_INS_TX, lld[lld_bptr].mbr_ta);
			xdmac_channel_set_microblock_control(XDMAC, DMA_CH_SPI_INS_TX, lld[lld_bptr].mbr_ubc);

			//Move pointer
			lld_bptr = (lld_bptr + 1) & DMA_LLD_MASK;

			// Enable DMA interrupt
			xdmac_enable_interrupt(XDMAC, DMA_CH_SPI_INS_TX);
			spi_disable_interrupt(SPI_INS_BASE, SPI_IER_TDRE);
			xdmac_channel_enable_interrupt(XDMAC, DMA_CH_SPI_INS_TX, XDMAC_CIE_BIE);

			//Enable - cache should have already been cleaned
			XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << DMA_CH_SPI_INS_TX);
		}
	}

	NVIC_EnableIRQ(XDMAC_IRQn);
}

static uint32_t getTxFree(void)
{
	if ((XDMAC->XDMAC_GS & (XDMAC_GS_ST0 << DMA_CH_SPI_INS_TX)) == 0)
		return TX_BUFFER_SIZE;

	uint8_t* curDmaAddr = XDMAC->XDMAC_CHID[DMA_CH_SPI_INS_TX].XDMAC_CDA;

	if (txfptr < curDmaAddr)
		return (int)(curDmaAddr - txfptr);
	else
		return TX_BUFFER_SIZE - (int)(txfptr - curDmaAddr);	
}

int spiTouINS_serWrite(const unsigned char *buf, int size)
{
	if (size <= 0) return 0;
	if (g_rtos.task[EVB_TASK_SPI_UINS_COM].handle == 0) return 0;	//Block executing if we have not been configured
	
	//Disable interrupt so we don't get double writes
	pio_disable_interrupt(INS_DATA_RDY_PIN_PIO, INS_DATA_RDY_PIN_MASK);
	spi_disable_interrupt(SPI_INS_BASE, SPI_IER_TDRE);

	// Bytes free in DMA buffer
	int dmaFreeBytes = getTxFree();

	taskENTER_CRITICAL();
	
	// Limit size
	if (size > dmaFreeBytes || ((lld_fptr + 1) & DMA_LLD_MASK) == lld_bptr )
		return 0;

	//////////////////////////////////////////////////////////////////////////
	// Copy into DMA buffer
	// Bytes before end of DMA buffer
	int bytesToEnd = (TxBuf + TX_BUFFER_SIZE) - txfptr;

	// Add data to ring buffer
	if (size <= bytesToEnd)
	{
		// Don't need to wrap
		// Copy data into DMA buffer
		MEMCPY_DCACHE_CLEAN(txfptr, buf, size);

		// Setup DMA
		lld[lld_fptr].mbr_ta = (uint32_t)txfptr;
		lld[lld_fptr].mbr_ubc = XDMAC_UBC_UBLEN(size);

		// Increment buffer pointer
		txfptr += size;

		// Move to next lld
		lld_fptr = (lld_fptr + 1) & DMA_LLD_MASK;
	}
	else	// Need to wrap
	{
		// Bytes to write at buffer start
		int bytesWrapped = size - bytesToEnd;
		uint32_t bufptr = lld_fptr;
		int count = 1;
		
		//Write out data needed to fill the end of the buffer
		if (bytesToEnd)
		{
			// Not already at buffer end.  Fill buffer to end.
			// Copy data into DMA buffer & update transfer size
			MEMCPY_DCACHE_CLEAN(txfptr, buf, bytesToEnd);

			// Setup DMA for end
			lld[bufptr].mbr_ta = (uint32_t)txfptr;
			lld[bufptr].mbr_ubc = XDMAC_UBC_UBLEN(bytesToEnd);

			// Move to next lld
			bufptr = (bufptr + 1) & DMA_LLD_MASK;
			count++;
		}
		
		// Copy data into DMA buffer start & update pointer
		MEMCPY_DCACHE_CLEAN(TxBuf, buf + bytesToEnd, bytesWrapped);

		// Setup DMA for beginning
		lld[bufptr].mbr_ta = (uint32_t)TxBuf;
		lld[bufptr].mbr_ubc = XDMAC_UBC_UBLEN(bytesWrapped);

		// Increment buffer pointer
		txfptr = TxBuf + bytesWrapped;

		// Move to next lld
		lld_fptr = (lld_fptr + count) & DMA_LLD_MASK;
	}

	//////////////////////////////////////////////////////////////////////////
	// Transfer out of DMA buffer

	//Block interrupt handler from running and causing a double configure
	NVIC_DisableIRQ(XDMAC_IRQn);

	//Enable DMA if it isn't running
	if((XDMAC->XDMAC_GS & (XDMAC_GS_ST0 << DMA_CH_SPI_INS_TX)) == 0)
	{
		if(lld_bptr != lld_fptr)
		{
			//Activate CS line
			ioport_set_pin_level(SPI_INS_CS_PIN, IOPORT_PIN_LEVEL_LOW);
		
			//Setup TX DMA
			xdmac_channel_set_source_addr(XDMAC, DMA_CH_SPI_INS_TX, lld[lld_bptr].mbr_ta);
			xdmac_channel_set_microblock_control(XDMAC, DMA_CH_SPI_INS_TX, lld[lld_bptr].mbr_ubc);

			//Move pointer
			lld_bptr = (lld_bptr + 1) & DMA_LLD_MASK;

			// Enable DMA interrupt
			xdmac_enable_interrupt(XDMAC, DMA_CH_SPI_INS_TX);
			spi_disable_interrupt(SPI_INS_BASE, SPI_IER_TDRE);
			xdmac_channel_enable_interrupt(XDMAC, DMA_CH_SPI_INS_TX, XDMAC_CIE_BIE);

			//Enable - cache should have already been cleaned
			XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << DMA_CH_SPI_INS_TX);
		}
	}

	NVIC_EnableIRQ(XDMAC_IRQn);

	taskEXIT_CRITICAL();
	
	return size;
}

int spiTouINS_dataReady(void)
{
	int dataAvail = rxfptr - rxbptr;
	if(dataAvail < 0)
		return RX_BUFFER_SIZE + dataAvail;
	else
		return dataAvail;
}

int spiTouINS_serRead(unsigned char *buf, int size)
{
	if(rxfptr == rxbptr)	//No data available.
		return 0;
	
	int dataAvail = rxfptr - rxbptr;
	
	if(dataAvail < 0)	//Take care of buffer wrap
	{
		// Limit to bytes available
		size = Min(size, RX_BUFFER_SIZE + dataAvail);
		int bytesToEnd = RX_BUFFER_SIZE - rxbptr;

		// Copy data
		if(size <= bytesToEnd)
		{
			memcpy(buf, (const uint8_t*)&RxBuf[rxbptr], size);
		}
		else
		{
			memcpy(buf, (const uint8_t*)&RxBuf[rxbptr], bytesToEnd);
			memcpy(&buf[bytesToEnd], (const uint8_t*)RxBuf, size - bytesToEnd);
		}
	}
	else
	{
		// Limit to bytes available
		size = Min(size, dataAvail);		
		
		// Copy data
		memcpy(buf, (const uint8_t*)&RxBuf[rxbptr], size);
	}

	// Move pointer
	rxbptr = (rxbptr + size) & (RX_BUFFER_SIZE - 1);

	return size;
}



void test_spiTouINS(void)
{
#if 0
    vTaskDelay(1000);
    udi_cdc_write_buf("Testing\r\n", 9);
    
    //Clear uINS buffer
    {
        #define SIZE 500
        uint8_t buf[SIZE];
        buf[0] = 0xFF;
        buf[1] = 0xFE;
        for(int i=2; i<SIZE; i++)
        buf[i] = 0;
        spiTouINS_serWrite(buf, SIZE);
        vTaskDelay(1000);
        spiTouINS_serRead(buf, SIZE);
    }

    int packet_count = 0;
    int failure = 0;
    int seed = 0;
    
    uint32_t total = 0;
    int seconds = 0;
    
    while(1)
    {
        #define BUF_SIZE 100
        uint8_t tx_buf[BUF_SIZE];
        uint8_t rx_buf[BUF_SIZE];
        char str[BUF_SIZE];
        int tx_len, rx_len, len;

        //Make packet
        tx_buf[0] = 0xFF;	//Start
        rx_buf[0] = 0;
        len = 60 + rand() % 30;
        for(tx_len = 1; tx_len < len; tx_len++)
        {
            tx_buf[tx_len] = tx_len + seed;
            rx_buf[tx_len] = 0;
        }
        tx_buf[tx_len] = 0xFE;	//End
        rx_buf[tx_len++] = 0;
        if(++seed >= 100)
        seed = 0;
        total += tx_len;
        
        //Send data
        spiTouINS_serWrite(tx_buf, tx_len);
        
        //Wait for TX/RX (with timeout)
        volatile uint32_t j=20000;
        while(spiTouINS_dataReady() < tx_len)
        {
            if(--j == 0)
            break;
        }
        
        //Read data
        rx_len = spiTouINS_serRead(rx_buf, BUF_SIZE);
        
        if(tx_len != rx_len)
        {
            len = sprintf(str, "len %d %d %d\r\n", tx_len, rx_len, packet_count);
            udi_cdc_write_buf(str, len);
        }
        
        bool issue = false;
        for(int i=0;i<tx_len;i++)
        {
            if(rx_buf[i] != tx_buf[i])
            {
                len = sprintf(str, "byte %d %d ", tx_buf[i], rx_buf[i]);
                udi_cdc_write_buf(str, len);
                issue = true;
            }
        }
        if(issue)
        {
            udi_cdc_write_buf("\r\n", 2);
            failure++;
        }
        
        ++packet_count;
        /*			if(packet_count % 1000 == 0)
        {
        len = sprintf(str, "Packets %d %d\r\n", packet_count, failure);
        udi_cdc_write_buf(str, len);
        }
        */
        if((RTC->RTC_SR & RTC_SR_SEC) == RTC_SR_SEC)
        {
            RTC->RTC_SCCR = RTC_SCCR_SECCLR;
            
            if(++seconds >= 10)
            {
                len = sprintf(str, "Packets %d, failures %d, rate %.01f kB/s (each direction)\r\n", packet_count, failure, (float)(total / seconds) / 1000.0);
                udi_cdc_write_buf(str, len);
                
                total = 0;
                seconds = 0;
            }
            
        }
    }
#endif
    
#if 0
    while(1)
    {
        #define BUF_SIZE 256
        uint8_t buf[BUF_SIZE];
        int len;

        //USB -> SPI
        while(udi_cdc_is_rx_ready())
        {
            len = udi_cdc_read_no_polling(buf, BUF_SIZE);
            spiTouINS_serWrite(buf, len);
        }
        
        //SPI -> USB
        while((len = spiTouINS_serRead(buf, BUF_SIZE)) > 0)
        {
            if(g_usb_cdc_open && udi_cdc_is_tx_ready())
            {
                udi_cdc_write_buf(buf, len);
            }
        }
        
        vTaskDelay(1);
    }
#endif
    
}
