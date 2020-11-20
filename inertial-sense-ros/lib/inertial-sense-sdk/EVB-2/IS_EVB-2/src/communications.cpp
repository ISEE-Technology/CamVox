/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "../../../src/ISUtilities.h"
#include "../../../src/ISLogger.h"
#include "../../../hw-libs/misc/bootloaderApp.h"
#include "../drivers/d_quadEnc.h"
#include "ISLogFileFatFs.h"
#include "xbee.h"
#include "wifi.h"
#include "sd_card_logger.h"
#include "communications.h"
#include "CAN.h"
#include "../hw-libs/communications/CAN_comm.h"
#include "../src/protocol_nmea.h"

typedef struct
{
	// Comm instances
	is_comm_instance_t comm;

	// Comm instance data buffer
	uint8_t comm_buffer[PKT_BUF_SIZE];

} comm_rx_port_t;


#define COM_RX_PORT_COUNT	(EVB2_PORT_COUNT-1)		// exclude CAN port
comm_rx_port_t              g_comRxPort[COM_RX_PORT_COUNT] = {};
static uint8_t				s_rxDecodeBuffer[PKT_BUF_SIZE] = {};

StreamBufferHandle_t        g_xStreamBufferUINS;
StreamBufferHandle_t        g_xStreamBufferWiFiRx;
StreamBufferHandle_t        g_xStreamBufferWiFiTx;

int comWrite(int serialNum, const unsigned char *buf, int size, uint32_t ledPin )
{
    int len;
    if (serialNum == EVB2_PORT_UINS1 && g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE)
    {
        len = spiTouINS_serWrite(buf, size);
    }
    else
    {
        len = serWrite(serialNum, buf, size);
    }
    
    if(len)
    {
        LED_ON(ledPin);
    }
    return len;
}

int comRead(int serialNum, unsigned char *buf, int bufSize, uint32_t ledPin)
{
    int len = 0;
	
    if (serialNum == EVB2_PORT_UINS1 && g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE)
    {
        len = spiTouINS_serRead(buf, bufSize);
    }
    else
    {
#if 0	// Wait for end of data.  Check Rx buffer has data and hasn't change over 1ms. 			
		static int usedLast[EVB2_PORT_COUNT] = {0};
		int used = serRxUsed(serialNum);
		if(used!=0 && used==usedLast[serialNum])
		{	// Data in Rx buffer and amount hasn't changed.
			len = serRead(serialNum, buf, bufSize, 0);
		}
		usedLast[serialNum] = used;		
#else
		// Normal read	
        len = serRead(serialNum, buf, bufSize);
#endif
    }
    
    if(len)
    {
        LED_ON(ledPin);
    }
    return len;
}

void callback_cdc_set_config(uint8_t port, usb_cdc_line_coding_t * cfg)
{
    UNUSED(port);
#if 0
    uint32_t stopbits, parity, databits;
//     uint32_t imr;

    switch (cfg->bCharFormat)
    {
    case CDC_STOP_BITS_2:
        stopbits = US_MR_NBSTOP_2_BIT;
        break;
    case CDC_STOP_BITS_1_5:
        stopbits = US_MR_NBSTOP_1_5_BIT;
        break;
    case CDC_STOP_BITS_1:
        default:
        // Default stop bit = 1 stop bit
        stopbits = US_MR_NBSTOP_1_BIT;
        break;
    }

    switch (cfg->bParityType)
    {
    case CDC_PAR_EVEN:
        parity = US_MR_PAR_EVEN;
        break;
    case CDC_PAR_ODD:
        parity = US_MR_PAR_ODD;
        break;
    case CDC_PAR_MARK:
        parity = US_MR_PAR_MARK;
        break;
    case CDC_PAR_SPACE:
        parity = US_MR_PAR_SPACE;
        break;
        default:
    case CDC_PAR_NONE:
        parity = US_MR_PAR_NO;
        break;
    }
    
    switch(cfg->bDataBits)
    {
    case 5:
    case 6:
    case 7:
        databits = cfg->bDataBits - 5;
        break;
        default:
    case 8:
        databits = US_MR_CHRL_8_BIT;
        break;
    }

    // Options for USART.  This gets called when USB is first connected AND each time the USB CDC serial port is opened by the host.
    //  sam_usart_opt_t usart_options;
    // 	usart_options.baudrate = LE32_TO_CPU(cfg->dwDTERate);
    // 	usart_options.char_length = databits;
    // 	usart_options.parity_type = parity;
    // 	usart_options.stop_bits = stopbits;
    // 	usart_options.channel_mode = US_MR_CHMODE_NORMAL;
#endif
	
    uint32_t baudrate = LE32_TO_CPU(cfg->dwDTERate);
    if (comManagerValidateBaudRate(baudrate)==0)
    {
        // Set baudrate based on USB CDC baudrate
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_UINS0))
        {
            serSetBaudRate(EVB2_PORT_UINS0, baudrate);
        }

        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_UINS1) &&
		  !(g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE))
        {
            serSetBaudRate(EVB2_PORT_UINS1, baudrate);
        }
    }

    if (comManagerValidateBaudRate(baudrate)==0 || baudrate==9600)
    {
        // 	    if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_XBEE))
        // 	    {
        // 		    serSetBaudRate(EVB2_PORT_XBEE, baudrate);
        // 	    }
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_XRADIO))
        {
            serSetBaudRate(EVB2_PORT_XRADIO, baudrate);
        }
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_BLE))
        {
            serSetBaudRate(EVB2_PORT_BLE, baudrate);
        }
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_SP330))
        {
            serSetBaudRate(EVB2_PORT_SP330, baudrate);
        }
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_GPIO_H8))
        {
            serSetBaudRate(EVB2_PORT_GPIO_H8, baudrate);
        }
    }

}

void callback_cdc_set_dtr(uint8_t port, bool b_enable)
{
    if (b_enable)
    {	// Host terminal has open COM
		d_usartDMA_callback_cdc_enable();
    }
    else
    {	// Host terminal has close COM
		d_usartDMA_callback_cdc_disable();
    }

    if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_XBEE))
    {
        if (b_enable)
        {	// Assert (LOW) DTR
            ioport_set_pin_level(UART_XBEE_N_DTR_PIN, IOPORT_PIN_LEVEL_LOW);      // Assert LOW
            ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_LOW);      // Assert LOW
        }
        else
        {	// De-assert (HIGH) DTR
            ioport_set_pin_level(UART_XBEE_N_DTR_PIN, IOPORT_PIN_LEVEL_HIGH);     // De-assert HIGH
            ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_HIGH);     // De-assert HIGH
        }
    }
}

// This function never gets called because of a bug in Atmel CDC driver.  Bummer.
// void callback_cdc_set_rts(uint8_t port, bool b_enable)
// {
// 	switch(g_msg.evb.comBridgeCfg)
// 	{
//     case EVB2_CBC_USBxXBEE:
//         if (b_enable)
//         {
//             ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_HIGH);
//         }
//         else
//         {
//             ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_LOW);
//         }
//         break;
//
//     case EVB2_CBC_RS232xUSB:
//     case EVB2_CBC_RS422xUSB:
//         break;
//     }
// }


void uINS_stream_stop_all(is_comm_instance_t &comm)
{
    int len = is_comm_stop_broadcasts_current_port(&comm);
    comWrite(g_flashCfg->uinsComPort, comm.buf.start, len, LED_INS_TXD_PIN);
}

void uINS_stream_enable_std(is_comm_instance_t &comm)
{
    int len;
    
    len = is_comm_get_data(&comm, DID_INS_2, 0, 0, 10);       // 20 x 4ms = 40ms
    comWrite(g_flashCfg->uinsComPort, comm.buf.start, len, LED_INS_TXD_PIN);

    len = is_comm_get_data(&comm, DID_DEV_INFO, 0, 0, 500);   // 500ms
    comWrite(g_flashCfg->uinsComPort, comm.buf.start, len, LED_INS_TXD_PIN);
}

void uINS_stream_enable_PPD(is_comm_instance_t &comm)
{
    rmc_t rmc;
    rmc.bits = RMC_PRESET_PPD_BITS;
    rmc.options = 0;
    int len = is_comm_set_data(&comm, DID_RMC, 0, sizeof(rmc_t), &rmc);
    comWrite(g_flashCfg->uinsComPort, comm.buf.start, len, LED_INS_TXD_PIN);

//     len = is_comm_get_data(&comm, DID_INS_2, 0, 0, 1);       // 1 x 4ms = 4ms
//     comWrite(EVB2_PORT_UINS0, comm.buffer, len, LED_INS_TXD_PIN);
}


void handle_data_from_uINS(p_data_hdr_t &dataHdr, uint8_t *data)
{
	uDatasets d;

	// Save uINS data to global variables
	switch(dataHdr.id)
	{
	case DID_DEV_INFO:
		copyDataPToStructP2(&g_msg.uInsInfo, &dataHdr, data, sizeof(dev_info_t));
		break;

	case DID_INS_1:
		copyDataPToStructP2(&d, &dataHdr, data, sizeof(ins_1_t));
		g_status.week = d.ins1.week;
		g_status.timeOfWeekMs = (uint32_t)(d.ins1.timeOfWeek*1000);
		break;
	                    
	case DID_INS_2:
		copyDataPToStructP2(&g_msg.ins2, &dataHdr, data, sizeof(ins_2_t));
		g_status.week = g_msg.ins2.week;
		g_status.timeOfWeekMs = (uint32_t)(g_msg.ins2.timeOfWeek*1000);
		break;

	case DID_INS_3:
		copyDataPToStructP2(&d, &dataHdr, data, sizeof(ins_3_t));
		g_status.week = d.ins1.week;
		g_status.timeOfWeekMs = (uint32_t)(d.ins3.timeOfWeek*1000);
		break;

	case DID_INS_4:
		copyDataPToStructP2(&d, &dataHdr, data, sizeof(ins_4_t));
		g_status.week = d.ins4.week;
		g_status.timeOfWeekMs = (uint32_t)(d.ins4.timeOfWeek*1000);
		break;
	}
}


void log_uINS_data(cISLogger &logger, is_comm_instance_t &comm)
{
	is_evb_log_stream stm = {};
	uint8_t data[MAX_DATASET_SIZE];
	p_data_hdr_t dataHdr = {};
	
    while (xStreamBufferReceive(g_xStreamBufferUINS, (void*)&stm, sizeof(is_evb_log_stream), 0) == sizeof(is_evb_log_stream))
    {	
		if (stm.marker==DATA_CHUNK_MARKER)
		{	
			switch (stm.ptype)
			{
			case _PTYPE_INERTIAL_SENSE_DATA:
				if (xStreamBufferReceive(g_xStreamBufferUINS, (void*)&dataHdr, sizeof(p_data_hdr_t), 0) == sizeof(p_data_hdr_t))
				{
					if (dataHdr.id < DID_COUNT && dataHdr.size < MAX_DATASET_SIZE)
					{
						if (xStreamBufferReceive(g_xStreamBufferUINS, (void*)data, dataHdr.size, 0) == dataHdr.size)
						{
							// Log Inertial Sense Binary data to SD Card
							if(g_loggerEnabled)
							{
								logger.LogData(0, &dataHdr, data);
							}
						}
					}	
				}
				break;
				
			case _PTYPE_ASCII_NMEA:
				break;

			case _PTYPE_UBLOX:
#if UBLOX_LOG_ENABLE
				if (xStreamBufferReceive(g_xStreamBufferUINS, (void*)&data, stm.size, 0) == stm.size)
				{
extern void log_ublox_raw_to_SD(cISLogger& logger, uint8_t *dataPtr, uint32_t dataSize);
					log_ublox_raw_to_SD(logger, data, stm.size);
				}
#endif
				break;
			}	
		}
	}
}


void update_flash_cfg(evb_flash_cfg_t &newCfg)
{
    if(error_check_config(&newCfg))
    {
        return;
    }
    
    // Data is identical, no changes.
	if( !memcmp(&newCfg, g_flashCfg, sizeof(evb_flash_cfg_t)) )
    {
		return;
    }        

    bool initCbPreset = false;
    bool initIOconfig = false;
    bool reinitXBee = false;
    bool reinitWiFi = false;

    // Detect changes
    if (newCfg.cbPreset != g_flashCfg->cbPreset ||
		newCfg.uinsComPort != g_flashCfg->uinsComPort)
    {
        initCbPreset = true;
        initIOconfig = true;
    }
    if (newCfg.radioPID != g_flashCfg->radioPID ||
        newCfg.radioNID != g_flashCfg->radioNID ||
        newCfg.radioPowerLevel != g_flashCfg->radioPowerLevel)
    {
        reinitXBee = true;
    }    
    if (EVB_CFG_BITS_IDX_WIFI(newCfg.bits) != EVB_CFG_BITS_IDX_WIFI(g_flashCfg->bits) ||
        EVB_CFG_BITS_IDX_SERVER(newCfg.bits) != EVB_CFG_BITS_IDX_SERVER(g_flashCfg->bits))
    {   // WiFi or TCP server preset changed
        reinitWiFi = true;
    }  
    int i = EVB_CFG_BITS_IDX_WIFI(newCfg.bits);        
    if (strncmp((const char*)(newCfg.wifi[i].ssid), (const char*)(g_flashCfg->wifi[i].ssid), WIFI_SSID_PSK_SIZE)!=0 ||
        strncmp((const char*)(newCfg.wifi[i].psk),  (const char*)(g_flashCfg->wifi[i].psk),  WIFI_SSID_PSK_SIZE)!=0 ||
        newCfg.server[i].ipAddr != g_flashCfg->server[i].ipAddr ||
        newCfg.server[i].port   != g_flashCfg->server[i].port)
    {   // WiFi or TCP settings changed
        reinitWiFi = true;            
    }
    i = EVB_CFG_BITS_IDX_SERVER(newCfg.bits);
    if (newCfg.server[i].ipAddr != g_flashCfg->server[i].ipAddr ||
        newCfg.server[i].port   != g_flashCfg->server[i].port)
    {   // TCP settings changed
        reinitWiFi = true;
    }
    if ((newCfg.cbOptions&EVB2_CB_OPTIONS_XBEE_ENABLE) != (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_XBEE_ENABLE) ||
        (newCfg.cbOptions&EVB2_CB_OPTIONS_WIFI_ENABLE) != (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_WIFI_ENABLE) ||
		(newCfg.h3sp330BaudRate != g_flashCfg->h3sp330BaudRate) ||
		(newCfg.h4xRadioBaudRate != g_flashCfg->h4xRadioBaudRate) ||
		(newCfg.h8gpioBaudRate != g_flashCfg->h8gpioBaudRate) )
    {
        initIOconfig = true;
    }
	if (g_flashCfg->CANbaud_kbps != newCfg.CANbaud_kbps)
	{
		g_flashCfg->CANbaud_kbps = newCfg.CANbaud_kbps;
		CAN_init();
	}
	if (g_flashCfg->can_receive_address != newCfg.can_receive_address)
	{
		g_flashCfg->can_receive_address = newCfg.can_receive_address;
		CAN_init();
	}    
    
    // Copy data from message to working location
    *g_flashCfg = newCfg;
    
    // Apply changes
    if(initCbPreset)
    {
        com_bridge_apply_preset(g_flashCfg);
    }
    if(initIOconfig)
    {	// Update EVB IO config
        board_IO_config();
    }
    if(reinitXBee)
    {
        xbee_init();
    }
    if(reinitWiFi)
    {
        wifi_reinit();
    }
    refresh_CFG_LED();
    
	// Enable flash write
	g_nvr_manage_config.flash_write_needed = true;
	g_nvr_manage_config.flash_write_enable = true;
}


void handle_data_from_host(is_comm_instance_t *comm, protocol_type_t ptype)
{
	uint8_t *dataPtr = comm->dataPtr + comm->dataHdr.offset;
	
	switch(ptype)
	{
	case _PTYPE_INERTIAL_SENSE_DATA:
		switch(comm->dataHdr.id)
		{	// From host to EVB
		case DID_EVB_STATUS:
			is_comm_copy_to_struct(&g_status, comm, sizeof(evb_status_t));
			break;
				
		case DID_EVB_FLASH_CFG:
			evb_flash_cfg_t newCfg;
			newCfg = *g_flashCfg;
			is_comm_copy_to_struct(&newCfg, comm, sizeof(evb_flash_cfg_t));
				
			update_flash_cfg(newCfg);				
			break;
				
		case DID_EVB_DEBUG_ARRAY:
			is_comm_copy_to_struct(&g_debug, comm, sizeof(debug_array_t));
			break;
		}

		// Disable uINS bootloader if host sends IS binary data
		g_uInsBootloaderEnableTimeMs = 0;
		break;

	case _PTYPE_INERTIAL_SENSE_CMD:
		if(comm->pkt.hdr.pid == PID_GET_DATA)
		{
			int n=0;
			is_comm_instance_t commTx;
			uint8_t buffer[MAX_DATASET_SIZE];
			is_comm_init(&commTx, buffer, sizeof(buffer));
			switch(comm->dataHdr.id)
			{
				case DID_DEV_INFO:          n = is_comm_data(&commTx, DID_EVB_DEV_INFO, 0, sizeof(dev_info_t), (void*)&(g_evbDevInfo));         break;
				case DID_EVB_STATUS:        n = is_comm_data(&commTx, DID_EVB_STATUS, 0, sizeof(evb_status_t), (void*)&(g_status));             break;
				case DID_EVB_FLASH_CFG:     n = is_comm_data(&commTx, DID_EVB_FLASH_CFG, 0, sizeof(evb_flash_cfg_t), (void*)(g_flashCfg));      break;
				case DID_EVB_DEBUG_ARRAY:   n = is_comm_data(&commTx, DID_EVB_DEBUG_ARRAY, 0, sizeof(debug_array_t), (void*)&(g_debug));        break;
				case DID_EVB_RTOS_INFO:     n = is_comm_data(&commTx, DID_EVB_RTOS_INFO, 0, sizeof(evb_rtos_info_t), (void*)&(g_rtos));         g_enRtosStats = true;	break;
			}
			if(n>0)
			{
				serWrite(EVB2_PORT_USB, commTx.buf.start, n);
			}			

			// Disable uINS bootloader mode if host sends IS binary command
			g_uInsBootloaderEnableTimeMs = 0;
		}
		break;

	case _PTYPE_ASCII_NMEA:
		{
			uint32_t messageIdUInt = ASCII_MESSAGEID_TO_UINT(&(dataPtr[1]));
			if(comm->dataHdr.size == 10)
			{	// 4 character commands (i.e. "$STPB*14\r\n")
				switch (messageIdUInt)
				{
				case ASCII_MSG_ID_BLEN: // Enable bootloader (uINS)
					g_uInsBootloaderEnableTimeMs = g_comm_time_ms;
					break;
							
				case ASCII_MSG_ID_EBLE: // Enable bootloader (EVB)
					// Disable uINS bootloader if host enables EVB bootloader
					g_uInsBootloaderEnableTimeMs = 0;
					
					enable_bootloader(PORT_SEL_USB);
					break;					
				}
				break;							
			}
			else
			{	// General ASCII							
				switch (messageIdUInt)
				{
				case ASCII_MSG_ID_NELB: // SAM bootloader assistant (SAM-BA) enable
					if (comm->dataHdr.size == 22 &&
// 									(pHandle == EVB2_PORT_USB) && 
						strncmp((const char*)(&(comm->buf.start[6])), "!!SAM-BA!!", 6) == 0)
					{	// 16 character commands (i.e. "$NELB,!!SAM-BA!!\0*58\r\n")
						enable_bootloader_assistant();
					}
					break;
					
				default:
					// Disable uINS bootloader if host sends larger ASCII sentence
					g_uInsBootloaderEnableTimeMs = 0;
					break;
				}				
			}
		}
		break;
	}

}


#define CAN_COM_PORT 1
#define CAN_HDR  0xFC;
#define CAN_FTR  0xFE;

void sendRadio(uint8_t *data, int dataSize, bool sendXbee, bool sendXrad)
{
	if(g_flashCfg->portOptions == EVB2_PORT_OPTIONS_RADIO_RTK_FILTER)
	{	// Filter RTK Base Messages
		protocol_type_t ptype;

		static is_comm_instance_t comm = {};
		static uint8_t buffer[PKT_BUF_SIZE];
		if (comm.buf.start == NULL)
		{	// Init buffer
			is_comm_init(&comm, buffer, sizeof(buffer));
		}

		for(uint8_t *ptr=data; dataSize>0; ) 
		{
			// Number of bytes to copy
			int n = _MIN(dataSize, is_comm_free(&comm));

			// Copy data into buffer
			memcpy(comm.buf.tail, ptr, n);
			comm.buf.tail += n;
			dataSize -= n;
			ptr += n;
		
			while((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
			{			
				// Parse Data
				switch(ptype)
				{
				case _PTYPE_UBLOX:
				case _PTYPE_RTCM3:
				case _PTYPE_ASCII_NMEA:
					if(sendXbee){ comWrite(EVB2_PORT_XBEE, comm.dataPtr, comm.dataHdr.size, LED_XBEE_TXD_PIN); }
					if(sendXrad){ comWrite(EVB2_PORT_XRADIO, comm.dataPtr, comm.dataHdr.size, 0); }
					break;
				}
			}
		}
	}
	else
	{
		if(sendXbee){ comWrite(EVB2_PORT_XBEE, data, dataSize, LED_XBEE_TXD_PIN); }
		if(sendXrad){ comWrite(EVB2_PORT_XRADIO, data, dataSize, 0); }		
	}		
}


// This function only forwards data after complete valid packets are received 
void com_bridge_smart_forward(uint32_t srcPort, uint32_t ledPin)
{	
	if(srcPort>=COM_RX_PORT_COUNT)
	{
		return;
	}
	
	is_comm_instance_t &comm = g_comRxPort[srcPort].comm;

	// Get available size of comm buffer
	int n = is_comm_free(&comm);

	if ((n = comRead(srcPort, comm.buf.tail, n, ledPin)) > 0)
	{
		if (g_uInsBootloaderEnableTimeMs)
		{	// When uINS bootloader is enabled forwarding is disabled below is_comm_parse(), so forward bootloader data here.
			switch (srcPort)
			{
				case EVB2_PORT_USB:		comWrite(EVB2_PORT_UINS0, comm.buf.tail, n, LED_INS_TXD_PIN);	break;
				case EVB2_PORT_UINS0:	comWrite(EVB2_PORT_USB, comm.buf.tail, n, 0);					break;
			}					
		}
		
		// Update comm buffer tail pointer
		comm.buf.tail += n;
				
		// Search comm buffer for valid packets
		protocol_type_t ptype;
		while((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
		{
			switch(srcPort)
			{
			case EVB2_PORT_USB:
			case EVB2_PORT_XBEE:
				handle_data_from_host(&comm, ptype);
				break;
			}			

			if (ptype!=_PTYPE_NONE && 
				ptype!=_PTYPE_PARSE_ERROR &&
				g_uInsBootloaderEnableTimeMs==0)
			{	// Forward data
				uint32_t pktSize = _MIN(comm.buf.scan - comm.pktPtr, PKT_BUF_SIZE);
				com_bridge_forward(srcPort, comm.pktPtr, pktSize);

				// Send uINS data to Logging task
				if (srcPort == g_flashCfg->uinsComPort)
				{
					is_evb_log_stream stm;
					stm.marker = DATA_CHUNK_MARKER;
					stm.ptype = ptype;
					switch (ptype)
					{
					case _PTYPE_INERTIAL_SENSE_DATA:
						handle_data_from_uINS(comm.dataHdr, comm.dataPtr);
					
						stm.size = sizeof(p_data_hdr_t) + comm.dataHdr.size;
						xStreamBufferSend(g_xStreamBufferUINS, (void*)&(stm), sizeof(is_evb_log_stream), 0);
						xStreamBufferSend(g_xStreamBufferUINS, (void*)&(comm.dataHdr), sizeof(p_data_hdr_t), 0);
						xStreamBufferSend(g_xStreamBufferUINS, (void*)(comm.dataPtr), comm.dataHdr.size, 0);
						break;
					case _PTYPE_UBLOX:
#if UBLOX_LOG_ENABLE
						stm.size = pktSize;
						xStreamBufferSend(g_xStreamBufferUINS, (void*)&(stm), sizeof(is_evb_log_stream), 0);
						xStreamBufferSend(g_xStreamBufferUINS, (void*)(comm.pktPtr), pktSize, 0);
#endif
						break;
					}
				}
			}
		}
	}	
}


void com_bridge_smart_forward_xstream(uint32_t srcPort, StreamBufferHandle_t xStreamBuffer);
void com_bridge_smart_forward_xstream(uint32_t srcPort, StreamBufferHandle_t xStreamBuffer)
{	
	if(srcPort>=COM_RX_PORT_COUNT)
	{
		return;
	}
	
	is_comm_instance_t &comm = g_comRxPort[srcPort].comm;

	// Get available size of comm buffer
	int n = is_comm_free(&comm);

	if ((n = xStreamBufferReceive(xStreamBuffer, comm.buf.tail, n, 0)) > 0)
	{
		// Update comm buffer tail pointer
		comm.buf.tail += n;
		
		// Search comm buffer for valid packets
		protocol_type_t ptype;
		while ((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
		{
			if (srcPort == EVB2_PORT_USB)
			{
				handle_data_from_host(&comm, ptype);
			}
			
			if (ptype!=_PTYPE_NONE && 
				ptype!=_PTYPE_PARSE_ERROR &&
				g_uInsBootloaderEnableTimeMs==0)
			{	// Forward data
				uint32_t pktSize = _MIN(comm.buf.scan - comm.pktPtr, PKT_BUF_SIZE);
				com_bridge_forward(srcPort, comm.pktPtr, pktSize);
			}
		}
	}
}


void com_bridge_forward(uint32_t srcPort, uint8_t *buf, int len)
{
    uint32_t dstCbf = g_flashCfg->cbf[srcPort];
    
    if(dstCbf == 0 || len==0)    // None
    {
        return;
    }        

	if (g_uInsBootloaderEnableTimeMs==0)
	{	// uINS bootloader mode enabled - don't allow forwarding on these ports
		if(dstCbf & (1<<EVB2_PORT_USB))
		{
			comWrite(EVB2_PORT_USB, buf, len, 0);
		}
    
		if(dstCbf & (1<<EVB2_PORT_UINS0))
		{
			comWrite(EVB2_PORT_UINS0, buf, len, LED_INS_TXD_PIN);
		}		
	}

    if(dstCbf & (1<<EVB2_PORT_UINS1))
    {
        comWrite(EVB2_PORT_UINS1, buf, len, LED_INS_TXD_PIN);
    }
	
	bool sendXbee   = dstCbf & (1<<EVB2_PORT_XBEE) && xbee_runtime_mode();	// Disable XBee communications when being configured
	bool sendXradio = dstCbf & (1<<EVB2_PORT_XRADIO);
	if(sendXbee || sendXradio)
	{
		sendRadio(buf, len, sendXbee, sendXradio);
	}

    if(dstCbf & (1<<EVB2_PORT_BLE))
    {
        comWrite(EVB2_PORT_BLE, buf, len, LED_WIFI_TXD_PIN);
    }

    if(dstCbf & (1<<EVB2_PORT_SP330))
    {
        comWrite(EVB2_PORT_SP330, buf, len, g_flashCfg->uinsComPort==EVB2_PORT_SP330?LED_INS_TXD_PIN:0);
    }

    if(dstCbf & (1<<EVB2_PORT_GPIO_H8))
    {
        comWrite(EVB2_PORT_GPIO_H8, buf, len, g_flashCfg->uinsComPort==EVB2_PORT_GPIO_H8?LED_INS_TXD_PIN:0);
    }

#if 0   // Disabled when forwarding data directly in wifi task
    if(dstCbf & (1<<EVB2_PORT_WIFI))
    {
        xStreamBufferSend(g_xStreamBufferWiFiTx, (void*)buf, len, 0);
    }    
#endif
}


void step_com_bridge(is_comm_instance_t &comm)
{	
	// USB CDC Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_USB, 0);

	// uINS Ser0 Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_UINS0, LED_INS_RXD_PIN);

	// uINS Ser1 (TTL or SPI) Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_UINS1, LED_INS_RXD_PIN);
    
#ifdef CONF_BOARD_SERIAL_XBEE           // XBee Rx   =======================================================
    xbee_step(&comm);
#endif

#ifdef CONF_BOARD_SERIAL_EXT_RADIO      // External Radio Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_XRADIO, 0);
#endif

#ifdef CONF_BOARD_SERIAL_ATWINC_BLE     // ATWINC BLE Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_BLE, LED_WIFI_RXD_PIN);
#endif

#ifdef CONF_BOARD_SERIAL_SP330          // SP330 RS232/RS422 converter Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_SP330, g_flashCfg->uinsComPort==EVB2_PORT_SP330?LED_INS_RXD_PIN:0);
#endif

#ifdef CONF_BOARD_SERIAL_GPIO_H8        // H8 Header GPIO TTL Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_GPIO_H8, g_flashCfg->uinsComPort==EVB2_PORT_GPIO_H8?LED_INS_RXD_PIN:0);
#endif

#ifdef CONF_BOARD_SPI_ATWINC_WIFI       // WiFi Rx   =======================================================
	com_bridge_smart_forward_xstream(EVB2_PORT_WIFI, g_xStreamBufferWiFiRx);
#endif

#ifdef CONF_BOARD_CAN1
	uint8_t can_rx_message[CONF_MCAN_ELEMENT_DATA_SIZE];
	uint32_t id;
	uint8_t lenCAN;
	is_can_message msg;
	msg.startByte = CAN_HDR;
	msg.endByte = CAN_FTR;
	if((lenCAN = mcan_receive_message(&id, can_rx_message)) > 0)// && --timeout > 0))
	{
		msg.address = id;
		msg.payload = *(is_can_payload*)can_rx_message;
		msg.dlc = lenCAN;
		com_bridge_forward(EVB2_PORT_CAN,(uint8_t*)&msg, sizeof(is_can_message));
	}
	/*Test CAN*/
		//static uint8_t tx_message_1[8] = { 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87};
		//mcan_send_message(0x100000A5, tx_message_1, CONF_MCAN_ELEMENT_DATA_SIZE);
#endif	
}


void communications_init(void)
{
    const size_t xTriggerLevel = 1;
    g_xStreamBufferUINS = xStreamBufferCreate( STREAM_BUFFER_SIZE, xTriggerLevel );
    g_xStreamBufferWiFiRx = xStreamBufferCreate( STREAM_BUFFER_SIZE, xTriggerLevel );
    g_xStreamBufferWiFiTx = xStreamBufferCreate( STREAM_BUFFER_SIZE, xTriggerLevel );

	for(int i=0; i<COM_RX_PORT_COUNT; i++)
	{
		is_comm_init(&(g_comRxPort[i].comm), g_comRxPort[i].comm_buffer, PKT_BUF_SIZE);

		// Use alternate decode buffer on EVB so we can preserve and forward original packet received.
		g_comRxPort[i].comm.altDecodeBuf = s_rxDecodeBuffer;
	}
}
