/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include <string>
#include "conf_board.h"
#include "drivers/d_usartDMA.h"
#include "globals.h"
#include "../../../src/ISComm.h"
#include "xbee.h"

using namespace std;


enum xbee_frame_id
{
    AT_COMMAND_FRAME					=	0x08,		// AT Command frame
    AT_COMMAND_QUEUE_FRAME				=	0x09,		// AT Command - Queue Parameter Value frame
    TX_REQUEST_FRAME					=	0x10,		// Transmit Request frame
    TX_EXPLICIT_REQUEST_FRAME			=	0x11,		// Explicit Addressing Command frame
    AT_REMOTE_COMMAND_FRAME				=	0x17,		// Remote AT Command Request frame
    AT_SECURE_REMOTE_COMMAND_FRAME		=	0x18,		// Secure Remote AT Command Request frame
    AT_COMMAND_RESPONSE_FRAME			=	0x88,		// AT Command Response frame
    RX_INDICATOR_FRAME					=	0x90,		// RX Indicator frame
    RX_EXPLICIT_INDICATOR_FRAME			=	0x91,		// Explicit Rx Indicator frame
    NODE_ID_FRAME						=	0x95,		// Node Identification Indicator frame
    REMOTE_COMMAND_RESPONSE_FRAME		=	0x97,		// Remote Command Response frame
};

/** Special Commands */
#define  XBEE_APPLY_CHANGES 		"AC"
#define  XBEE_SOFTWARE_RESET 		"FR"
#define  XBEE_RESTORE_DEFAULTS 		"RE"
#define  XBEE_WRITE_VALUES 			"WR"
#define  XBEE_EXIT_COMMAND_MODE		"CN"

/** Settable parameters. Only list the ones that are different from the stock config. */
#define  XBEE_PREAMBLE_ID 			"HP"
#define  XBEE_NETWORK_ID			"ID"
#define  XBEE_BCAST_MULTITX 		"MT"
#define  XBEE_UCAST_RETRIES 		"RR"
#define  XBEE_TX_OPTIONS			"TO"
#define  XBEE_TX_POWER_LEVEL		"PL"
#define  XBEE_BAUD_RATE 			"BD"
#define  XBEE_PACKETIZATION_TO 		"RO"
#define  XBEE_API_ENABLE			"AP"
#define  XBEE_SLEEP_MODE			"SM"

/** Readable parameters. Only list the ones you want to read */
#define  XBEE_RSSI =				"DB"


/** Settable parameters. Only list the ones that are different from the stock config. */
struct xbee_diff_commands
{
    uint8_t     preamble_id         = 23;
    uint16_t    network_id          = 45;
    uint8_t     bcast_multi_tx      = 0;
    uint8_t     ucast_retries       = 0;
    uint8_t     tx_options          = 0x40;     // Point to point/multipoint
    uint8_t     tx_power_level      = 2;        // 2 = 30dBm (full power)
    uint8_t     baud_rate           = 7;        // 7 = 115,200
    uint8_t     packetization_to    = 0;        
    uint8_t     api_enable          = 1;             
    uint8_t     sleep_options       = 1;        // Use I/O pin to sleep
};

/** Readable parameters. Only list the ones you want to read */
struct xbee_read_commands
{
    uint8_t rssi;
};


enum
{
    XSTATE_RUN_TIME_DEFAULT = 0,
    XSTATE_START_CONFIG,
    XSTATE_AT_CMD_MODE_START_1,
    XSTATE_AT_CMD_MODE_START_2,
    XSTATE_AT_CMD_MODE_CONFIG,
    XSTATE_AT_CMD_MODE_FAILURE,
    XSTATE_AT_CMD_MODE_FAILURE_DELAY,
    XSTATE_AT_CMD_MODE_FINISH,
    XSTATE_AT_CMD_MODE_FINISH_DELAY,
};

// #define BUF_SIZE 256

xbee_diff_commands xbee_diff;
xbee_read_commands xbee_read;


// uint8_t xbee_buf[BUF_SIZE];
// static uint8_t s_last_api_command[3] = {0};
static int s_at_command_idx = 0;

static bool s_at_ok = 0;
static int s_timer_ms = 0;
static int s_baud_rate = 115200;
static int s_xstate = XSTATE_RUN_TIME_DEFAULT;



static void send_at_command(string cmd)
{
    cmd = string("AT") + cmd + "\r";    
	comWrite(EVB2_PORT_XBEE, (uint8_t *)(cmd.c_str()), cmd.size(), LED_XBEE_TXD_PIN);    
}


static bool send_next_at_command() 
{
    switch(s_at_command_idx++)
    {
    case 0:     send_at_command( XBEE_RESTORE_DEFAULTS );  break;
    case 1:     send_at_command( XBEE_PREAMBLE_ID + to_string(g_flashCfg->radioPID) );  break;
    case 2:     send_at_command( XBEE_NETWORK_ID + to_string(g_flashCfg->radioNID) );  break;
    case 3:     send_at_command( XBEE_TX_OPTIONS + string("0x40") );  break;
    case 4:     send_at_command( XBEE_TX_POWER_LEVEL + to_string(g_flashCfg->radioPowerLevel) );  break;
	case 5:		send_at_command( XBEE_BCAST_MULTITX + string("0") );   break;                // sets multi-transmit to off
	case 6:		send_at_command( XBEE_UCAST_RETRIES + string("0") );   break;                // sets retry to off
    case 7:     send_at_command( XBEE_BAUD_RATE + to_string(xbee_diff.baud_rate) );  break;     // baud rate doesn't change until XBEE_EXIT_COMMAND_MODE is sent
    case 8:     
        if(s_baud_rate!=115200)
        {   // Update flash IF baud is not at 115200 so next time it starts faster
            send_at_command( XBEE_WRITE_VALUES ); 
            break;
        }
        else
        {   // Continue to next command
            s_at_command_idx++;
        }    
    // This must be the last command sent
    case 9:     send_at_command( XBEE_EXIT_COMMAND_MODE );  break;
    
//     case 3:     send_at_command( XBEE_BCAST_MULTITX + to_string(xbee_diff.bcast_multitx_val) );  break;
//     case 4:     send_at_command( XBEE_UCAST_RETRIES + to_string(xbee_diff.ucast_retries_val) );  break;
//     case 6:     send_at_command( XBEE_PACKETIZATION_TO + to_string(xbee_diff.packetization_to) );  break;
//     case 7:     send_at_command( XBEE_API_ENABLE + to_string(xbee_diff.api_enable) );  break;
//     case 8:     send_at_command( XBEE_SLEEP_MODE + to_string(xbee_diff.sleep_options) );  break;
//     case 10:    send_at_command( XBEE_WRITE_VALUES );  break;
    default:    return true;    // done
    }
    
    // Not done
    return false;
}


// uint8_t xbee_packetize(uint8_t *raw_buf, uint8_t len_raw_buf, uint8_t *pack_buf) {
// 	raw_buf[0] = 0x7E;				// Start delimiter
// 	raw_buf[1] = 0x00;				// MSB
// 	raw_buf[2] = len_raw_buf;		// LSB
// 	
// 	uint32_t checksum;
// 	
// 	pack_buf = raw_buf - 3;
// 	
// 	for(size_t i = 0 ; i < len_raw_buf ; ++i ) {
// 		checksum += raw_buf[i];
// 	}
// 	
// 	pack_buf[len_raw_buf+3] = 0xFF - (checksum & 0xFF);	// Checksum calculation
// 	
// 	return len_raw_buf+3;	// Length of packetized buffer
// }

static void xbee_receive(is_comm_instance_t *comm) 
{
    if( xbee_runtime_mode()  
//         || g_flashCfg->cbPreset == EVB2_CB_PRESET_USB_HUB_RS232
//         || g_flashCfg->cbPreset == EVB2_CB_PRESET_USB_HUB_RS422
    )
    {   // Normal communications
		com_bridge_smart_forward(EVB2_PORT_XBEE, LED_XBEE_RXD_PIN);
	}
    else 
    {   // XBee configuration/command mode
#define BUF_SIZE 256    //USB CDC buffer size is 320
		uint8_t buf[BUF_SIZE];
		int len;
		if((len = comRead(EVB2_PORT_XBEE, buf, BUF_SIZE, LED_XBEE_RXD_PIN)) <= 0)
		{   // No data
			return;
		}

        // Look for AT "OK\r" response
        while(buf[0] == 0x4F && len < 3)
        {   // Ensure we have at lease 3 bytes
            len += comRead(EVB2_PORT_XBEE, &buf[len], BUF_SIZE-len, LED_XBEE_RXD_PIN); //Read new com data into buffer after original data and update len value
        }
        if(memcmp(reinterpret_cast<char *>(buf), "OK\r", 3) == 0)
        {   // AT command mode
            s_at_ok = 1;											// Make sure to reset this when you go to the next command
        }
		
#if 0       
        if(buf[0] == 0x7E)
        {	// API packet
		    uint8_t lsb = buf[2];
		
		    switch (buf[3])
		    {
			case AT_COMMAND_RESPONSE_FRAME:
				s_last_api_command[0] = buf[5];		// First AT character
				s_last_api_command[1] = buf[6];		// Second AT character
				s_last_api_command[2] = buf[7];		// Command status
				
				if(buf[5] == ('D') && buf[6] == ('B')) 
                {
					xbee_read.rssi = buf[8];
				}
				break;
                
			case RX_INDICATOR_FRAME:
				com_bridge_forward(EVB2_PORT_XBEE, buf + 15, lsb - 12);		// Forward data into com bridge
				break;
		    }
	    }
        else
#endif  
    }        
}


void xbee_init(void)
{
    s_baud_rate = 115200;
    g_status.evbStatus &= (~EVB_STATUS_XBEE_CONFIGURED | ~EVB_STATUS_XBEE_CONFIG_FAILURE);
    s_xstate = XSTATE_START_CONFIG;
    LED_OFF(LED_XBEE_RXD_PIN);
    LED_OFF(LED_XBEE_TXD_PIN);
}


void xbee_step(is_comm_instance_t *comm)
{
    if( !(g_status.evbStatus & EVB_STATUS_XBEE_ENABLED) )
    {   // Don't run XBee if its disabled.  Enabled inside board_IO_config().
        return;
    }
    
    switch(s_xstate)
    {
    default:    // XSTATE_RUN_TIME_DEFAULT
        LED_OFF(LED_XBEE_RXD_PIN);
        LED_OFF(LED_XBEE_TXD_PIN);
        break;
    
    case XSTATE_START_CONFIG:
    	serSetBaudRate(EVB2_PORT_XBEE, s_baud_rate);
        LED_OFF(LED_XBEE_RXD_PIN);  // green off
        LED_OFF(LED_XBEE_TXD_PIN);  // red off
        s_timer_ms = g_comm_time_ms;
        s_xstate++;
        break;
    
    case XSTATE_AT_CMD_MODE_START_1:
        if((g_comm_time_ms - s_timer_ms) < 1500)
        {   
            LED_OFF(LED_XBEE_RXD_PIN);  // green off
            LED_OFF(LED_XBEE_TXD_PIN);  // red off
            break;
        }            
            
        comWrite(EVB2_PORT_XBEE, (uint8_t *)"+++", 3, LED_XBEE_TXD_PIN);
        LED_ON(LED_XBEE_RXD_PIN);   // green on
        LED_ON(LED_XBEE_TXD_PIN);   // red on   (orange) 
        s_timer_ms = g_comm_time_ms;
        s_xstate++;
        break;
                
    case XSTATE_AT_CMD_MODE_START_2:
        if((g_comm_time_ms - s_timer_ms) < 1200)
        {   
            break;  // Wait
        }
        
        if(s_at_ok)
        {   // Received OK
            s_at_command_idx = 0;
            s_timer_ms = g_comm_time_ms;
            s_xstate = XSTATE_AT_CMD_MODE_CONFIG;
            break;
        }        
        if(s_baud_rate == 115200)
        {   // Retry config at 9600 baud
            s_baud_rate = 9600;
            s_xstate = XSTATE_START_CONFIG;
            break;
        }     
        s_xstate = XSTATE_AT_CMD_MODE_FAILURE;   
        break;

    case XSTATE_AT_CMD_MODE_CONFIG:
        if(s_at_ok)
        {   // Last command OK.  Send next.
            s_at_ok = 0;
            s_timer_ms = g_comm_time_ms;
            if(send_next_at_command())
            {
                s_xstate = XSTATE_AT_CMD_MODE_FINISH;
            }
        }
        else
        {
            if((g_comm_time_ms - s_timer_ms) > 1000)
            {   // Timeout failure
                s_xstate = XSTATE_AT_CMD_MODE_FAILURE;
            }            
        }        
        break;

    case XSTATE_AT_CMD_MODE_FAILURE:
        // Failure to enter AT command mode
        serSetBaudRate(EVB2_PORT_XBEE, 115200);
        g_status.evbStatus |= EVB_STATUS_XBEE_CONFIG_FAILURE;
        s_timer_ms = g_comm_time_ms;
        s_xstate = XSTATE_AT_CMD_MODE_FAILURE_DELAY;
        break;

    case XSTATE_AT_CMD_MODE_FAILURE_DELAY:
        if((g_comm_time_ms - s_timer_ms) > 1000)
        {
            s_xstate = XSTATE_RUN_TIME_DEFAULT;
        }
        LED_OFF(LED_XBEE_RXD_PIN);      // green off
        LED_ON(LED_XBEE_TXD_PIN);       // red on
        break;    

    case XSTATE_AT_CMD_MODE_FINISH:
        // Completed configuration!
        serSetBaudRate(EVB2_PORT_XBEE, 115200);
        g_status.evbStatus |= EVB_STATUS_XBEE_CONFIGURED;
        s_timer_ms = g_comm_time_ms;
        s_xstate = XSTATE_AT_CMD_MODE_FINISH_DELAY;
        break;
        
    case XSTATE_AT_CMD_MODE_FINISH_DELAY:
        if((g_comm_time_ms - s_timer_ms) > 1000)
        {
            s_xstate = XSTATE_RUN_TIME_DEFAULT;
        }
        LED_ON(LED_XBEE_RXD_PIN);       // green on
        LED_OFF(LED_XBEE_TXD_PIN);      // red off
        break;    
    }
    
    xbee_receive(comm);
}



int xbee_runtime_mode(void)
{
    return s_xstate==XSTATE_RUN_TIME_DEFAULT;
}


