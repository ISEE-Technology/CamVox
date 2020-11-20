/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __GLOBALS_H_
#define __GLOBALS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <asf.h>
#include "../../../src/ISComm.h"
#include "../../../src/data_sets.h"
#include "../../../src/ISConstants.h"
#include "../../../hw-libs/misc/bootloaderShared.h"
#include "../../../hw-libs/misc/rtos.h"
#include "../../../hw-libs/drivers/d_usartDMA.h"
#include "drivers/d_time.h"
#include "conf_board.h"

#define USE_RTC_DATE_TIME       1   // Use RTC for system data and time

#define STREAM_INS_FOR_TIME_SYNC        1
#define SKI_BOX_STATUS_LED_PIN          GPIO_10_PIN
#define UBLOX_LOG_ENABLE			    0

typedef struct
{
    uint32_t year, month, day, week, hour, minute, second;
} date_time_t;

typedef struct
{
    dev_info_t      uInsInfo;
    ins_2_t         ins2;
}evb_msg_t;

typedef struct PACKED      // Non-volatile memory state
{
    uint32_t                flash_write_needed;					// 0=No write; 1=config write needed; 2=config write needed without backup 0xFFFFFFFF=reset defaults
    uint32_t                flash_write_count;                  // Number of times flash is written to since reset
    uint32_t                flash_write_enable;				    // 1 = enables flash writes.  This is used to prevent stutters in RTOS caused by flash writes until controlled times.
} nvr_manage_t;

PUSH_PACK_1

// All Flash Parameters - config max size is 8K for ARM
typedef struct PACKED
{
    union
    {
	    evb_flash_cfg_t m;
	    uint32_t padding[BOOTLOADER_FLASH_BLOCK_SIZE / sizeof(uint32_t)];  // 8 Kb
// 	    uint32_t padding[IFLASH_PAGE_SIZE / sizeof(uint32_t)];  // 512 bytes
    } g0;
} nvm_config_t;

POP_PACK


#define STREAM_BUFFER_SIZE      4096

extern uint8_t                      g_hdw_detect;
extern dev_info_t                   g_evbDevInfo;
extern wheel_encoder_t              g_wheelEncoder;
extern evb_status_t                 g_status;
extern evb_flash_cfg_t*             g_flashCfg;
extern nvr_manage_t                 g_nvr_manage_config;
extern nvm_config_t                 g_userPage;
extern evb_msg_t                    g_msg;
extern debug_array_t                g_debug;
extern evb_rtos_info_t              g_rtos;
extern date_time_t                  g_gps_date_time;
//extern uint32_t					g_CANbaud_kbps;
//extern uint32_t					g_can_receive_address;
extern bool                         g_gpsTimeSync;
extern uint32_t                     g_comm_time_ms;
extern bool                         g_loggerEnabled;
extern uint32_t                     g_uInsBootloaderEnableTimeMs;
extern bool                         g_enRtosStats;


void globals_init(void);
void com_bridge_apply_preset(evb_flash_cfg_t* cfg);
void reset_config_defaults(evb_flash_cfg_t* cfg);
int comWrite(int serialNum, const unsigned char *buf, int size, uint32_t ledPin );
int comRead(int serialNum, unsigned char *buf, int size, uint32_t ledPin);
void com_bridge_forward(uint32_t srcPort, uint8_t *buf, int len);
void com_bridge_smart_forward(uint32_t srcPort, uint32_t ledPin);

void nvr_init(void);
void nvr_slow_maintenance(void);

int error_check_config(evb_flash_cfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif // __GLOBALS_H_
