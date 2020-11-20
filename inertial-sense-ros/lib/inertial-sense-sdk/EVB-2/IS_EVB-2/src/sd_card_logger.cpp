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
#include "globals.h"
#include "ISLogFileFatFs.h"
#include "communications.h"
#include "sd_mmc_mem.h"

#include "sd_card_logger.h"

#undef printf
#define printf(...)
#define printf_mutex(...)

int                         g_indicateFsWriteMs=0;

static FATFS                s_fs;
static bool                 s_sd_card_ready=false;
static int                  s_enableLogger=0;       // 1=enable, -1=disable

#if UBLOX_LOG_ENABLE
static cISLogFileBase       *s_ubloxLog = NULLPTR;
#endif


void update_led_log(void)
{
    if(s_sd_card_ready)
    {   // SD ready

        if(g_loggerEnabled)
        {   // Logging
            if(g_indicateFsWriteMs > 0)
            {   // writing to disk
                g_indicateFsWriteMs -= g_rtos.task[EVB_TASK_LOGGER].periodMs;
                LED_LOG_YELLOW();
//                 ioport_set_pin_level(SKI_BOX_STATUS_LED_PIN,IOPORT_PIN_LEVEL_HIGH);     // LED ON
            }
            else
            {   // not writing to disk
                LED_LOG_OFF();
//                 ioport_set_pin_level(SKI_BOX_STATUS_LED_PIN,IOPORT_PIN_LEVEL_LOW);     // LED OFF
            }
        }
        else
        {   // Not logging
            if(g_gpsTimeSync)
            {   // w/ GPS
                LED_LOG_GREEN();
            }
            else
            {   // No GPS
                LED_LOG_BLUE();
            }
        }
    }        
    else
    {   // SD not ready
        LED_LOG_OFF();
    }
}


static void start_logger(cISLogger& logger, is_comm_instance_t &comm)
{
    if( s_sd_card_ready==false )
    {
        return;
    }
        
    g_loggerEnabled = true;
    g_status.evbStatus |= EVB_STATUS_SD_LOG_ENABLED;
    update_led_log();

//     uINS0_stream_stop_all(comm);
    uINS_stream_enable_PPD(comm);

//     logger.InitSave(LOGTYPE_DAT, cISLogger::g_emptyString, 1, 0.5f, 1024 * 1024 * 5, 131072);
    logger.InitSave(cISLogger::LOGTYPE_DAT, "IS_logs", 1, 0.5f, 1024 * 1024 * 5, 16384);
//     logger.InitSave();
    logger.SetDeviceInfo(&g_msg.uInsInfo);   // set uINS serial number 
    logger.EnableLogging(true);
}


static void stop_logger(cISLogger& logger, is_comm_instance_t &comm)
{
    g_loggerEnabled = false;
    g_status.evbStatus &= ~EVB_STATUS_SD_LOG_ENABLED;
    update_led_log();

//     uINS0_stream_stop_all(comm);
//     uINS0_stream_enable_std(comm);

    logger.EnableLogging(false);
    logger.CloseAllFiles();
	
#if UBLOX_LOG_ENABLE
	if (s_ubloxLog != NULLPTR)
	{
		delete s_ubloxLog;
		s_ubloxLog = NULLPTR;
	}
#endif
}


void enable_logger(bool enable)
{
    if(enable)
    {
        s_enableLogger = 1;
    }
    else
    {   // Disable logger
        s_enableLogger = -1;
    }
}


void step_logger_control(cISLogger& logger, is_comm_instance_t &comm)
{
    switch(s_enableLogger)
    {
    case 1:     // Enable logger
        s_enableLogger = 0;
        start_logger(logger, comm);
        break;
    case -1:    // Disable logger
        s_enableLogger = 0;
        stop_logger(logger, comm);
    }    
}



void log_ublox_raw_to_SD(cISLogger& logger, uint8_t *dataPtr, uint32_t dataSize)
{
#if UBLOX_LOG_ENABLE

    if(g_loggerEnabled)
    {	// log raw u-blox, rtcm, etc.
        if (s_ubloxLog == NULLPTR)
        {
            std::string ubloxFile = (logger.LogDirectory() + "/raw_ublox.ubx");
            s_ubloxLog = new cISLogFileFatFs(ubloxFile.c_str(), "wb");
        }
		if (dataSize <= MAX_DATASET_SIZE)
		{
	        s_ubloxLog->write(dataPtr, dataSize);
		}
    }

#endif
}


void time_sync_from_uINS(void)
{
    static uint32_t timeOfWeekMsLast=0;
    
    // Real-time clock (RTC) time synchronization with uINS
    if(g_status.week != 0 && g_status.timeOfWeekMs!=timeOfWeekMsLast)
    {
        timeOfWeekMsLast = g_status.timeOfWeekMs;
        int32_t gpsSeconds = g_status.timeOfWeekMs/1000;
        convertMjdToDate( convertGpsToMjd(g_msg.ins2.week, gpsSeconds), (int32_t*)&g_gps_date_time.year, (int32_t*)&g_gps_date_time.month, (int32_t*)&g_gps_date_time.day);
        convertGpsToHMS( gpsSeconds, (int32_t*)&g_gps_date_time.hour, (int32_t*)&g_gps_date_time.minute, (int32_t*)&g_gps_date_time.second );

#if USE_RTC_DATE_TIME   // use RTC
        date_time_t rtc;
        rtc_get_date(RTC, &rtc.year, &rtc.month, &rtc.day, &rtc.week);
        rtc_get_time(RTC, &rtc.hour, &rtc.minute, &rtc.second);
    
        if( g_gps_date_time.year != rtc.year ||
            g_gps_date_time.month != rtc.month ||
            g_gps_date_time.day != rtc.day ||
            g_gps_date_time.hour != rtc.hour ||
            g_gps_date_time.minute != rtc.minute )
        {            
            g_gps_date_time.week = dateToWeekDay(g_gps_date_time.year, g_gps_date_time.month, g_gps_date_time.day);            

            rtc_set_date(RTC, g_gps_date_time.year, g_gps_date_time.month, g_gps_date_time.day, g_gps_date_time.week);      // year, month, day, day of week
            rtc_set_time(RTC, g_gps_date_time.hour, g_gps_date_time.minute, g_gps_date_time.second);                        // hour, minute, second
        }
        else
#endif
        {
            g_gpsTimeSync = true;
        }
    }
}


void sd_card_maintenance(void)
{
    static Ctrl_status statusLast = CTRL_NO_PRESENT;
    Ctrl_status status = sd_mmc_test_unit_ready(0);
//     FRESULT res;
        
    if( status != statusLast )
    {   // Event
        statusLast = status;
              
        switch(status)
        {        
        case CTRL_GOOD:         // Card inserted and ready
            s_sd_card_ready = true;
            g_status.evbStatus |= EVB_STATUS_SD_CARD_READY;
            break;

        case CTRL_NO_PRESENT:   // Card removed
            s_sd_card_ready = false;
            g_status.evbStatus &= ~EVB_STATUS_SD_CARD_READY;
            printf_mutex("Card removed\r\n");
            break;
        }
    }
}


void sd_card_logger_init(void)
{
    /* Initialize SD MMC stack */
    sd_mmc_init();

    /* Initialize FatFs handle: g_fs */
    memset(&s_fs, 0, sizeof(FATFS));
    f_mount(LUN_ID_SD_MMC_0_MEM, &s_fs);

    /* Init real-time clock: RTC configuration, 24-hour mode */
    rtc_set_hour_mode(RTC, 0);

    /* Init real-time timer */
    time_init();    

#if USE_RTC_DATE_TIME
	/* Init real-time clock */    
	if((RTC->RTC_CR & (RTC_CR_UPDTIM | RTC_CR_UPDCAL)) != 0)	//Make sure RTC is running
		RTC->RTC_CR &= ~(RTC_CR_UPDTIM | RTC_CR_UPDCAL);
	
	rtc_set_time(RTC, 1, 1, 1);                                 // hour, minute, second
	rtc_set_date(RTC, 2018, 1, 1, dateToWeekDay(2018, 1, 1));   // year, month, day, week
#endif
}