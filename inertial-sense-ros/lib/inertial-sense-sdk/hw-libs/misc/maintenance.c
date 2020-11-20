#include <asf.h>
#include "misc/rtos.h"
#include "maintenance.h"
#include "d_flash.h"
#include "bootloaderApp.h"
#include "globals.h"



void led_maintenance(void)
{
	if( g_nvmFlashCfg->sysCfgBits & SYS_CFG_BITS_DISABLE_LEDS )
	{
		LEDS_ALL_OFF();
		return;
	}
	
	int solStatus = INS_STATUS_SOLUTION(g_insStatus);
	
#if LED_SCHEME==LED_SCHEME_BLINK
	//////////////////////////////////////////////////////////////////////////
	//  System Slow Blink Flash

	// Red LED
	switch( g_sc.calState )
	{
		// Calibration in Progress
	default:
		LED_ON(LED_RED);
		break;
			
		// GPS Status
	case SC_RUNTIME:
	case SC_TCAL_DONE:
		// end LED Flash
		if( g_gpsTimeOfWeekMs >= g_led.gps_timeMs )
			LED_OFF(LED_GPS_STATUS);
		break;
	}

//         if( g_led.heartbeat_cnt > 50 )	// incremented in solution task
// 		{
//             g_led.heartbeat_cnt = 0;
//             LED_TOGGLE(LED_SYS_HEALTH);
//         }            
#elif LED_SCHEME==LED_SCHEME_HEARTBEAT
	//////////////////////////////////////////////////////////////////////////
	//  System Heart Beat Flash
	vTaskDelay( mainLED_FLASH_RATE );        
	// End LED Flash
	if( g_gpsTimeOfWeekMs >= g_led.gps_timeMs )
		LED_OFF(LED_GPS_STATUS);       

	// System LED Heartbeat           
#define HEARTBEAT_RAMP_RATE     (500/mainLED_FLASH_RATE) 
#define HEARTBEAT_WAIT          (100/mainLED_FLASH_RATE)
	switch(g_led.heartbeat_state)
	{
	default:    // ramp up
		g_led.heartbeat_duty += HEARTBEAT_RAMP_RATE;
            
		if( g_led.heartbeat_duty >= 100 )
		{
			g_led.heartbeat_duty    = 100;
			g_led.heartbeat_cnt     = 0;
			g_led.heartbeat_state   = 1;
		}
		set_LED_PWM(g_led.heartbeat_duty);
		break;
                
	case 1:     // Wait
		if( ++g_led.heartbeat_cnt >= HEARTBEAT_WAIT )
			g_led.heartbeat_state++;
		break;
                
	case 2:     // Ramp down
		g_led.heartbeat_duty -= HEARTBEAT_RAMP_RATE;
                        
		if( g_led.heartbeat_duty <= 0 )
		{
			g_led.heartbeat_duty    = 0;
			g_led.heartbeat_cnt     = 0;
			g_led.heartbeat_state++;
		}
		set_LED_PWM(g_led.heartbeat_duty);
		break;
                
	case 3:     // Wait
		if( ++g_led.heartbeat_cnt >= HEARTBEAT_WAIT )
			g_led.heartbeat_state++;
		break;
	}
		
#elif LED_SCHEME==LED_SCHEME_SOLID
	// Solid LED Scheme:
	// Red - general fault, unhanded interrupt, calibrating
	// Orange - unaligned
	// Blue - aligned w/o GPS lock
	// Green - aligned w GPS lock and sync (blip Red)
	// White Purple - On boot
	// Purple - boot loader
	// Cyan (green/blue) -

	if( g_sc.calState != SC_RUNTIME )
	{
		switch(g_sc.calState)
		{
		default: 
		case SC_ORTH_SAMPLE_INIT:
			LED_COLOR_RED();
			break;
		
		case SC_ACCEL_ALIGN_CHECK:
			if( g_sc.status == SC_STATUS_ALIGNED )
			{	
				LED_COLOR_GREEN();
			}
			else
			{
				LED_COLOR_YELLOW();
			}
			break;

		case SC_ORTH_SAMPLE_MEAN:				
			LED_COLOR_CYAN();
			break;
			
		case SC_DONE:
// 			LED_COLOR_WHITE();
			LED_COLOR_RED();			// Red
			break;
		}
	}
	else
	{
		// Allow for GPS red pulse
		if( g_gpsTimeOfWeekMs < g_led.gps_timeMs )
			return;

		// Mag is recalibrating
		if( g_insStatus & INS_STATUS_MAG_RECALIBRATING )
		{
			LED_COLOR_PURPLE();
			return;
		}
		
		// Normal Operation
		switch( solStatus )
		{
		case INS_STATUS_SOLUTION_OFF:
			//LEDS_ALL_OFF();
			LED_COLOR_YELLOW();
			break;
			
		case INS_STATUS_SOLUTION_ALIGNING:
			LED_COLOR_WHITE();
			break;
			
		case INS_STATUS_SOLUTION_ALIGNMENT_COMPLETE:
		case INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE:
		case INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE:
			LED_COLOR_YELLOW();
			break;
		
		case INS_STATUS_SOLUTION_NAV:
			LED_COLOR_GREEN();
			break;
			
		case INS_STATUS_SOLUTION_AHRS:
			LED_COLOR_BLUE();
			break;
		}			
	}			
#endif	
}


void led_sys_heartbeat(void)
{
#if LED_SCHEME==LED_SCHEME_BLINK
	// Pulse LED status
	if( ++g_led.heartbeat_cnt >= 100 )
	{
		g_led.heartbeat_cnt = 0;
		g_led.heartbeat_state = 1;
		LED_ON(LED_SYS_HEALTH);
	}
	// Keep LED on for 100ms
	if( g_led.heartbeat_state && g_led.heartbeat_cnt >= g_led.heartbeat_duty )
	{
		g_led.heartbeat_state = 0;
		LED_OFF(LED_SYS_HEALTH);
	}
#else
	// Breathing LED status
#endif	
}


void rtos_maintenance(void)
{
	// Observe state of RTOS
	if (g_enRtosStats)
	{
		rtos_monitor(RTOS_NUM_TASKS);
	}

	// Check if task runtime has gone over the allotted period
	for (size_t i = TASK_SAMPLE; i < TASK_MAINTENANCE; i++)
	{
		if(g_rtos.task[TASK_SAMPLE].maxRunTimeUs > (g_rtos.task[TASK_SAMPLE].periodMs * 1000))
		{
			g_insStatus |= INS_STATUS_RTOS_TASK_PERIOD_OVERRUN;
		}
	}	
}


void save_persistent_messages(void)
{
    for(int port=0; port<NUM_COM_PORTS; port++)
    {   
        // Copy RMC
        g_nvmInternalFlashCfg->startupRmc[port] = g_rmci[port];
        
        // Copy ASCII
        g_nvmInternalFlashCfg->startupAsciiPeriod[port] = g_asciiPeriod[port];
    }   
	// Copy CAN
	g_nvmInternalFlashCfg->startupCAN = g_CAN;

	nvr_flash_config_write_needed_enable();     
}
