/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ 

#include <asf.h>
#include "d_quadEnc.h"
#include "conf_d_quadEnc.h"


//Use of index signal is not support at this time. Needs setup for pin and reading values
#define QDEC_USE_INDEX	0

static void quadEncSetModePosition(Tc *const timercounter, uint32_t ID_timercounter)
{
	//Enable the QDEC channel clocks
	sysclk_enable_peripheral_clock(ID_timercounter);		//Channel 0
#if QDEC_USE_INDEX
	sysclk_enable_peripheral_clock(ID_timercounter + 1);	//Channel 1
#endif
		
	//Init TC channel 0 to QDEC Position mode
	tc_init(timercounter, 0,
		TC_CMR_TCCLKS_XC0
		| TC_CMR_ETRGEDG_RISING     /* To clear the counter on a rising edge of the TIOA signal*/
		| TC_CMR_ABETRG             /* To select TIOA as a trigger for this channel 0 */
	);

#if QDEC_USE_INDEX
	//Init TC channel 1 to QDEC Rotation mode
	tc_init(timercounter, 1,
		/* QDEC Clock Selection */
		TC_CMR_TCCLKS_XC0
	);
#endif

	//Enable TC QDEC channel 0 in QDEC Position mode
	tc_set_block_mode(timercounter, TC_BMR_QDEN /* QDEC mode enabled */
		| TC_BMR_POSEN              /* Position measure is enabled */
		| TC_BMR_EDGPHA             /* Detect quadrature on both PHA and PHB (4X decoding)*/
		| (0<< TC_BMR_MAXFILT_Pos)  /* enable filter on input signal*/
	);	
	
	tc_start(timercounter, 0);	//For position measurement
#if QDEC_USE_INDEX
	tc_start(timercounter, 1);	//For rotation measurement
#endif
}

static uint32_t speed_capture[2] = {0, 0};
static uint32_t speed_capture_timeMs = 0;

void TCCAP0_SPD_Handler(void)
{
	static bool running = false;
	
	uint32_t status = tc_get_status(TCCAP0_SPD, TCCAP0_SPD_CHANNEL);
	
	if ((status & TC_SR_LDRAS) != 0)
	{
		if(running)
		{
			speed_capture[0] = tc_read_ra(TCCAP0_SPD, TCCAP0_SPD_CHANNEL);
			speed_capture_timeMs = time_msec();
		}
		else
		{
			//first read after stopping is invalid
			tc_read_ra(TCCAP0_SPD, TCCAP0_SPD_CHANNEL);	//discard read
			running = true;
		}
	}

	if ((status & TC_SR_COVFS) != 0)
	{
		//Timer overflow so we assume we are not moving
		speed_capture[0] = 0;
		speed_capture_timeMs = time_msec();
		running = false;
	}
}

void TCCAP1_SPD_Handler(void)
{
	static bool running = false;
	
	uint32_t status = tc_get_status(TCCAP1_SPD, TCCAP1_SPD_CHANNEL);
	
	if ((status & TC_SR_LDRAS) != 0)
	{
		if(running)
		{
			speed_capture[1] = tc_read_ra(TCCAP1_SPD, TCCAP1_SPD_CHANNEL);
			speed_capture_timeMs = time_msec();
		}
		else
		{
			//first read after stopping is invalid
			tc_read_ra(TCCAP1_SPD, TCCAP1_SPD_CHANNEL);	//discard read
			running = true;
		}
	}

	if ((status & TC_SR_COVFS) != 0)
	{
		//Timer overflow so we assume we are not moving
		speed_capture[1] = 0;
		speed_capture_timeMs = time_msec();
		running = false;
	}
}

static void quadEncSetModeSpeed(Tc *const timercounter, int timerchannel, int timerirq, uint32_t ID_timercounter)
{
	//Enable the TC channel clock
	sysclk_enable_peripheral_clock(ID_timercounter);

	if(!pmc_is_pck_enabled(PMC_PCK_6))
	{
		pmc_disable_pck(PMC_PCK_6);
		pmc_switch_pck_to_mainck(PMC_PCK_6, PMC_PCK_PRES(239));
		pmc_enable_pck(PMC_PCK_6);
	}
			
	// Init TC to capture mode.  20us per counter LSB.
	tc_init(timercounter, timerchannel,
		TC_CMR_TCCLKS_TIMER_CLOCK1  /* Clock Selection */
		| TC_CMR_LDRA_EDGE          /* RA Loading: rising edge of TIOA */
		| TC_CMR_ABETRG             /* External Trigger: TIOA */
		| TC_CMR_ETRGEDG_EDGE	    /* External Trigger Edge: rising */
	);
				
	// Setup capture and overflow interrupt		
	NVIC_DisableIRQ(timerirq);
	NVIC_ClearPendingIRQ(timerirq);
	NVIC_SetPriority(timerirq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY-1);
	NVIC_EnableIRQ(timerirq);
	tc_enable_interrupt(timercounter, timerchannel, TC_IER_LDRAS | TC_IER_COVFS);
	
	tc_start(timercounter, timerchannel);
}

void quadEncInit(void)
{
	/**** Configure pins ****/
	// Left Encoder - Position
	ioport_set_pin_mode(PIN_QE0_POS_PHA, PIN_QE0_POS_PHA_MUX);
	ioport_disable_pin(PIN_QE0_POS_PHA);
	ioport_set_pin_mode(PIN_QE0_POS_PHB, PIN_QE0_POS_PHB_MUX);
	ioport_disable_pin(PIN_QE0_POS_PHB);

	// Left Encoder - Speed
	ioport_set_pin_mode(PIN_TCCAP0_SPD, PIN_TCCAP0_SPD_MUX);
	ioport_disable_pin(PIN_TCCAP0_SPD);
	
	// Right Encoder - Position
	ioport_set_pin_mode(PIN_QE1_POS_PHA, PIN_QE1_POS_PHA_MUX);
	ioport_disable_pin(PIN_QE1_POS_PHA);
	ioport_set_pin_mode(PIN_QE1_POS_PHB, PIN_QE1_POS_PHB_MUX);
	ioport_disable_pin(PIN_QE1_POS_PHB);

	// Right Encoder - Speed
	ioport_set_pin_mode(PIN_TCCAP1_SPD, PIN_TCCAP1_SPD_MUX);
	ioport_disable_pin(PIN_TCCAP1_SPD);
	
	/**** Setup hardware ****/
	quadEncSetModePosition(QE0_POS, ID_QE0_POS);
	quadEncSetModeSpeed(TCCAP0_SPD, TCCAP0_SPD_CHANNEL, TCCAP0_SPD_IRQn, ID_TCCAP0_SPD);
	
	quadEncSetModePosition(QE1_POS, ID_QE1_POS);
	quadEncSetModeSpeed(TCCAP1_SPD, TCCAP1_SPD_CHANNEL, TCCAP1_SPD_IRQn, ID_TCCAP1_SPD);
}

void quadEncReadPositionAll(int *pos0, bool *dir0, int *pos1, bool *dir1)
{
	int16_t cv;

	taskENTER_CRITICAL();
	
	cv = QE0_POS->TC_CHANNEL[0].TC_CV;
	*pos0 = cv;
	*dir0 = (QE0_POS->TC_QISR & TC_QISR_DIR) / TC_QISR_DIR;
	
	cv = QE1_POS->TC_CHANNEL[0].TC_CV;
	*pos1 = cv;
	*dir1 = (QE1_POS->TC_QISR & TC_QISR_DIR) / TC_QISR_DIR;

	taskEXIT_CRITICAL();
}

void quadEncReadSpeedAll(uint32_t *speed0, uint32_t *speed1)
{
	taskENTER_CRITICAL();
	
	uint32_t dtMs = time_msec() - speed_capture_timeMs;
	if(dtMs < 100)
	{
		*speed0 = speed_capture[0];
		*speed1 = speed_capture[1];		
	}
	else
	{	// Return zero if encoder pulses occur slower than 100ms
		*speed0 = 0;
		*speed1 = 0;
	}	

	taskEXIT_CRITICAL();
}

void test_quad_encoders(void)
{
#if	1	//quadEnc testing

	udi_cdc_write_buf("Running\r\n", 9);
	quadEncInit();

	while(1)
	{
		#define BUF_SIZE 100
		char str[BUF_SIZE];
		int chL, chR;
		bool dirL, dirR;
		int speedL, speedR;
		
		quadEncReadPositionAll(&chL, &dirL, &chR, &dirR);
		quadEncReadSpeedAll((uint32_t*)&speedL, (uint32_t*)&speedR);
		
		// Set velocity direction
		if(dirL)
		{
			speedL = -speedL;
		}
		if(dirR)
		{
			speedR = -speedR;
		}
		
		int len = SNPRINTF(str, BUF_SIZE, "ch0 %c %7d %7d ch1 %c %7d %7d\r\n",
				dirL ? 'R':'F', chL, (int)speedL,
				dirR ? 'R':'F', chR, (int)speedR);
		udi_cdc_write_buf(str, len);
		
		vTaskDelay(200);
	}

#endif	//END quadEnc testing
}