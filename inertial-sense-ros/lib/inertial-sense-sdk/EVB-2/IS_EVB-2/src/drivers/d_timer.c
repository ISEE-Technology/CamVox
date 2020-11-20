#include <asf.h>
#include <tc.h>
#include "d_timer.h"
#ifdef ENABLE_TC_WAVEFORM_INTERRUPT
#include "conf_interrupts.h"
#endif // ENABLE_TC_WAVEFORM_INTERRUPT
// #include "globals.h"
#include "misc/debug_gpio.h"

#ifdef DEBUG
//#define TIMER_DEBUG
//#define TIMER_IO_DEBUG
#endif // DEBUG

#ifdef TIMER_IO_DEBUG
#if (BOARD == SAME70_XPLAINED)
#define TIMER_PIN  PIN_TC0_TIOA0
#elif (BOARD == USER_BOARD)
#define TIMER_PIN  GPIO_8_PIN
#endif // BOARD
#endif // TIMER_IO_DEBUG

// source based on example acquired from:
// https://github.com/avrxml/asf/blob/master/sam/drivers/tc/tc_capture_waveform_example


//----------------------------------------
// waveform mode
//----------------------------------------
// The last one is meaningless
static const uint32_t g_divisors[5] = { 2, 8, 32, 128, 0 };
static void (*g_wvf_callback)(void) = NULL;
static unsigned int g_wvf_channel;

static void timer_io_enable(void)
{
#ifdef TIMER_IO_DEBUG
//#if (BOARD == SAME70_XPLAINED)
//	ioport_set_pin_mode(PIN_TC0_TIOA0, PIN_TC0_TIOA0_MUX);
//	ioport_disable_pin(PIN_TC0_TIOA0);
//#endif // BOARD
	ioport_set_pin_dir(TIMER_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(TIMER_PIN, 0);
#endif // TIMER_IO_DEBUG
}


#ifdef ENABLE_TC_WAVEFORM_INTERRUPT
#ifdef TIMER_DEBUG
#include "d_time.h"
static uint32_t g_times[TC_SAMPLE_SENSORS_RATE_HZ+1];

static inline void timer_report(void)
{
	uint32_t k;
	uint32_t sum = 0;
	uint32_t diffs = 0;
	uint32_t diff;

	for (k=1; k<TC_SAMPLE_SENSORS_RATE_HZ; k++)
	{
		if (g_times[k] > g_times[k-1])
		{
			sum += g_times[k] - g_times[k-1];
			diffs++;
		}
	}
	//printf("diffs=%04u sum=%u\r\n", (unsigned int)diffs, (unsigned int)sum);
	printf("avg=%u us expected=%u us\r\n",
		(unsigned int)((sum/TIME_TICKS_PER_US)/diffs),
		(unsigned int)(1000000.0/(TC_SAMPLE_SENSORS_RATE_HZ*1.0)));
	diff = g_times[TC_SAMPLE_SENSORS_RATE_HZ]-g_times[0];
	printf("t0=%u t%d=%u diff=%u\r\n",
		(unsigned int)g_times[0],
		TC_SAMPLE_SENSORS_RATE_HZ,
		(unsigned int)g_times[TC_SAMPLE_SENSORS_RATE_HZ],
		(unsigned int)diff);
}
#endif // TIMER_DEBUG


static void timer_register_callback(void (*func)(void))
{
	g_wvf_callback = func;
}



void TC_SAMPLE_SENSORS_HANDLER(void)
{
	if (tc_get_status(TC0, TC_SAMPLE_SENSORS_CHANNEL) & TC_IMR_CPCS)
	{
		tc_disable_interrupt(TC0, TC_SAMPLE_SENSORS_CHANNEL, TC_IDR_CPCS);
		NVIC_DisableIRQ(TC_SAMPLE_SENSORS_IRQn);
		NVIC_ClearPendingIRQ(TC_SAMPLE_SENSORS_IRQn);
		if (g_wvf_callback)
			(*g_wvf_callback)();
		NVIC_EnableIRQ(TC_SAMPLE_SENSORS_IRQn);
		tc_enable_interrupt(TC0, TC_SAMPLE_SENSORS_CHANNEL, TC_IER_CPCS);
	}
}
#endif // ENABLE_TC_WAVEFORM_INTERRUPT


void timer_waveform_init(unsigned int channel, void (*func)(void))
{
	uint32_t ra, rc;
	waveconfig_t wvf = {TC_CMR_TCCLKS_TIMER_CLOCK2, TC_SAMPLE_SENSORS_RATE_HZ, 50};
		
	g_wvf_channel = channel;
	g_wvf_callback = func;

	timer_io_enable();

	// Configure the PMC to enable the TC module.
	sysclk_enable_peripheral_clock(TC_SAMPLE_SENSORS_ID);

#if SAMG55
	// Enable PCK output
	pmc_disable_pck(PMC_PCK_3);
	pmc_switch_pck_to_mck(PMC_PCK_3, PMC_PCK_PRES(0));
	pmc_enable_pck(PMC_PCK_3);
#endif // SAMG55

	// Init TC to waveform mode.
	tc_init(TC0, g_wvf_channel,
		wvf.ul_intclock     // Waveform Clock Selection
		| TC_CMR_WAVE       // Waveform mode is enabled
		| TC_CMR_ACPA_SET   // RA Compare Effect: set
		| TC_CMR_ACPC_CLEAR // RC Compare Effect: clear
		| TC_CMR_CPCTRG     // UP mode with automatic trigger on RC Compare
		);

	// Configure waveform frequency and duty cycle.
	rc = (sysclk_get_peripheral_bus_hz(TC0) / g_divisors[wvf.ul_intclock]) / wvf.us_frequency;
	tc_write_rc(TC0, g_wvf_channel, rc);
	ra = (100 - wvf.us_dutycycle) * rc / 100;
	tc_write_ra(TC0, g_wvf_channel, ra);

#ifdef ENABLE_TC_WAVEFORM_INTERRUPT
	timer_register_callback(func);
	tc_disable_interrupt(TC0, g_wvf_channel, TC_IDR_CPCS);
	NVIC_DisableIRQ(TC_SAMPLE_SENSORS_IRQn);
	NVIC_ClearPendingIRQ(TC_SAMPLE_SENSORS_IRQn);
	NVIC_SetPriority(TC_SAMPLE_SENSORS_IRQn, INT_PRIORITY_TC);
	NVIC_EnableIRQ(TC_SAMPLE_SENSORS_IRQn);
	tc_enable_interrupt(TC0, g_wvf_channel, TC_IER_CPCS);
#endif // ENABLE_TC_WAVEFORM_INTERRUPT

#ifdef TIMER_DEBUG
	printf("rc: %u, ra: %u\r\n", (unsigned int)rc, (unsigned int)ra);
	printf("Start waveform: Frequency = %d Hz,Duty Cycle = %2d%%\n\r",
		wvf.us_frequency, wvf.us_dutycycle);
#endif // TIMER_DEBUG
}


//----------------------------------------
// capture mode
//----------------------------------------
static int g_capture_initialized;
#ifdef ENABLE_TC_CAPTURE_INTERRUPT
static uint32_t g_captured;
extern void gps_sync_int_handler(int index, int addTimeSinceTrigger);


void TC_TIME_SYNC_HANDLER(void)
{
	uint32_t status = tc_get_status(TC0, TC_TIME_SYNC_CHANNEL);

	if (status & TC_TIME_SYNC_IER_MASK)
	{
#if 0
		if (TC_CAPTURE_IER_MASK & TC_IER_LDRAS)
			g_captured = tc_read_ra(TC0, TC_CAPTURE_CHANNEL);
		else
			g_captured = tc_read_cv(TC0, TC_CAPTURE_CHANNEL);

		if (!(g_nvmFlashCfg->sysCfgBits & SYS_CFG_BITS_DISABLE_LEDS) && g_sysParams.hStatus&HDW_STATUS_GPS_SATELLITE_RX)
		{
			LEDS_GPS_PULSE();
			g_led.gps_timeMs = g_gpsTimeOfWeekMs;
		}
#else
		gps_sync_int_handler(0,1);
#endif // 0
	}
}
#endif // ENABLE_TC_CAPTURE_INTERRUPT


int timer_capture_enable(void)
{
	if (!g_capture_initialized)
		return -1;

#ifdef ENABLE_TC_CAPTURE_INTERRUPT
	tc_enable_interrupt(TC0, TC_TIME_SYNC_CHANNEL, TC_TIME_SYNC_IER_MASK);
#endif // ENABLE_TC_CAPTURE_INTERRUPT
	tc_start(TC0, TC_TIME_SYNC_CHANNEL);

	return 0;
}


int timer_capture_disable(void)
{
	if (!g_capture_initialized)
		return -1;

#ifdef ENABLE_TC_CAPTURE_INTERRUPT
	tc_disable_interrupt(TC0, TC_TIME_SYNC_CHANNEL, TC_TIME_SYNC_IER_MASK);
#endif // ENABLE_TC_CAPTURE_INTERRUPT
	tc_stop(TC0, TC_TIME_SYNC_CHANNEL);

	return 0;
}


void timer_capture_init(void)
{
	if (g_capture_initialized)
		return;

	// Configure the PMC to enable the TC module.
	sysclk_enable_peripheral_clock(TC_TIME_SYNC_ID);

#if SAMG55
	// Enable PCK output
	pmc_disable_pck(PMC_PCK_3);
	pmc_switch_pck_to_mck(PMC_PCK_3, PMC_PCK_PRES(0));
	pmc_enable_pck(PMC_PCK_3);
#endif // SAMG55

	// Init TC to capture mode.
	tc_init(TC0, TC_TIME_SYNC_CHANNEL,
		TC_CMR_TCCLKS_TIMER_CLOCK2 // Clock Selection
		| TC_CMR_LDRA_RISING       // RA Loading: rising edge of TIOA
		| TC_CMR_ABETRG            // External Trigger: TIOA
		| TC_CMR_ETRGEDG_RISING    // External Trigger Edge: Rising edge
	);

#ifdef ENABLE_TC_CAPTURE_INTERRUPT
	tc_disable_interrupt(TC0, TC_TIME_SYNC_CHANNEL, TC_TIME_SYNC_IER_MASK);
	NVIC_DisableIRQ(TC_TIME_SYNC_IRQn);
	NVIC_ClearPendingIRQ(TC_TIME_SYNC_IRQn);
	NVIC_SetPriority(TC_TIME_SYNC_IRQn, INT_PRIORITY_TC);
	NVIC_EnableIRQ(TC_TIME_SYNC_IRQn);
	tc_enable_interrupt(TC0, TC_TIME_SYNC_CHANNEL, TC_TIME_SYNC_IER_MASK);
#endif // ENABLE_TC_CAPTURE_INTERRUPT

	tc_start(TC0, TC_TIME_SYNC_CHANNEL);

	g_capture_initialized = 1;
}


uint32_t timer_captured(void)
{
#ifdef ENABLE_TC_CAPTURE_INTERRUPT
	return g_captured;
#else
	tc_stop(TC0, TC_TIME_SYNC_CHANNEL);
	return tc_read_ra(TC0, TC_TIME_SYNC_CHANNEL);
#endif // ENABLE_TC_CAPTURE_INTERRUPT
}


//----------------------------------------
// time timer
//----------------------------------------
static uint16_t g_rollover = 0;


void TC_TIME_HANDLER(void)
{
	if (tc_get_status(TC3, TC_TIME_CHANNEL) & TC_IMR_CPCS)
	{
		g_rollover++;
	}
}


volatile uint64_t time_cv(void)
{
	union
	{
		uint16_t u16[4];
		uint64_t u64;
	} ticks;

	ticks.u16[3] = g_rollover;
	ticks.u16[2] = (uint16_t) tc_read_cv(TC3, 2);
	ticks.u16[1] = (uint16_t) tc_read_cv(TC3, 1);
	ticks.u16[0] = (uint16_t) tc_read_cv(TC3, 0);

	return ticks.u64;
}


static void time_channel_init(uint32_t channel, uint32_t intclock)
{
	// Configure the PMC to enable the TC module.
	sysclk_enable_peripheral_clock(TC_TIME_BASE_ID + channel);

	// Init TC to waveform mode.
	tc_init(TC3, channel,
		intclock            // Waveform Clock Selection
		| TC_CMR_WAVE       // Waveform mode is enabled
		| TC_CMR_WAVSEL_UP  // WAVSEL = 00 without Trigger
		| TC_CMR_ACPA_CLEAR // RA Compare Effect: clear
		| TC_CMR_ACPC_SET   // RC Compare Effect: set
		| TC_CMR_CPCTRG     // UP mode with automatic trigger on RC Compare
		);

	// Configure waveform frequency and duty cycle.
	tc_write_ra(TC3, channel, 0xFF);
	tc_write_rc(TC3, channel, 0xFFFF);
}


void timer_time_init(void)
{
#ifdef ENABLE_TC_WAVEFORM_INTERRUPT
	time_channel_init(0, TC_CMR_TCCLKS_TIMER_CLOCK2);
	time_channel_init(1, TC_CMR_TCCLKS_XC1);
	time_channel_init(2, TC_CMR_TCCLKS_XC2);

	tc_set_block_mode(TC3,
		TC_BMR_TC1XC1S_TIOA0 | // select TIOA0 as external clock signal for channel 1
		TC_BMR_TC2XC2S_TIOA1); // select TIOA1 as external clock signal for channel 2

	// the last channel will generate an interrupt for rollover
	tc_disable_interrupt(TC3, TC_TIME_CHANNEL, TC_IDR_CPCS);
	NVIC_DisableIRQ(TC_TIME_IRQn);
	NVIC_ClearPendingIRQ(TC_TIME_IRQn);
	NVIC_SetPriority(TC_TIME_IRQn, INT_PRIORITY_TC);
	NVIC_EnableIRQ(TC_TIME_IRQn);
	tc_enable_interrupt(TC3, TC_TIME_CHANNEL, TC_IER_CPCS);

	tc_start(TC3, 2);
	tc_start(TC3, 1);
	tc_start(TC3, 0);

	uint32_t previous_time = tc_read_cv(TC3, 0);
	while (previous_time == tc_read_cv(TC3, 0));
#endif
}
