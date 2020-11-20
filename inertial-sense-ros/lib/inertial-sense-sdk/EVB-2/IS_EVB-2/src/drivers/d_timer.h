#ifndef _D_TIMER_H_
#define _D_TIMER_H_
#ifdef __cplusplus
extern "C" {
#endif

#ifdef NDEBUG

// enable additional timer interrupts, release mode only, makes debugging impossible
#define ENABLE_TC_WAVEFORM_INTERRUPT
#define ENABLE_TC_CAPTURE_INTERRUPT

#endif

// NOTE: TC2_IRQn is the IRQ id for TC0 channel 2 not for TC2
// see datasheet table on "peripheral identifiers"

#define TC_SAMPLE_SENSORS_CHANNEL  0
#define TC_SAMPLE_SENSORS_ID       ID_TC0
#define TC_SAMPLE_SENSORS_HANDLER  TC0_Handler
#define TC_SAMPLE_SENSORS_IRQn     TC0_IRQn
#define TC_SAMPLE_SENSORS_RATE_HZ  8000

// according to SAMS70 doc 46.5.1
// PA15 & PA16 (GPS# TPULSE) are on channel 1
#define TC_TIME_SYNC_CHANNEL   1
#define TC_TIME_SYNC_ID        ID_TC1
#define TC_TIME_SYNC_HANDLER   TC1_Handler
#define TC_TIME_SYNC_IRQn      TC1_IRQn
#define TC_TIME_SYNC_IER_MASK  TC_IER_LDRAS
//#define TC_TIME_SYNC_IER_MASK  TC_IER_ETRGS

//#define TC_TIME_SYNC_TICKS_TO_SEC  (0.000030517578125) // 1/32768 based on SLCK
#define TC_TIME_SYNC_TICKS_TO_SEC  (5.3333333333333333333333333333333e-8) // 8/150,000,000 based on MCK/8

#define TC_TIME_BASE_ID      ID_TC9
#define TC_TIME_CHANNEL      2
#define TC_TIME_HANDLER      TC9_Handler
#define TC_TIME_IRQn         TC9_IRQn

// typedefs
typedef struct
{
	/** Internal clock signals selection. */
	uint32_t ul_intclock;
	/** Waveform frequency (in Hz). */
	uint32_t us_frequency;
	/** Duty cycle in percent (positive).*/
	uint16_t us_dutycycle;
} waveconfig_t;

// prototypes
void timer_waveform_init(unsigned int channel, void (*func)(void));
void timer_capture_init(void);
int timer_capture_enable(void);
int timer_capture_disable(void);
uint32_t timer_captured(void);
void timer_time_init(void);
volatile uint64_t time_cv(void);

#ifdef __cplusplus
}
#endif
#endif // _D_TIMER_H_
