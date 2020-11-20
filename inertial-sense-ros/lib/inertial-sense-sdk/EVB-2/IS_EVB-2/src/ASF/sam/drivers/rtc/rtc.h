/**
 * \file
 *
 * \brief Real-Time Clock (RTC) driver for SAM.
 *
 * Copyright (c) 2011-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef RTC_H_INCLUDED
#define RTC_H_INCLUDED

#include  "compiler.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

void rtc_set_hour_mode(Rtc *p_rtc, uint32_t ul_mode);
uint32_t rtc_get_hour_mode(Rtc *p_rtc);
void rtc_enable_interrupt(Rtc *p_rtc, uint32_t ul_sources);
void rtc_disable_interrupt(Rtc *p_rtc, uint32_t ul_sources);
uint32_t rtc_get_interrupt_mask(Rtc *p_rtc);
void rtc_get_time(Rtc *p_rtc, uint32_t *pul_hour, uint32_t *pul_minute,
		uint32_t *pul_second);
uint32_t rtc_set_time(Rtc *p_rtc, uint32_t ul_hour, uint32_t ul_minute,
		uint32_t ul_second);
uint32_t rtc_set_time_alarm(Rtc *p_rtc,
		uint32_t ul_hour_flag, uint32_t ul_hour,
		uint32_t ul_minute_flag, uint32_t ul_minute,
		uint32_t ul_second_flag, uint32_t ul_second);
void rtc_get_date(Rtc *p_rtc, uint32_t *pul_year, uint32_t *pul_month,
		uint32_t *pul_day, uint32_t *pul_week);
uint32_t rtc_set_date(Rtc *p_rtc, uint32_t ul_year, uint32_t ul_month,
		uint32_t ul_day, uint32_t ul_week);
uint32_t rtc_set_date_alarm(Rtc *p_rtc,
		uint32_t ul_month_flag, uint32_t ul_month,
		uint32_t ul_day_flag, uint32_t ul_day);
void rtc_clear_time_alarm(Rtc *p_rtc);
void rtc_clear_date_alarm(Rtc *p_rtc);
uint32_t rtc_get_status(Rtc *p_rtc);
void rtc_clear_status(Rtc *p_rtc, uint32_t ul_clear);
uint32_t rtc_get_valid_entry(Rtc *p_rtc);
void rtc_set_time_event(Rtc *p_rtc, uint32_t ul_selection);
void rtc_set_calendar_event(Rtc *p_rtc, uint32_t ul_selection);

#if ((SAM3S8) || (SAM3SD8) || (SAM4S) || (SAM4N) || (SAM4C) || (SAMG) || (SAM4CP) || (SAM4CM) || defined(__DOXYGEN__))
void rtc_set_calendar_mode(Rtc *p_rtc, uint32_t ul_mode);
uint32_t rtc_get_calendar_mode(Rtc *p_rtc);
void rtc_set_calibration(Rtc *p_rtc, uint32_t ul_direction_ppm,
		uint32_t ul_correction, uint32_t ul_range_ppm);
#endif

#if ((SAM3S8) || (SAM3SD8) || (SAM4S) || (SAM4C) || (SAMG) || (SAM4CP) || (SAM4CM) || SAMV71 || SAMV70 || SAME70 || SAMS70 || defined(__DOXYGEN__))
void rtc_set_waveform(Rtc *p_rtc, uint32_t ul_channel, uint32_t ul_value);
#if ((SAM3S8) || (SAM3SD8) || (SAM4S) || (SAM4C)|| (SAM4CP) || (SAM4CM) || SAMV71 || SAMV70 || SAME70 || SAMS70 || defined(__DOXYGEN__))
void rtc_set_pulse_parameter(Rtc *p_rtc, uint32_t ul_time_high,
		uint32_t ul_period);
#endif
#endif

#if ((SAM3N) || (SAM3U) || (SAM3XA) || defined(__DOXYGEN__))
void rtc_set_writeprotect(Rtc *p_rtc, uint32_t ul_enable);
#endif /* ((SAM3N) || (SAM3U) || (SAM3XA)) */

#if ((SAM4C) || (SAM4CP) || (SAM4CM) || defined(__DOXYGEN__))
void rtc_get_tamper_time(Rtc *p_rtc, uint32_t *pul_hour, uint32_t *pul_minute,
		uint32_t *pul_second, uint8_t reg_num);
void rtc_get_tamper_date(Rtc *p_rtc, uint32_t *pul_year, uint32_t *pul_month,
		uint32_t *pul_day, uint32_t *pul_week, uint8_t reg_num);
uint32_t rtc_get_tamper_source(Rtc *p_rtc, uint8_t reg_num);
uint32_t rtc_get_tamper_event_counter(Rtc *p_rtc);
bool rtc_is_tamper_occur_in_backup_mode(Rtc *p_rtc, uint8_t reg_num);
#endif

#if (SAMG55 || defined(__DOXYGEN__))
uint32_t rtc_get_milliseconds(Rtc *p_rtc);
#endif

/**
 * \page sam_rtc_quickstart Quickstart guide for SAM RTC driver
 *
 * This is the quickstart guide for the \ref rtc_group "SAM RTC driver",
 * with step-by-step instructions on how to configure and use the driver in a
 * selection of use cases.
 *
 * The use cases contain several code fragments. The code fragments in the
 * steps for setup can be copied into a custom initialization function, while
 * the steps for usage can be copied into, e.g., the main application function.
 *
 * \section rtc_basic_use_case Basic use case
 * In this basic use case, the RTC module is using 32kHz external crystal and
 * configured for 24-hour mode. It will read the current date and time.
 *
 * \subsection sam_rtc_quickstart_prereq Prerequisites
 * -# \ref sysclk_group "System Clock Management (Sysclock)"
 *
 * \section rtc_basic_use_case_setup Setup steps
 * \subsection rtc_basic_use_case_setup_code Example code
 * Add to application C-file:
 * \code
	   void rtc_setup(void)
	   {
	       pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL);

	       while (!pmc_osc_is_ready_32kxtal());

	       rtc_set_hour_mode(RTC, 0);
	   }
\endcode
 *
 * \subsection rtc_basic_use_case_setup_flow Workflow
 *   - \note Please make sure the external 32kHz crystal is available.
 * -# Enable the External 32K crystal :
 *   - \code pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL); \endcode
 * -# Wait for 32K crystal ready:
 *   - \code while (!pmc_osc_is_ready_32kxtal()); \endcode
 * -# Set default RTC configuration, 24-hour mode .
 *   - \code rtc_set_hour_mode(RTC, 0); \endcode
 *
 * \section rtc_basic_use_case_usage Usage steps
 * \subsection rtc_basic_use_case_usage_code Example code
 * Add to, e.g., main loop in application C-file:
 * \code
	    uint32_t hour, minute, second;
	    uint32_t year, month, day, week;

	    rtc_get_time(RTC, &hour, &minute, &second);
	    rtc_get_date(RTC, &year, &month, &day, &week);
\endcode
 *
 * \subsection rtc_basic_use_case_usage_flow Workflow
 * -# Start Define the variables for the date and time:
 *   - \code uint32_t hour, minute, second; \endcode
 *   - \code uint32_t year, month, day, week; \endcode
 * -# Read current time:
 *   - \code rtc_get_time(RTC, &hour, &minute, &second); \endcode
 * -# Read current date:
 *   - \code rtc_get_date(RTC, &year, &month, &day, &week); \endcode
 *
 */

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* RTC_H_INCLUDED */
