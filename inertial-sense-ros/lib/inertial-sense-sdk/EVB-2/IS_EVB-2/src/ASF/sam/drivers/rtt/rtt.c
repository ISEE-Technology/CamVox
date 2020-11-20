/**
 * \file
 *
 * \brief Real-time Timer (RTT) driver for SAM.
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

#include "rtt.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup sam_drivers_rtt_group Real-time Timer (RTT)
 *
 * The Real-time Timer is built around a 32-bit counter used to count
 * roll-over events of the programmable 16-bit prescaler, which enables
 * counting elapsed seconds from a 32 kHz slow clock source.
 * This is a driver for configuration and use of the RTT peripheral.
 *
 * @{
 */

/*
 * In follow series chip, the bit RTC1HZ and RTTDIS in RTT_MR are write only.
 * So we use a variable to record status of these bits.
 */
#if (SAM4N || SAM4S || SAM4E || SAM4C || SAMG51 || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70)
static uint32_t g_wobits_in_rtt_mr = 0;
#endif

/**
 * \brief Initialize the given RTT.
 *
 * \note This function restarts the real-time timer. If w_prescaler is equal to zero,
 *  the prescaler period is equal to 2^16 * SCLK period. If not, the prescaler period
 *  is equal to us_prescaler * SCLK period.
 *
 * \param p_rtt Pointer to an RTT instance.
 * \param us_prescaler Prescaler value for the RTT.
 *
 * \return 0 if successful.
 */
uint32_t rtt_init(Rtt *p_rtt, uint16_t us_prescaler)
{
#if (SAM4N || SAM4S || SAM4E || SAM4C || SAMG51 || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70)
	p_rtt->RTT_MR = (us_prescaler | RTT_MR_RTTRST | g_wobits_in_rtt_mr);
#else
	p_rtt->RTT_MR = (us_prescaler | RTT_MR_RTTRST);
#endif
	return 0;
}

#if (SAM4N || SAM4S || SAM4E || SAM4C || SAMG51 || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70)
/**
 * \brief Select RTT counter source.
 *
 * \param p_rtt Pointer to an RTT instance.
 * \param is_rtc_sel RTC 1Hz Clock Selection.
 */
void rtt_sel_source(Rtt *p_rtt, bool is_rtc_sel)
{
	if(is_rtc_sel) {
		g_wobits_in_rtt_mr |= RTT_MR_RTC1HZ;
		p_rtt->RTT_MR |= g_wobits_in_rtt_mr;
	} else {
		g_wobits_in_rtt_mr &= ~RTT_MR_RTC1HZ;
		p_rtt->RTT_MR |= g_wobits_in_rtt_mr;
	}
}

/**
 * \brief Enable RTT.
 *
 * \param p_rtt Pointer to an RTT instance.
 */
void rtt_enable(Rtt *p_rtt)
{
	g_wobits_in_rtt_mr &= ~RTT_MR_RTTDIS;
	p_rtt->RTT_MR |= g_wobits_in_rtt_mr;
}
/**
 * \brief Disable RTT.
 *
 * \param p_rtt Pointer to an RTT instance.
 */
void rtt_disable(Rtt *p_rtt)
{
	g_wobits_in_rtt_mr |= RTT_MR_RTTDIS;
	p_rtt->RTT_MR |= g_wobits_in_rtt_mr;
}
#elif (SAMG53 || SAMG54 || SAMG55)
void rtt_sel_source(Rtt *p_rtt, bool is_rtc_sel)
{
	if(is_rtc_sel) {
		p_rtt->RTT_MR |= RTT_MR_RTC1HZ;
	} else {
		p_rtt->RTT_MR &= ~RTT_MR_RTC1HZ;
	}
}

void rtt_enable(Rtt *p_rtt)
{
	p_rtt->RTT_MR &= ~RTT_MR_RTTDIS;
}
/**
 * \brief Disable RTT.
 *
 * \param p_rtt Pointer to an RTT instance.
 */
void rtt_disable(Rtt *p_rtt)
{
	p_rtt->RTT_MR |= RTT_MR_RTTDIS;
}
#endif

/**
 * \brief Enable RTT interrupts.
 *
 * \param p_rtt Pointer to an RTT instance.
 * \param ul_sources Interrupts to be enabled.
 */
void rtt_enable_interrupt(Rtt *p_rtt, uint32_t ul_sources)
{
	uint32_t temp;

	temp = p_rtt->RTT_MR;
	temp |= ul_sources;
#if (SAM4N || SAM4S || SAM4E || SAM4C || SAMG51 || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70)
	temp |= g_wobits_in_rtt_mr;
#endif
	p_rtt->RTT_MR = temp;
}

/**
 * \brief Disable RTT interrupts.
 *
 * \param p_rtt Pointer to an RTT instance.
 * \param ul_sources Interrupts to be disabled.
 */
void rtt_disable_interrupt(Rtt *p_rtt, uint32_t ul_sources)
{
	uint32_t temp = 0;

	temp = p_rtt->RTT_MR;
	temp &= (~ul_sources);
#if (SAM4N || SAM4S || SAM4E || SAM4C || SAMG51 || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70)
	temp |= g_wobits_in_rtt_mr;
#endif
	p_rtt->RTT_MR = temp;
}

/**
 * \brief Read the current value of the RTT timer value.
 *
 * \param p_rtt Pointer to an RTT instance.
 *
 * \return The current Real-time Timer value.
 */
uint32_t rtt_read_timer_value(Rtt *p_rtt)
{
	uint32_t rtt_val = p_rtt->RTT_VR;

	while (rtt_val != p_rtt->RTT_VR) {
		rtt_val = p_rtt->RTT_VR;
	}

	return rtt_val;
}

/**
 * \brief Get the status register value of the given RTT.
 *
 * \param p_rtt Pointer to an RTT instance.
 *
 * \return The Real-time Timer status.
 */
uint32_t rtt_get_status(Rtt *p_rtt)
{
	return p_rtt->RTT_SR;
}

/**
 * \brief Configure the RTT to generate an alarm at the given time.
 * alarm happens when CRTV value equals ALMV+1, so RTT_AR should be alarmtime - 1.
 * if you want to get alarm when rtt hit 0 , ALMV should be set to 0xFFFFFFFF.
 *
 * \param p_rtt Pointer to an RTT instance.
 * \param ul_alarm_time Alarm time,Alarm time = ALMV + 1.
 *
 * \retval 0 Configuration is done.
 */
uint32_t rtt_write_alarm_time(Rtt *p_rtt, uint32_t ul_alarm_time)
{
	uint32_t flag;

	flag = p_rtt->RTT_MR & RTT_MR_ALMIEN;

	rtt_disable_interrupt(RTT, RTT_MR_ALMIEN);

	/**
	 * Alarm time = ALMV + 1,If the incoming parameter 
	 * is 0, the ALMV is set to 0xFFFFFFFF.
	*/
	if(ul_alarm_time == 0) {
		p_rtt->RTT_AR = 0xFFFFFFFF;
	}
	else {
		p_rtt->RTT_AR = ul_alarm_time - 1;
	}

	if (flag) {
		rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	}

	return 0;
}

//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
