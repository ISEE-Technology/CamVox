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

#ifndef RTT_H_INCLUDED
#define RTT_H_INCLUDED

#include "compiler.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

uint32_t rtt_init(Rtt *p_rtt, uint16_t us_prescaler);
#if (SAM4N || SAM4S || SAM4E || SAM4C || SAMG || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70)
void rtt_sel_source(Rtt *p_rtt, bool is_rtc_sel);
void rtt_enable(Rtt *p_rtt);
void rtt_disable(Rtt *p_rtt);
#endif
void rtt_enable_interrupt(Rtt *p_rtt, uint32_t ul_sources);
void rtt_disable_interrupt(Rtt *p_rtt, uint32_t ul_sources);
uint32_t rtt_read_timer_value(Rtt *p_rtt);
uint32_t rtt_get_status(Rtt *p_rtt);
uint32_t rtt_write_alarm_time(Rtt *p_rtt, uint32_t ul_alarm_time);

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* RTT_H_INCLUDED */
