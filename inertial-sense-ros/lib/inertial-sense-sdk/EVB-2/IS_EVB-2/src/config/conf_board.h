/**
 * \file
 *
 * \brief Board configuration.
 *
 * Copyright (c) 2012-2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef CONF_BOARD_H_INCLUDED
#define CONF_BOARD_H_INCLUDED

#include <board.h>
#include "board_opt.h"

#define CONF_BOARD_CONFIG_MPU_AT_INIT

// Setup to use the Watchdog
#define ENABLE_WDT

#ifdef ENABLE_WDT
	#define CONF_BOARD_KEEP_WATCHDOG_AT_INIT    // Don't change WDT mode register in init.c
#endif

/* Enable ICache and DCache */
#if CONF_BOARD_ENABLE_DCACHE == 1
	#define CONF_BOARD_ENABLE_CACHE_AT_INIT
#endif

/** Enable SD MMC interface pins through HSMCI */
#define CONF_BOARD_SD_MMC_HSMCI
#define CONF_HSMCI_XDMAC_CHANNEL  XDMAC_CHANNEL_HWID_HSMCI

//! [tc_define_peripheral]
/* Use TC Peripheral 0. */
#define TC             TC0
#define TC_PERIPHERAL  0
//! [tc_define_peripheral]

//! [tc_define_ch1]
/* Configure TC0 channel 1 as waveform output. */
#define TC_CHANNEL_WAVEFORM 1
#define ID_TC_WAVEFORM      ID_TC1
#define PIN_TC_WAVEFORM     PIN_TC0_TIOA1
#define PIN_TC_WAVEFORM_MUX PIN_TC0_TIOA1_MUX
//! [tc_define_ch1]

//! [tc_define_ch2]
/* Configure TC0 channel 2 as capture input. */
#define TC_CHANNEL_CAPTURE 2
#define ID_TC_CAPTURE ID_TC2
#define PIN_TC_CAPTURE PIN_TC0_TIOA2
#define PIN_TC_CAPTURE_MUX PIN_TC0_TIOA2_MUX
//! [tc_define_ch2]

//! [tc_define_irq_handler]
/* Use TC2_Handler for TC capture interrupt. */
#define TC_Handler  TC2_Handler
#define TC_IRQn     TC2_IRQn
//! [tc_define_irq_handler]

// Serial interfaces
#define CONF_BOARD_SERIAL_UINS_SER0
#define CONF_BOARD_SERIAL_UINS_SER1
#define CONF_BOARD_SERIAL_XBEE
#define CONF_BOARD_SERIAL_EXT_RADIO
#define CONF_BOARD_SERIAL_ATWINC_BLE
#define CONF_BOARD_SERIAL_SP330
#define CONF_BOARD_SERIAL_GPIO_H8

// SPI interfaces
#define CONF_BOARD_SPI_UINS
#define CONF_BOARD_SPI_ATWINC_WIFI

// Encoder interfaces
#define CONF_BOARD_QUAD_ENCODER

// I2C interface
// #define CONF_BOARD_I2C_UINS

// CAN interface
 //#define CONF_BOARD_CAN1
 //#define CONF_BOARD_CAN_TEST

#endif /* CONF_BOARD_H_INCLUDED */
