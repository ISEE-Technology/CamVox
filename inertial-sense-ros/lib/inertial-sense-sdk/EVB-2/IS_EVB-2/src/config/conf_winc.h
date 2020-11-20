/**
 *
 * \file
 *
 * \brief WINC3400 configuration.
 *
 * Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef CONF_WINC_H_INCLUDED
#define CONF_WINC_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"

/** Settings for EVB-2. */

/*
   ---------------------------------
   ---------- PIN settings ---------
   ---------------------------------
*/

#define CONF_WINC_PIN_RESET				IOPORT_CREATE_PIN(PIOC, 21)
#define CONF_WINC_PIN_CHIP_ENABLE		IOPORT_CREATE_PIN(PIOC, 22)
#define CONF_WINC_PIN_WAKE				IOPORT_CREATE_PIN(PIOC, 31)	//Not On Board

/*
   ---------------------------------
   ---------- SPI settings ---------
   ---------------------------------
*/

#define CONF_WINC_USE_SPI				(1)

/** SPI pin and instance settings. */
#define CONF_WINC_SPI					SPI1
#define CONF_WINC_SPI_ID				ID_SPI1
#define CONF_WINC_SPI_MISO_GPIO			PIO_PC26_IDX
#define CONF_WINC_SPI_MISO_FLAGS		IOPORT_MODE_MUX_C
#define CONF_WINC_SPI_MOSI_GPIO			PIO_PC27_IDX
#define CONF_WINC_SPI_MOSI_FLAGS		IOPORT_MODE_MUX_C
#define CONF_WINC_SPI_CLK_GPIO			PIO_PC24_IDX
#define CONF_WINC_SPI_CLK_FLAGS			IOPORT_MODE_MUX_C
#define CONF_WINC_SPI_CS_GPIO			PIO_PC25_IDX
#define CONF_WINC_SPI_CS_FLAGS			IOPORT_MODE_MUX_C
#define CONF_WINC_SPI_NPCS				(1)

/** SPI delay before SPCK and between consecutive transfer. */
#define CONF_WINC_SPI_DLYBS				(0)
#define CONF_WINC_SPI_DLYBCT 			(0)

/** SPI interrupt pin. */
#define CONF_WINC_SPI_INT_PIN			IOPORT_CREATE_PIN(PIOC, 23)
#define CONF_WINC_SPI_INT_PIO			PIOC
#define CONF_WINC_SPI_INT_PIO_ID		ID_PIOC
#define CONF_WINC_SPI_INT_MASK			PIO_PC23
#define CONF_WINC_SPI_INT_PRIORITY		(0)

/** Clock polarity & phase. */
#define CONF_WINC_SPI_POL				(0)
#define CONF_WINC_SPI_PHA				(1)

/** SPI clock: (sysclk_get_cpu_hz() / CONF_WINC_SPI_CLOCK). Beware of integer division. */
#define CONF_WINC_SPI_CLOCK				(48000000)

/*
   ---------------------------------
   --------- Debug Options ---------
   ---------------------------------
*/

#define CONF_WINC_DEBUG					(0)
#define CONF_WINC_PRINTF				//printf

#ifdef __cplusplus
}
#endif

#endif /* CONF_WINC_H_INCLUDED */
