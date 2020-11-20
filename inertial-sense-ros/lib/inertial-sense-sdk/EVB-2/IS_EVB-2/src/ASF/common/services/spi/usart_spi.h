/**
 * \file
 *
 * \brief USART in SPI mode driver functions.
 *
 * Copyright (c) 2010-2018 Microchip Technology Inc. and its subsidiaries.
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
#ifndef USART_SPI_H_INCLUDED
#define USART_SPI_H_INCLUDED

#include <parts.h>

#if XMEGA
# include "xmega_usart_spi/usart_spi.h"
#elif MEGA_RF
# include "megarf_usart_spi/usart_spi.h"
#elif UC3
# include "uc3_usart_spi/usart_spi.h"
#elif SAM
# include "sam_usart_spi/usart_spi.h"
#else
# error Unsupported chip type
#endif

/**
 *
 * \defgroup usart_spi_group USART in SPI (Serial Peripheral Interface) mode
 *
 * This is the common API for USART in SPI mode. Additional features are available
 * in the documentation of the specific modules.
 *
 * \section spi_group_platform Platform Dependencies
 *
 * The spi API is partially chip- or platform-specific. While all
 * platforms provide mostly the same functionality, there are some
 * variations around how different bus types and clock tree structures
 * are handled.
 *
 * The following functions are available on all platforms, but there may
 * be variations in the function signature (i.e. parameters) and
 * behaviour. These functions are typically called by platform-specific
 * parts of drivers, and applications that aren't intended to be
 * portable:
 *   - usart_spi_init()
 *   - usart_spi_setup_device()
 *   - usart_spi_select_device()
 *   - usart_spi_deselect_device()
 *   - usart_spi_write_single()
 *   - usart_spi_write_packet()
 *   - usart_spi_read_single()
 *   - usart_spi_read_packet()
 *   - usart_spi_is_tx_empty()
 *   - usart_spi_is_tx_ready()
 *   - usart_spi_is_rx_full()
 *   - usart_spi_is_rx_ready()
 *   - usart_spi_enable()
 *   - usart_spi_disable()
 *   - usart_spi_is_enabled()
 *
 *
 * @{
 */

//! @}

#endif /* USART_SPI_H_INCLUDED */
