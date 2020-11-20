/**
 *
 * \file
 *
 * \brief This module contains WINC3400 M2M driver APIs declarations.
 *
 * Copyright (c) 2017-2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef _NMDRV_H_
#define _NMDRV_H_

#include "common/include/nm_common.h"

/**
*  @struct		tstrM2mRev
*  @brief		Structure holding firmware version parameters and build date/time
*/
typedef struct {
	uint16 u16FirmwareHifInfo; /* Fw HIF Info */
	uint8 u8FirmwareMajor; /* Version Major Number */
	uint8 u8FirmwareRsvd; /* Reserved */
	uint8 u8FirmwareMinor; /* Version Minor */
	uint8 u8FirmwarePatch; /* Patch Number */
	uint8 BuildDate[sizeof(__DATE__)]; // 12 bytes
	uint8 BuildTime[sizeof(__TIME__)]; // 9 bytes
} tstrM2mRev;

#ifdef __cplusplus
     extern "C" {
 #endif
/**
*	@fn		nm_get_hif_info(uint16 *pu16FwHifInfo, uint16 *pu16OtaHifInfo);
*	@brief	Get Hif info of images in both partitions (Firmware and Ota).
*	@param [out]	pu16FwHifInfo
*					Pointer holding Hif info of image in the active partition.
*	@param [out]	pu16OtaHifInfo
*					Pointer holding Hif info of image in the inactive partition.
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_get_hif_info(uint16 *pu16FwHifInfo, uint16 *pu16OtaHifInfo);
/**
*	@fn		nm_get_firmware_full_info(tstrM2mRev* pstrRev)
*	@brief	Get Firmware version info
*	@param [out]	pstrRev
*					Pointer holds address of structure @ref tstrM2mRev that contains the version parameters
*					of image in the active partition.
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_get_firmware_full_info(tstrM2mRev* pstrRev);
/**
*	@fn		nm_get_ota_firmware_info(tstrM2mRev* pstrRev)
*	@brief	Get Firmware version info
*	@param [out]	pstrRev
*					Pointer holds address of structure @ref tstrM2mRev that contains the version parameters
*					of image in the inactive partition.
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_get_ota_firmware_info(tstrM2mRev* pstrRev);
/*
*	@fn		nm_drv_init
*	@brief	Initialize NMC1000 driver
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_drv_init_download_mode(uint32 req_serial_number);

/*
*	@fn		nm_drv_init
*	@brief	Initialize NMC1000 driver
*	@return	M2M_SUCCESS in case of success and Negative error code in case of failure
*	@param [in]	arg - Generic argument passed on to nm_drv_init_start
*	@param [in]	req_serial_number
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_drv_init(void * arg, uint32 req_serial_number);

/*
*	@fn		nm_drv_init_hold
*	@brief	First part of nm_drv_init, up to the point of initializing spi for flash access.
*	@see	nm_drv_init
*	@return	M2M_SUCCESS in case of success and Negative error code in case of failure
*	@param [in]	req_serial_number
*				Parameter inherited from nm_drv_init
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_drv_init_hold(uint32 req_serial_number);

/*
*	@fn		nm_drv_init_start
*	@brief	Second part of nm_drv_init, continuing from where nm_drv_init_hold left off.
*	@see	nm_drv_init
*	@return	M2M_SUCCESS in case of success and Negative error code in case of failure
*	@param [in]	arg
*				Parameter inherited from nm_drv_init
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_drv_init_start(void * arg);

/**
*	@fn		nm_drv_deinit
*	@brief	Deinitialize NMC1000 driver
*	@author	M. Abdelmawla
*   @param [in]	arg - Generic argument unused.
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_drv_deinit(void * arg);

/**
*	@fn		nm_cpu_start(void)
*	@brief	Start CPU from the WINC module	
*	@return	ZERO in case of success and Negative error code in case of failure
*/

sint8 nm_cpu_start(void);

#ifdef __cplusplus
	 }
 #endif

#endif	/*_NMDRV_H_*/


