/**
 *
 * \file
 *
 * \brief This module contains WINC3400 M2M driver APIs implementation.
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

#include "common/include/nm_common.h"
#include "driver/source/nmbus.h"
#include "bsp/include/nm_bsp.h"
#include "driver/source/nmdrv.h"
#include "driver/source/nmasic.h"
#include "driver/include/m2m_types.h"

#ifdef CONF_WINC_USE_SPI
#include "driver/source/nmspi.h"
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
sint8 nm_get_hif_info(uint16 *pu16FwHifInfo, uint16 *pu16OtaHifInfo)
{
	sint8 ret = M2M_SUCCESS;
	uint32 reg = 0;

	ret = nm_read_reg_with_ret(NMI_REV_REG, &reg);
	if(ret == M2M_SUCCESS)
	{
		if(pu16FwHifInfo != NULL)
		{
			*pu16FwHifInfo = (uint16)reg;
		}
		if(pu16OtaHifInfo)
		{
			*pu16OtaHifInfo = (uint16)(reg>>16);
		}
	}
	return ret;
}
/**
*	@fn		nm_get_firmware_full_info(tstrM2mRev* M2mRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*				pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
*	@version	1.0
*/
sint8 nm_get_firmware_full_info(tstrM2mRev* pstrRev)
{
	uint16  fw_hif_info = 0;
	uint32	reg = 0;
	sint8	ret = M2M_SUCCESS;
	tstrGpRegs strgp = {0};

	m2m_memset((uint8*)pstrRev,0,sizeof(tstrM2mRev));
	nm_get_hif_info(&fw_hif_info, NULL);

	M2M_INFO("Fw HIF: %04x\n", fw_hif_info);
	if(M2M_GET_HIF_BLOCK(fw_hif_info) == M2M_HIF_BLOCK_VALUE)
	{
		ret = nm_read_reg_with_ret(rNMI_GP_REG_0, &reg);
		if(ret == M2M_SUCCESS)
		{
			if(reg != 0)
			{
				ret = nm_read_block(reg|0x30000,(uint8*)&strgp,sizeof(tstrGpRegs));
				if(ret == M2M_SUCCESS)
				{
					reg = strgp.u32Firmware_Ota_rev;
					reg &= 0x0000ffff;
					if(reg != 0)
					{
						ret = nm_read_block(reg|0x30000,(uint8*)pstrRev,sizeof(tstrM2mRev));
						if(ret == M2M_SUCCESS)
						{
							M2M_INFO("Firmware HIF (%u) : %u.%u \n", M2M_GET_HIF_BLOCK(pstrRev->u16FirmwareHifInfo), M2M_GET_HIF_MAJOR(pstrRev->u16FirmwareHifInfo), M2M_GET_HIF_MINOR(pstrRev->u16FirmwareHifInfo));
							M2M_INFO("Firmware ver   : %u.%u.%u \n", pstrRev->u8FirmwareMajor, pstrRev->u8FirmwareMinor, pstrRev->u8FirmwarePatch);
							M2M_INFO("Firmware Build %s Time %s\n", pstrRev->BuildDate, pstrRev->BuildTime);

							/* Check Hif info is consistent */
							if(fw_hif_info != pstrRev->u16FirmwareHifInfo)
							{
								ret = M2M_ERR_FAIL;
								M2M_ERR("Inconsistent Firmware Version\n");
							}
						}
					}
					else
					{
						ret = M2M_ERR_FAIL;
					}
				}
			}
			else
			{
				ret = M2M_ERR_FAIL;
			}
		}
	}
	else
	{
		ret = M2M_ERR_FAIL;
	}
	if(ret != M2M_SUCCESS)
	{
		M2M_ERR("Unknown Firmware Version\n");
	}
	return ret;
}
/**
*	@fn		nm_get_ota_firmware_info(tstrM2mRev* pstrRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*			    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
			
*	@version	1.0
*/
sint8 nm_get_ota_firmware_info(tstrM2mRev* pstrRev)
{
	uint16  ota_hif_info = 0;
	uint32	reg = 0;
	sint8	ret = M2M_SUCCESS;
	tstrGpRegs strgp = {0};

	m2m_memset((uint8*)pstrRev,0,sizeof(tstrM2mRev));
	nm_get_hif_info(NULL, &ota_hif_info);

	M2M_INFO("Ota HIF: %04x\n", ota_hif_info);
	if(M2M_GET_HIF_BLOCK(ota_hif_info) == M2M_HIF_BLOCK_VALUE)
	{
		ret = nm_read_reg_with_ret(rNMI_GP_REG_0, &reg);
		if(ret == M2M_SUCCESS)
		{
			if(reg != 0)
			{
				ret = nm_read_block(reg|0x30000,(uint8*)&strgp,sizeof(tstrGpRegs));
				if(ret == M2M_SUCCESS)
				{
					reg = strgp.u32Firmware_Ota_rev;
					reg >>= 16;
					if(reg != 0)
					{
						ret = nm_read_block(reg|0x30000,(uint8*)pstrRev,sizeof(tstrM2mRev));
						if(ret == M2M_SUCCESS)
						{
							M2M_INFO("OTA HIF (%u) : %u.%u \n", M2M_GET_HIF_BLOCK(pstrRev->u16FirmwareHifInfo), M2M_GET_HIF_MAJOR(pstrRev->u16FirmwareHifInfo), M2M_GET_HIF_MINOR(pstrRev->u16FirmwareHifInfo));
							M2M_INFO("OTA ver   : %u.%u.%u \n", pstrRev->u8FirmwareMajor, pstrRev->u8FirmwareMinor, pstrRev->u8FirmwarePatch);
							M2M_INFO("OTA Build %s Time %s\n", pstrRev->BuildDate, pstrRev->BuildTime);

							/* Check Hif info is consistent */
							if(ota_hif_info != pstrRev->u16FirmwareHifInfo)
							{
								ret = M2M_ERR_FAIL;
								M2M_ERR("Inconsistent OTA Version\n");
							}
						}
					}
					else
					{
						ret = M2M_ERR_FAIL;
					}
				}
			}
			else
			{
				ret = M2M_ERR_FAIL;
			}
		}
	}
	else
	{
		ret = M2M_ERR_FAIL;
	}
	if(ret != M2M_SUCCESS)
	{
		M2M_INFO("No valid Ota image\n");
	}
	return ret;
}

/*
*	@fn		nm_drv_init_download_mode
*	@brief	Initialize NMC1000 driver
*	@return	M2M_SUCCESS in case of success and Negative error code in case of failure
*   @param [in]	arg
*				Generic argument
*	@author	Viswanathan Murugesan
*	@date	10 Oct 2014
*	@version	1.0
*/
sint8 nm_drv_init_download_mode(uint32 req_serial_number)
{
	sint8 ret = M2M_SUCCESS;
	
	ret = nm_bus_iface_init(NULL, req_serial_number);
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi start]: fail init bus\n");
		goto ERR1;
	}


#ifdef CONF_WINC_USE_SPI
	/* Must do this after global reset to set SPI data packet size. */
	nm_spi_init();
#endif

	M2M_INFO("Chip ID %lx\n", nmi_get_chipid());
	
	/*disable all interrupt in ROM (to disable uart) in 2b0 chip*/
	nm_write_reg(0x20300,0);

ERR1:
	return ret;
}

sint8 nm_drv_init_hold(uint32 req_serial_number)
{
	sint8 ret = M2M_SUCCESS;

	ret = nm_bus_iface_init(NULL, req_serial_number);
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi start]: fail init bus\n");
		goto ERR1;
	}

#ifdef BUS_ONLY
	return;
#endif
	
	ret = chip_wake();
	nm_bsp_sleep(10);
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi start]: fail chip_wakeup\n");
		goto ERR2;
	}
	/**
	Go...
	**/
	ret = chip_reset();
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}
	M2M_INFO("Chip ID %lx\n", nmi_get_chipid());
#ifdef CONF_WINC_USE_SPI
	/* Must do this after global reset to set SPI data packet size. */
	nm_spi_init();
#endif
	/*return power save to default value*/
	chip_idle();

	return ret;
ERR2:
	nm_bus_iface_deinit();
ERR1:
	return ret;
}

sint8 nm_drv_init_start(void * arg)
{
	sint8 ret = M2M_SUCCESS;
	uint8 u8Mode = M2M_WIFI_MODE_NORMAL;
	
	if(NULL != arg) {
		if(M2M_WIFI_MODE_CONFIG == *((uint8 *)arg)) {
			u8Mode = M2M_WIFI_MODE_CONFIG;
			} else {
			/*continue running*/
		}
		} else {
		/*continue running*/
	}

	ret = cpu_start();
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}
	ret = wait_for_bootrom(u8Mode);
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}
		
	ret = wait_for_firmware_start(u8Mode);
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}
	
	if(M2M_WIFI_MODE_CONFIG == u8Mode) {
		goto ERR1;
	} else {
		/*continue running*/
	}
	
	ret = enable_interrupts();
	if (M2M_SUCCESS != ret) {
		M2M_ERR("failed to enable interrupts..\n");
		goto ERR2;
	}
	
	return ret;
ERR2:
	nm_bus_iface_deinit();
#ifdef CONF_WINC_USE_SPI
	nm_spi_deinit();
#endif
ERR1:	
	return ret;
}

/*
*	@fn		nm_drv_init
*	@brief	Initialize NMC1000 driver
*	@return	M2M_SUCCESS in case of success and Negative error code in case of failure
*   @param [in]	arg - Generic argument passed on to nm_drv_init_start
*	@author	M. Abdelmawla
*	@date	15 July 2012
*	@version	1.0
*/
sint8 nm_drv_init(void * arg, uint32 req_serial_number)
{
	sint8 ret = M2M_SUCCESS;

	ret = nm_drv_init_hold(req_serial_number);

	if(ret == M2M_SUCCESS)
		ret = nm_drv_init_start(arg);

	return ret;
}

/*
*	@fn		nm_drv_deinit
*	@brief	Deinitialize NMC1000 driver
*	@author	M. Abdelmawla
*	@date	17 July 2012
*	@version	1.0
*/
sint8 nm_drv_deinit(void * arg) 
{
	sint8 ret;
	
	ret = chip_deinit();
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi stop]: chip_deinit fail\n");
		goto ERR1;
	}

	ret = nm_bus_iface_deinit();
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi stop]: fail init bus\n");
		goto ERR1;
	}
#ifdef CONF_WINC_USE_SPI
	/* Must do this after global reset to set SPI data packet size. */
	nm_spi_deinit();
#endif

ERR1:
	return ret;
}


/**
*	@fn		nm_cpu_start(void)
*	@brief	Start CPU from the WINC module	
*	@return	ZERO in case of success and Negative error code in case of failure
*/

sint8 nm_cpu_start(void)
{
	sint8 ret;

	ret = cpu_start();
	return ret;
}