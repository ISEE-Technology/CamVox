/**
 *
 * \file
 *
 * \brief SPI Flash.
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
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

/** @defgroup SPIFLASHAPI SPI FLASH
 */

#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "driver/source/nmbus.h"
#include "spi_flash_map.h"

/** @defgroup SPIFLASHFUNCTIONS Functions
 *  @ingroup SPIFLASHAPI
 */

  /**@{*/
/*!
 * @fn           uint32 spi_flash_get_size(void);
 * @brief        Returns with \ref uint32 value which is total flash size\n
 * @note         Returned value in Mb (Mega Bit).
 * @return       SPI flash size in case of success and a ZERO value in case of failure.
 */
uint32 spi_flash_get_size(void);

/*!
 * @fn             sint8 spi_flash_read(uint8 *, uint32, uint32);
 * @brief          Read a specified portion of data from SPI Flash.\n
 * @param [out]    pu8Buf
 *                 Pointer to data buffer which will be filled with data in case of successful operation.
 * @param [in]     u32Addr
 *                 Address (Offset) to read from at the SPI flash.
 * @param [in]     u32Sz
 *                 Total size of data to be read in bytes
 * @warning	       
 *                 - Address (offset) plus size of data must not exceed flash size.\n
 *                 - No firmware is required for reading from SPI flash.\n
 *                 - In case of there is a running firmware, it is required to pause your firmware first 
 *                   before any trial to access SPI flash to avoid any racing between host and running firmware on bus using 
 *                   @ref m2m_wifi_download_mode
 * @note           
 *                 - It is blocking function\n
 * @sa             m2m_wifi_download_mode, spi_flash_get_size
 * @return        The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
 */
sint8 spi_flash_read(uint8 *pu8Buf, uint32 u32Addr, uint32 u32Sz);

/*!
 * @fn             sint8 spi_flash_write(uint8 *, uint32, uint32);
 * @brief          Write a specified portion of data to SPI Flash.\n
 * @param [in]     pu8Buf
 *                 Pointer to data buffer which contains the data to be written.
 * @param [in]     u32Offset
 *                 Address (Offset) to write at the SPI flash.
 * @param [in]     u32Sz
 *                 Total number of size of data bytes
 * @note           
 *                 - It is blocking function\n
 *                 - It is user's responsibility to verify that data has been written successfully 
 *                   by reading data again and comparing it with the original.
 * @warning	       
 *                 - Address (offset) plus size of data must not exceed flash size.\n
 *                 - No firmware is required for writing to SPI flash.\n
 *                 - In case of there is a running firmware, it is required to pause your firmware first 
 *                   before any trial to access SPI flash to avoid any racing between host and running firmware on bus using 
 *                   @ref m2m_wifi_download_mode.
 *                 - Before writing to any section, it is required to erase that section first.
 * @sa             m2m_wifi_download_mode, spi_flash_get_size, spi_flash_erase
 * @return       The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
 
 */
sint8 spi_flash_write(uint8* pu8Buf, uint32 u32Offset, uint32 u32Sz);

/*!
 * @fn             sint8 spi_flash_erase(uint32, uint32);
 * @brief          Erase a specified portion of SPI Flash.\n
 * @param [in]     u32Offset
 *                 Address (Offset) to erase from the SPI flash.
 * @param [in]     u32Sz
 *                 Total number of bytes required to be erased.
 * @note         It is blocking function \n  
* @warning	       
*                 - Address (offset) plus size of data must not exceed flash size.\n
*                 - No firmware is required for writing to SPI flash.\n
 *                 - In case of there is a running firmware, it is required to pause your firmware first 
 *                   before any trial to access SPI flash to avoid any racing between host and running firmware on bus using 
 *                   @ref m2m_wifi_download_mode
 * @sa             m2m_wifi_download_mode, spi_flash_get_size
 * @return       The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
 *  \section SPIFLASHExample Example
 * @code{.c}
 *					#include "spi_flash/include/spi_flash.h"
 *
 *					#define DATA_TO_REPLACE	"THIS IS A NEW SECTOR IN FLASH"
 *
 *					int main()
 *					{
 *						uint8	au8FlashContent[FLASH_SECTOR_SZ] = {0};
 *						uint32	u32FlashTotalSize = 0;
 *						uint32	u32FlashOffset = 0;
 *	
 *						ret = m2m_wifi_download_mode();
 *						if(M2M_SUCCESS != ret)
 *						{
 *							printf("Unable to enter download mode\r\n");
 *						}
 *						else
 *						{
 *							u32FlashTotalSize = spi_flash_get_size();
 *						}
 *
 *						while((u32FlashTotalSize > u32FlashOffset) && (M2M_SUCCESS == ret))
 *						{
 *							ret = spi_flash_read(au8FlashContent, u32FlashOffset, FLASH_SECTOR_SZ);
 *							if(M2M_SUCCESS != ret)
 *							{
 *								printf("Unable to read SPI sector\r\n");
 *								break;
 *							}
 *							memcpy(au8FlashContent, DATA_TO_REPLACE, strlen(DATA_TO_REPLACE));
 *		
 *							ret = spi_flash_erase(u32FlashOffset, FLASH_SECTOR_SZ);
 *							if(M2M_SUCCESS != ret)
 *							{
 *								printf("Unable to erase SPI sector\r\n");
 *								break;
 *							}
 *		
 *							ret = spi_flash_write(au8FlashContent, u32FlashOffset, FLASH_SECTOR_SZ);
 *							if(M2M_SUCCESS != ret)
 *							{
 *								printf("Unable to write SPI sector\r\n");
 *								break;
 *							}
 *							u32FlashOffset += FLASH_SECTOR_SZ;
 *						}
 *	
 *						if(M2M_SUCCESS == ret)
 *						{
 *							printf("Successful operations\r\n");
 *						}
 *						else
 *						{
 *							printf("Failed operations\r\n");
 *						}
 *	
 *						while(1);
 *						return M2M_SUCCESS;
 *					}
 * @endcode  
 */

sint8 spi_flash_erase(uint32 u32Offset, uint32 u32Sz);


/**@}
 */
 
 
#endif	//__SPI_FLASH_H__