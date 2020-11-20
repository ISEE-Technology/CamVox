/**
 *
 * \file
 *
 * \brief SPI Flash.
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

#ifndef __SPI_FLASH_MAP_H__
#define __SPI_FLASH_MAP_H__

//#define DOWNLOAD_ROLLBACK
//#define OTA_GEN
#define _PROGRAM_POWER_SAVE_
#define BT_IMAGE_PRESENT

/* =======*=======*=======*=======*=======
 * 	  General Sizes for Flash Memory
 * =======*=======*=======*=======*=======
 */

#define FLASH_START_ADDR					(0UL)
/*!<Starting Address of Flash Memory
 *
 */
#define FLASH_BLOCK_SIZE					(32 * 1024UL)
/*!<Block Size in Flash Memory
 */
#define FLASH_SECTOR_SZ						(4 * 1024UL)
/*!<Sector Size in Flash Memory
 */
#define FLASH_PAGE_SZ						(256)
/*!<Page Size in Flash Memory
 */

#define FLASH_2M_TOTAL_SZ					(256 * 1024UL)
/*!<Total Size of 2M Flash Memory
 */
#define FLASH_4M_TOTAL_SZ					(512 * 1024UL)
/*!<Total Size of 4M Flash Memory
 */
#define FLASH_8M_TOTAL_SZ					(1024 * 1024UL)
/*!<Total Size of 8M Flash Memory
 */

/*
 * Detailed Sizes and locations for 8M Flash Memory:
 *  ____________________ ___________ ___________________________ _______________________________________________
 * | Starting Address	|	Size	|	Location's Name			|	Description									|
 * |____________________|___________|___________________________|_______________________________________________|
 * |	  0	K			|	  4	K	|	Boot Firmware			|	Firmware to select which version to run		|
 * |	  4	K 			|	  8	K	|	Control Section			|	Structured data used by Boot firmware		|
 * |	  8	K 			|	  4	K	|	Backup					|	Generic sector backup						|
 * |	 12	K			|	  8	K	|	PLL+GAIN :				|	LookUp Table for PLL and Gain calculations	|
 * |					|			|		PLL  Size = 2K		|		PLL				 						|
 * |					|			|		GAIN Size = 6K		|		Gain configuration				 		|
 * |	 20	K			|	  4	K	|	TLS CERTIFICATE			|	X.509 Root certificate storage				|
 * |	 24	K			|	  8	K	|	TLS Server				|	TLS Server Private Key and certificates		|
 * |	 32	K			|	  4	K	|	Connection Parameters	|	Parameters for success connection to AP		|
 * |	 36	K			|	  8	K	|	Program Firmware		|	Downloader firmware							|
 * |	 44	K			|	296	K 	|	Main Firmware			|	Main Firmware to run WiFi Chip				|
 * |	340	K			|	  8	K	|	HTTP Files				|	Files used with Provisioning Mode			|
 * |	348	K			|	  0 K	|	PS_Firmware				|	Power Save Firmware (Unused)				|
 * |	348	K			|	160	K	|	BT Firmware				|	BT Firmware for BT Cortus					|
 * |	508	K			|	  4	K	|	Host control section	|	Structured data used by Host driver			|
 * |	512	K			|	472	K	|	OTA						|	OTA image									|
 * |					|			|		Program Firmware	|												|
 * |					|			|		Main Firmware		|												|
 * |					|			|		HTTP Files			|												|
 * |					|			|		BT Firmware			|												|
 * |	984	K			|	 40	K	|	Empty					|	Unused										|
 * |____________________|___________|___________________________|_______________________________________________|
 *
 *
 */

/*
 * Boot Firmware: used to select which firmware to run
 */
#define M2M_BOOT_FIRMWARE_STARTING_ADDR		(FLASH_START_ADDR)
#define M2M_BOOT_FIRMWARE_FLASH_SZ			(FLASH_SECTOR_SZ * 1)

/*
 * Control Section: used by Boot firmware
 */
#define M2M_CONTROL_FLASH_OFFSET			(M2M_BOOT_FIRMWARE_STARTING_ADDR + M2M_BOOT_FIRMWARE_FLASH_SZ)
#define M2M_CONTROL_FLASH_SZ				(FLASH_SECTOR_SZ * 1)

/*
 * Generic Sector Backup: used as backup in case of interruption during sector read-erase-write
 */
#define M2M_BACKUP_FLASH_OFFSET				(M2M_CONTROL_FLASH_OFFSET + M2M_CONTROL_FLASH_SZ)
#define M2M_BACKUP_FLASH_SZ					(FLASH_SECTOR_SZ * 1)

/*
 * LUT for PLL and TX Gain settings
 */
#define M2M_PLL_FLASH_OFFSET				(M2M_BACKUP_FLASH_OFFSET + M2M_BACKUP_FLASH_SZ)
#define M2M_PLL_MAGIC_NUMBER_FLASH_SZ		(2*4)		// 2 32bit values
#define M2M_PLL_WIFI_CHAN_FLASH_OFFSET		(M2M_PLL_FLASH_OFFSET + M2M_PLL_MAGIC_NUMBER_FLASH_SZ)
#define M2M_PLL_WIFI_CHAN_FLASH_SZ			(14*8*4)	// Wifi Channels 1 to 14 inclusive, 8 32bit values for each channel.
#define M2M_PLL_FREQ_FLASH_OFFSET			(M2M_PLL_WIFI_CHAN_FLASH_OFFSET + M2M_PLL_WIFI_CHAN_FLASH_SZ)
#define M2M_PLL_FREQ_FLASH_SZ				((1+84)*4)	// Frequencies 2401 to 2484MHz inclusive, also 1920 used for cpll compensate.
#define M2M_PLL_FLASH_SZ					(1024 * 2)
#define M2M_GAIN_FLASH_OFFSET				(M2M_PLL_FLASH_OFFSET + M2M_PLL_FLASH_SZ)
#define M2M_GAIN_FLASH_SZ					(M2M_CONFIG_SECT_TOTAL_SZ - M2M_PLL_FLASH_SZ)
#define M2M_CONFIG_SECT_TOTAL_SZ			(FLASH_SECTOR_SZ * 2)

/*
 * TLS Certificates
 */
#define M2M_TLS_ROOTCER_FLASH_OFFSET		(M2M_PLL_FLASH_OFFSET + M2M_CONFIG_SECT_TOTAL_SZ)
#define M2M_TLS_ROOTCER_FLASH_SZ			(FLASH_SECTOR_SZ * 1)
#define M2M_TLS_ROOTCER_FLASH_SIG			{0x01,0xF1,0x02,0xF2,0x03,0xF3,0x04,0xF4,0x05,0xF5,0x06,0xF6,0x07,0xF7,0x08,0xF8}
#define M2M_TLS_ROOTCER_FLASH_SIG_LENGTH	16
/*!<
*/

/*
 * TLS Server Key Files
 */
#define M2M_TLS_SERVER_FLASH_OFFSET			(M2M_TLS_ROOTCER_FLASH_OFFSET + M2M_TLS_ROOTCER_FLASH_SZ)
#define M2M_TLS_SERVER_FLASH_SZ				(FLASH_SECTOR_SZ * 2)
#define M2M_TLS_SERVER_FLASH_SIG

/*
 * Saved Connection Parameters
 */
#define M2M_CACHED_CONNS_FLASH_OFFSET		(M2M_TLS_SERVER_FLASH_OFFSET + M2M_TLS_SERVER_FLASH_SZ)
#define M2M_CACHED_CONNS_FLASH_SZ			(FLASH_SECTOR_SZ * 1)
#define M2M_CACHED_CONNS_FLASH_SIG

/*
 *
 * OTA image1 Offset (Firmware offset)
 */
#define M2M_OTA_IMAGE1_OFFSET				(M2M_CACHED_CONNS_FLASH_OFFSET + M2M_CACHED_CONNS_FLASH_SZ)

/*
 * Common section size
 */
#define FLASH_COMMON_SZ (\
		M2M_BOOT_FIRMWARE_FLASH_SZ		+\
		M2M_CONTROL_FLASH_SZ			+\
		M2M_BACKUP_FLASH_SZ				+\
		M2M_CONFIG_SECT_TOTAL_SZ		+\
		M2M_TLS_ROOTCER_FLASH_SZ		+\
		M2M_TLS_SERVER_FLASH_SZ			+\
		M2M_CACHED_CONNS_FLASH_SZ		)

/*
 * Check addition
 */
#if (FLASH_COMMON_SZ != M2M_OTA_IMAGE1_OFFSET)
#error "Common Size Mismatch"
#endif

/*
 * Host Control Section
 */
#define HOST_CONTROL_FLASH_OFFSET			(M2M_OTA_IMAGE1_OFFSET + OTA_IMAGE_SIZE)
#define HOST_CONTROL_FLASH_SZ				(FLASH_SECTOR_SZ * 1)
#define HOST_CONTROL_FLASH_SIG				0x414f5354

/**
 *
 * OTA image 2 offset
 */
#define M2M_OTA_IMAGE2_OFFSET				(FLASH_4M_TOTAL_SZ)

/*
 * Firmware (including downloader firmware)
 */
#if (defined _FIRMWARE_)||(defined OTA_GEN)
#define M2M_FIRMWARE_FLASH_OFFSET			(0UL)
#else
#if (defined DOWNLOAD_ROLLBACK)
#define M2M_FIRMWARE_FLASH_OFFSET			(M2M_OTA_IMAGE2_OFFSET)
#else
#define M2M_FIRMWARE_FLASH_OFFSET			(M2M_OTA_IMAGE1_OFFSET)
#endif
#endif
#ifdef _PROGRAM_POWER_SAVE_
#define M2M_PROGRAM_FLASH_SZ				(8 * 1024UL)	/* downloader firmware */
#else
#define M2M_PROGRAM_FLASH_SZ				(0UL)
#endif	/* _PROGRAM_POWER_SAVE_ */
#define M2M_FIRMWARE_FLASH_SZ				(304 * 1024UL)	/* downloader firmware and main firmware */

/*
 * HTTP Files
 */
#define M2M_HTTP_MEM_FLASH_OFFSET			(M2M_FIRMWARE_FLASH_OFFSET + M2M_FIRMWARE_FLASH_SZ)
#define M2M_HTTP_MEM_FLASH_SZ				(FLASH_SECTOR_SZ * 2)

/*
 * ps_Firmware(Power Save Firmware): App. which runs for power saving purpose
 */
#define M2M_PS_FIRMWARE_FLASH_OFFSET		(M2M_HTTP_MEM_FLASH_OFFSET + M2M_HTTP_MEM_FLASH_SZ)
#define M2M_PS_FIRMWARE_FLASH_SZ			(FLASH_SECTOR_SZ * 0)

/*
 * BT(Bluetooth Firmware): BT/BLE firmware to run on the BT cortus
 */
#define M2M_BT_FIRMWARE_FLASH_OFFSET		(M2M_PS_FIRMWARE_FLASH_OFFSET + M2M_PS_FIRMWARE_FLASH_SZ)
#ifdef BT_IMAGE_PRESENT
#define M2M_BT_FIRMWARE_FLASH_SZ			(160 * 1024UL)
#else
#define M2M_BT_FIRMWARE_FLASH_SZ			(0)
#endif


/*
 * OTA image size
 */
#define OTA_IMAGE_SIZE (\
		M2M_FIRMWARE_FLASH_SZ		+\
		M2M_HTTP_MEM_FLASH_SZ		+\
		M2M_PS_FIRMWARE_FLASH_SZ	+\
		M2M_BT_FIRMWARE_FLASH_SZ	)

/*
 * Check addition
 */
#if ((M2M_FIRMWARE_FLASH_OFFSET + OTA_IMAGE_SIZE) != (M2M_BT_FIRMWARE_FLASH_OFFSET + M2M_BT_FIRMWARE_FLASH_SZ))
#error "OTA Size Mismatch"
#endif


/*
 * Check that total size of content
 * does not exceed total size of memory allocated
 */
#if ((FLASH_COMMON_SZ + OTA_IMAGE_SIZE + HOST_CONTROL_FLASH_SZ) > M2M_OTA_IMAGE2_OFFSET)
#error "Exceed Flash Size"
#endif /* ((FLASH_COMMON_SZ + OTA_IMAGE_SIZE + HOST_CONTROL_FLASH_SZ) > M2M_OTA_IMAGE2_OFFSET) */
#if ((M2M_OTA_IMAGE2_OFFSET + OTA_IMAGE_SIZE) > FLASH_8M_TOTAL_SZ)
#error "OTA Exceed Flash Size"
#endif /* ((M2M_OTA_IMAGE2_OFFSET + OTA_IMAGE_SIZE) > FLASH_8M_TOTAL_SZ) */

#endif /* __SPI_FLASH_MAP_H__ */
