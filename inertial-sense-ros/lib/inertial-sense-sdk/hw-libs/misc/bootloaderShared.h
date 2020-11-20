#ifndef __BOOTLOADER_SHARED_H__
#define __BOOTLOADER_SHARED_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// uINS-3 flash layout - uINS Flash Memory Map
/*
  page size:           512  (0x200)
  block size:          8192 (0x2000)
  flash base address:  0x00400000
  flash size:          0x00100000

    use                offset     flash address
  ------------------------------------------------------------
  -------------------- 0x00000000 (0x00400000)
  | bootloader (15872 bytes + 512 bootloader header bytes)
  -------------------- 0x00004000 (0x00404000)
  | application (966656 bytes) - application + bootloader = 983040 bytes
  -------------------- 0x000F0000 (0x004F0000)
  | 40K: unclaimed (40960 bytes)
  -------------------- 0x000FA000 (0x004FA000)
  | 8K : config mirror (8192 bytes)
  -------------------- 0x000FC000 (0x004FC000)
  | 8K : calibration (8192 bytes)
  -------------------- 0x000FE000 (0x00FE0000)
  | 8K : config (user page 0, user page 1, flash config, internal config, 8192 bytes)
  | Note: Last 4 bytes of flash is the migration marker, currently 0x01010101)
  -------------------- 0x00100000 (0x00500000)
*/

// #define CONF_UART_BAUDRATE_LEGACY   2000000
#define CONF_UART_BAUDRATE          921600
#define CONF_UART_BAUDRATE_RS232    230400
#define CONF_UART_BAUDRATE_SLOW     115200

// total size of all flash memory
#ifndef BOOTLOADER_FLASH_TOTAL_SIZE
#define BOOTLOADER_FLASH_TOTAL_SIZE ((uint32_t)1048576)
#endif

// start address of bootloader in flash memory
#ifndef BOOTLOADER_FLASH_START_ADDRESS
#define BOOTLOADER_FLASH_START_ADDRESS ((uint32_t)0x00400000)
#endif

// total space allocated to bootloader in flash memory
#ifndef BOOTLOADER_FLASH_BOOTLOADER_SIZE
#define BOOTLOADER_FLASH_BOOTLOADER_SIZE ((uint32_t)16384) // 16K
#endif

// size of the user application in flash memory
#ifndef BOOTLOADER_FLASH_USER_APPLICATION_SIZE
#define BOOTLOADER_FLASH_USER_APPLICATION_SIZE ((uint32_t)966656) // 966656 = EC000, 1MB flash - 16K bootloader - 64K footer
#endif

// note: the default is accurate for user flash memory, but sectors are smaller in the bootloader and start of application space
#ifndef BOOTLOADER_FLASH_SECTOR_SIZE
#define BOOTLOADER_FLASH_SECTOR_SIZE ((uint32_t)131072) // 128 KB
#endif

// size of flash page
#ifndef BOOTLOADER_FLASH_PAGE_SIZE
#define BOOTLOADER_FLASH_PAGE_SIZE ((uint32_t)512)
#endif

// size of flash block
#ifndef BOOTLOADER_FLASH_BLOCK_SIZE
#define BOOTLOADER_FLASH_BLOCK_SIZE ((uint32_t)8192) // 8K
#endif

// size of the bootloader header
#ifndef BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE
#define BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE BOOTLOADER_FLASH_PAGE_SIZE
#endif

// offset of bootloader header in flash memory
#ifndef BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS
#define BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS (BOOTLOADER_FLASH_START_ADDRESS + BOOTLOADER_FLASH_BOOTLOADER_SIZE - BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE)
#endif

// start offset in flash of config mirror data
#ifndef BOOTLOADER_FLASH_CONFIG_MIRROR_BASE_ADDRESS
#define BOOTLOADER_FLASH_CONFIG_MIRROR_BASE_ADDRESS (BOOTLOADER_FLASH_START_ADDRESS + 0x000FA000)
#endif

// start offset in flash of calibration data
#ifndef BOOTLOADER_FLASH_CALIB_BASE_ADDRESS
#define BOOTLOADER_FLASH_CALIB_BASE_ADDRESS (BOOTLOADER_FLASH_START_ADDRESS + 0x000FC000)
#endif

// start offset in flash of config data
#ifndef BOOTLOADER_FLASH_CONFIG_BASE_ADDRESS
#define BOOTLOADER_FLASH_CONFIG_BASE_ADDRESS (BOOTLOADER_FLASH_START_ADDRESS + 0x000FE000)
#endif

// start of user application in flash memory
#ifndef BOOTLOADER_FLASH_USER_APPLICATION_START_ADDRESS
#define BOOTLOADER_FLASH_USER_APPLICATION_START_ADDRESS (BOOTLOADER_FLASH_START_ADDRESS + BOOTLOADER_FLASH_BOOTLOADER_SIZE)
#endif

// start of user data in flash memory
#ifndef BOOTLOADER_FLASH_USER_DATA_START_ADDRESS
#define BOOTLOADER_FLASH_USER_DATA_START_ADDRESS (BOOTLOADER_FLASH_START_ADDRESS + BOOTLOADER_FLASH_BOOTLOADER_SIZE + BOOTLOADER_FLASH_USER_APPLICATION_SIZE)
#endif

// size of config in flash memory
#ifndef BOOTLOADER_FLASH_USER_DATA_SIZE
#define BOOTLOADER_FLASH_USER_DATA_SIZE (BOOTLOADER_FLASH_TOTAL_SIZE - BOOTLOADER_FLASH_USER_APPLICATION_SIZE - BOOTLOADER_FLASH_BOOTLOADER_SIZE)
#endif

// end address of user data in flash (exclusive)
#ifndef BOOTLOADER_FLASH_USER_DATA_END_ADDRESS
#define BOOTLOADER_FLASH_USER_DATA_END_ADDRESS (BOOTLOADER_FLASH_USER_DATA_START_ADDRESS + BOOTLOADER_FLASH_USER_DATA_SIZE)
#endif

// the last address in flash, EXCLUSIVE!!! - subtract 1 to get the last valid byte to read or write to
#define BOOTLOADER_FLASH_END_ADDRESS (BOOTLOADER_FLASH_START_ADDRESS + BOOTLOADER_FLASH_TOTAL_SIZE)

// start seed of bootloader hash
#define BOOTLOADER_HASH_CODE_START_VALUE ((uint32_t)435258227)

// jump signature to stay in bootloader - DO NOT MODIFY!
#define BOOTLOADER_JUMP_SIGNATURE_STAY_IN_BOOTLOADER       "__StayInBootLoader12345__"

// jump signature to stay in user application - DO NOT MODIFY!
#define BOOTLOADER_JUMP_SIGNATURE_STAY_IN_USER_APPLICATION "__StayInUserApplication__"

// jump signature size
#define BOOTLOADER_JUMP_SIGNATURE_SIZE ((uint32_t)32)

// signature size
#define BOOTLOADER_SIGNATURE_SIZE ((uint32_t)16)

// bootloader signature must be found within this number of bytes, or the bootload fails
#define BOOTLOADER_SIGNATURE_REQUIRED_WITHIN_BYTE_COUNT ((uint32_t)16384)

// signature found marker - this is the only valid value to indicate that the signature was found in the firmware
#define BOOTLOADER_SIGNATURE_FOUND_MARKER ((uint32_t)1)

// the size of each logical page size in the bootloader - 64K, this does not match the flash page size and is based on the fact that bootloader logic uses 16 bit unsigned int for page offsets
#define BOOTLOADER_LOGICAL_PAGE_SIZE ((uint32_t)65536)

// bootloader sets all bytes in header to this value on deploy
#define BOOTLOADER_HEADER_INITIAL_FILL_BYTE ((uint8_t)0xFE)

// bootloader sets all bytes in header to this value on deploy
#define BOOTLOADER_HEADER_INITIAL_FILL_UINT_32 ((uint32_t)0xFEFEFEFE)

// Serial port selection key
#define PORT_SEL_KEY_SYS_GPBR_3		0x09ea4f06
#define PORT_SEL_KEY_SYS_GPBR_4		0x13d6007e
#define PORT_SEL_KEY_SYS_GPBR_5		0x93f035fe
#define PORT_SEL_KEY_SYS_GPBR_6		0xd096ae0f

#define PORT_SEL_SER0	0
#define PORT_SEL_SER1	1
#define PORT_SEL_USB	2

typedef union
{
	struct  
	{
		char jumpSignature[BOOTLOADER_JUMP_SIGNATURE_SIZE]; // indicates whether to stay in bootloader or user application
		uint32_t signatureFound; // was a signature found, must be exactly BOOTLOADER_SIGNATURE_FOUND_MARKER to be true, no other value is true
		uint32_t hashCode; // hash code for user application
		char version[4]; // bootloader version
	} data;
	uint8_t bytes[BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE]; // any additional bytes are reserved space
} bootloader_header_t;

// calculate bootloader hash code over a set of data - hashCode should be BOOTLOADER_HASH_CODE_START_VALUE if just starting, otherwise previous value if continuing
uint32_t calculateBootloaderHashCode(uint32_t hashCode, const uint32_t* start, const uint32_t* end);

#ifdef __cplusplus
}
#endif
#endif  // __BOOTLOADER_SHARED_H__
