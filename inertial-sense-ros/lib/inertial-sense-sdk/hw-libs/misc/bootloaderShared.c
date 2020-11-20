#include "bootloaderShared.h"
#include <string.h>

#define BOOTLOADER_HASH_MIXER 0x5bd1e995
#define BOOTLOADER_HASH_SHIFTER 24

// calculate bootloader hash code over a set of data
uint32_t calculateBootloaderHashCode(uint32_t hashCode, const uint32_t* start, const uint32_t* end)
{
	uint32_t value;
	while (start < end)
	{
		value = *start++;
		value *= BOOTLOADER_HASH_MIXER;
		value ^= value >> BOOTLOADER_HASH_SHIFTER;
		value *= BOOTLOADER_HASH_MIXER;
		hashCode *= BOOTLOADER_HASH_MIXER;
		hashCode ^= value;
	}
	return hashCode;
}