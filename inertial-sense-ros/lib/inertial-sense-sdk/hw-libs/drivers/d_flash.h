/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef D_FLASH_H_
#define D_FLASH_H_
#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_GPNVM_SECURITY_BIT  0
#define FLASH_GPNVM_BOOTMODE_BIT  1 // clear:ROM, set:Flash

// uINS-3 flash layout - uINS Flash Memory Map at top of bootloaderShared.h

// write data at the given 8KB aligned flash address
// address the 8KB aligned flash address to write at
// newData is the new bytes to put in the block starting at offset - must be exactly 8K (flash block size) in size
// return FLASH_RC_OK if success, otherwise error code
uint32_t flash_update_block(uint32_t address, const void* newData, int dataSize, int noPageErase);

// erase the flash block at the 8K aligned address
uint32_t flash_erase_block(uint32_t address);

// Erase chip - Since it is a RAM function, it needs 'extern' declaration.
extern void flash_erase_chip(void);

// read the user signature, size must be multiple of 4 and less than or equal to 512
uint32_t flash_get_user_signature(volatile void* ptr, uint32_t size);

// write the user signature, size must be multiple of 4 and less than or equal to 512
uint32_t flash_set_user_signature(const volatile void* ptr, uint32_t size);

// is a flash write in progress? 0 = no, 1 = yes
int flash_write_in_progress(void);

#ifdef __cplusplus
}
#endif
#endif // D_FLASH_H_
