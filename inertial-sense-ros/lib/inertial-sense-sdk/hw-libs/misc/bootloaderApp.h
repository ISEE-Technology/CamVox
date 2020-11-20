/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __BOOTLOADER_APP_H__
#define __BOOTLOADER_APP_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "bootloaderShared.h"

void unlockUserFlash(void);
void soft_reset_no_backup_register(void); // soft reset without setting backup register
void soft_reset_backup_register(uint32_t key); // soft reset and set backup register to a value
void set_reset_pin_enabled(int enabled); // enabled is 0 or 1
void enable_bootloader(int pHandle);
void enable_bootloader_assistant(void);


#ifdef __cplusplus
}
#endif
#endif  // __BOOTLOADER_APP_H__
