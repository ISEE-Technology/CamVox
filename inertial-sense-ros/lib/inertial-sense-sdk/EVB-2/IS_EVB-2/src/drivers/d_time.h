/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _D_TIME_H_
#define _D_TIME_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// defines
//#define ENABLE_TC_TIME

// includes
// #include "user_board.h" // BOARD_FREQ_SLCK_XTAL
#ifdef ENABLE_TC_TIME
#include "d_timer.h"
#endif // ENABLE_TC_TIME

// defines
#ifdef ENABLE_TC_TIME
#define RTPRES_SEC              (CHIP_FREQ_MCK/8) // MCK/8
#define RTPRES                  1
#else
#define RTPRES_SEC              BOARD_FREQ_SLCK_XTAL
#define RTPRES                  3 // finest allowed granularity
#endif // ENABLE_TC_TIME

#define TIME_TICKS_PER_SEC      (RTPRES_SEC/RTPRES)
#define TIME_TICKS_PER_MS       (TIME_TICKS_PER_SEC/1000)
#define TIME_TICKS_PER_US       (TIME_TICKS_PER_SEC/1000000)

#define TIME_SECS_PER_TICK_LF   (((double)RTPRES)/(double)RTPRES_SEC)
#define TIME_MS_PER_TICK_LF     (TIME_SECS_PER_TICK_LF*1000.0)
#define TIME_US_PER_TICK_LF     (TIME_SECS_PER_TICK_LF*1000000.0)

#define TIME_SECS_PER_TICK_F    (((float)RTPRES)/(float)RTPRES_SEC)
#define TIME_MS_PER_TICK_F      (TIME_SECS_PER_TICK_F*1000.0f)
#define TIME_US_PER_TICK_F      (TIME_SECS_PER_TICK_F*1000000.0f)

// typedefs
typedef union
{
	uint32_t u32[2];
	uint64_t u64;
} ticks_t;

// prototypes
void time_init(void);
volatile uint64_t time_ticks(void);
void time_delay(uint32_t ms);
uint32_t time_msec(void);
uint32_t time_usec(void);
float time_secf(void);
float time_msecf(void);
float time_usecf(void);
double time_seclf(void);
double time_mseclf(void);
double time_useclf(void);

#ifdef __cplusplus
}
#endif

#endif // _D_TIME_H_
