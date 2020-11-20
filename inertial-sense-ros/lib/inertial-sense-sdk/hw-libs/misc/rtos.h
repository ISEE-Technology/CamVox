/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef RTOS_H_
#define RTOS_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "../../src/ISConstants.h"
#include "task.h"
#include "../drivers/d_time.h"
#if !defined(PLATFORM_IS_EVB_2)
#include "../../../hdw-src/uINS-3/IS_uINS/src/misc/debug_gpio.h"
#endif

#if defined(PLATFORM_IS_EVB_2) && PLATFORM_IS_EVB_2
#define RTOS_NUM_TASKS	(EVB_RTOS_NUM_TASKS)
#else
#define RTOS_NUM_TASKS	(UINS_RTOS_NUM_TASKS)
#endif

#if defined(DBGPIO_START) && defined(DBGPIO_END)
#define BEGIN_CRITICAL_SECTION	{vTaskSuspendAll(); taskENTER_CRITICAL(); DBGPIO_START(DBG_CRITICAL_SECTION_PIN);}
#define END_CRITICAL_SECTION	{DBGPIO_END(DBG_CRITICAL_SECTION_PIN); taskEXIT_CRITICAL(); xTaskResumeAll();}
#else
#define BEGIN_CRITICAL_SECTION	{vTaskSuspendAll(); taskENTER_CRITICAL();}
#define END_CRITICAL_SECTION	{taskEXIT_CRITICAL(); xTaskResumeAll();}
#endif

#define GPBR_IDX_STATUS             0
#define GPBR_IDX_G1_TASK            1
#define GPBR_IDX_G2_FILE_NUM        2
#define GPBR_IDX_G3_LINE_NUM        3
#define GPBR_IDX_G4_FLASH_MIG       4
#define GPBR_IDX_G5_LR              5
#define GPBR_IDX_PC                 6
#define GPBR_IDX_PSR                7

int createTask(
	int index,
	pdTASK_CODE pxTaskCode,
	const char * const pcName,
	unsigned short usStackDepth,
	void *pvParameters,
	unsigned portBASE_TYPE uxPriority,
	portTickType xTimeIncrement);

// Monitor state of RTOS (i.e. stack high water mark, unused words).
void rtos_monitor(int numRtosTasks);
void rtosResetStats(void);

void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(uint32_t size, uint32_t remaining, uint32_t prevLR);

extern uint32_t g_faultLineNumber;
extern uint32_t g_faultFileNumber;

#ifdef __cplusplus
}
#endif
#endif // RTOS_H_
