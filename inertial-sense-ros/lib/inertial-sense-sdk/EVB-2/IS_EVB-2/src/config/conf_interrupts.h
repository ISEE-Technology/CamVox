#ifndef _CONF_INTERRUPTS_H_
#define _CONF_INTERRUPTS_H_

// see ARM Cortex-M7 documentation on NVIC IPR
// Each priority field holds a priority value, 0-255.
// The lower the value, the greater the priority of
// the corresponding interrupt.

#define INT_PRIORITY_TC   1
#define INT_PRIORITY_RTT  2
#define INT_PRIORITY_DMA  3

#endif // _CONF_INTERRUPTS_H_

