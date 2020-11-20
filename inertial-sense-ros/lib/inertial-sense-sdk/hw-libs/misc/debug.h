#ifndef DEBUG_H_
#define DEBUG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"


//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//#  define DBG_USART               (&AVR32_USART1)
//#  define DBG_USART_RX_PIN        AVR32_USART1_RXD_0_0_PIN
//#  define DBG_USART_RX_FUNCTION   AVR32_USART1_RXD_0_0_FUNCTION
//#  define DBG_USART_TX_PIN        AVR32_USART1_TXD_0_0_PIN
//#  define DBG_USART_TX_FUNCTION   AVR32_USART1_TXD_0_0_FUNCTION
//#  define DBG_USART_BAUDRATE      57600

#define DEFAULT_MENU MENU_SYSTEM
// #define DEFAULT_MENU MENU_STABILIZED

extern int g_com_manager_enable_binary_protocol;

//_____ P R O T O T Y P E S ________________________________________________

#ifdef __cplusplus
extern "C" {
#endif

void dg_printf(const char *__fmt, ...);

#ifdef __cplusplus
}
#endif

#if 0
void dg_prt_apd(const char *__fmt, ...);
void dg_prt_snd(void);
void dg_prt_clr(void);

// ANSI/VT100 Terminal Control - http://www.termsys.demon.co.uk/vtansi.htm
void dg_vt100_clear(void);
void dg_vt100_home(void);
void dg_vt100_row(unsigned int row);
void dg_vt100_belowParams(void);
//void dg_vt100_pos(row,col);
//#define VT100_POS(row,col)	"\x1B[row;colf"	// doesn't work
#endif

#if 0	// terminal code
void init_term(void);
void term_update(void);
void term_addVarX08( void *ptr, char* name );
void term_addVarU08( void *ptr, char* name );
void term_addVarS08( void *ptr, char* name );
void term_addVarX16( void *ptr, char* name );
void term_addVarU16( void *ptr, char* name );
void term_addVarS16( void *ptr, char* name );
void term_addVarX32( void *ptr, char* name );
void term_addVarU32( void *ptr, char* name );
void term_addVarS32( void *ptr, char* name );
void term_addVarF32( void *ptr, char* name );
void term_addVarF64( void *ptr, char* name );
void term_addVarF32Rad2Deg( void *ptr, char* name );
void term_addVarStr( void *ptr, char* name );
void term_addLabel( char* name );

void term_addFlaX08( void *ptr, char* name );
void term_addFlaU08( void *ptr, char* name );
void term_addFlaS08( void *ptr, char* name );
void term_addFlaX16( void *ptr, char* name );
void term_addFlaU16( void *ptr, char* name );
void term_addFlaS16( void *ptr, char* name );
void term_addFlaX32( void *ptr, char* name );
void term_addFlaU32( void *ptr, char* name );
void term_addFlaS32( void *ptr, char* name );
void term_addFlaF32( void *ptr, char* name );
void term_addFlaF64( void *ptr, char* name );
void term_addFlaF32Rad2Deg( void *ptr, char* name );

void term_addBlankLine(void);
#endif	// terminal code

//#define CODE_PROFILE(__resultTimeUs__, __codeToProfile__) { uint32_t __startTimeUs__ = time_usec(); __codeToProfile__; uint32_t __endTimeUs__ = time_usec(); __resultTimeUs__ = _MAX((uint32_t)__resultTimeUs__, UINT32_TIME_DIFF(__endTimeUs__, __startTimeUs__)); }
#define CODE_PROFILE(__resultTimeUs__, __codeToProfile__) __codeToProfile__

// This function controls when to print for terminal compatibility
//void term_printf(const char *__fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_H_ */
