#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include "board.h"
// #include "communications.h"
#include "../SDK/src/ISConstants.h"
#include "../SDK/src/com_manager.h"
#include "conf_common.h"
#include "misc/utilities.h"
#include "globals.h"

#include "misc/debug.h"


//_____ D E F I N I T I O N S ______________________________________________

#define ROUNDUP(x, y) ((x % y) ? ((x) += 4-((x)%4)) : (x))

// Terminal Parameter Debug
#define PARAM_STARTING_LINE	1
#define MAX_NUM_LINES		50
#define MAX_NAME_LENGTH		60
#define MAX_VALUE_LENGTH	20

typedef enum memory_type_t{ M_RAM=0, M_FLASH }memory_type_t;
typedef enum number_type_t{ R_X08=0, R_U08, R_S08, R_X16, R_U16, R_S16, R_X32, R_U32, R_S32, R_F32, R_F64, R_F32_R2D, R_STR, R_BLANK }number_type_t;

typedef struct term_line_t{
	union{
		char*			s08p;
		unsigned char*	u08p;
		short*			s16p;
		unsigned short*	u16p;
		int*			s32p;
		unsigned int*	u32p;
		float*			f32p;
		double*			f64p;
	};
	union{
		char			s08;
		unsigned char	u08;
		short			s16;
		unsigned short	u16;
		int				s32;
		unsigned int	u32;
		float			f32;
		double			f64;
	};
	memory_type_t memType;
	number_type_t numType;
	char name[MAX_NAME_LENGTH];
}term_line_t;

//_____ G L O B A L S ______________________________________________________

// static char g_txBuf[TXBUF_SIZE];
// static int g_txN=0;

#if 0	// terminal code
// Terminal Parameter Debug
static char g_refresh_term;
static int g_numLines;
static int g_selLine=1;
static char g_value[MAX_VALUE_LENGTH]={0};
static int g_valueSize=0;

static term_line_t g_term[MAX_NUM_LINES];
// static int g_term_update_cnt=0, g_term_compare_cnt=0;

extern int g_menu;
char g_enableTerm=1;					// terminal on by default in Debug mode
#endif

//_____ L O C A L   P R O T O T Y P E S ____________________________________

void term_add(void *val, char* name, memory_type_t memType, number_type_t numType);

void dg_printf(const char* format, ...)
{
	va_list ap;
	va_start(ap, format);

	// Caution: This function can consume quite a bit of stack memory, especially w/ floats.
	char debugMsg[64];
 	int n = vsnprintf(debugMsg, sizeof(debugMsg - 1), format, ap) + 1; // add null terminator
	(void)n;

	va_end(ap);
}


#if 0
void dg_prt_apd(const char *__fmt, ...)
{
    int n;
    
	va_list ap;
	va_start(ap, __fmt);
 	n = vsnprintf(&g_txBuf[g_txN], TXBUF_SIZE-g_txN, __fmt, ap);
	va_end(ap);
	
    n = min(n, TXBUF_SIZE);
    g_txN += n;
    
// 	g_txN += strlen(&g_txBuf[g_txN])
}

void dg_prt_snd(void)
{
	DBG_WRITE((unsigned char*) g_txBuf, g_txN);
// 	if(g_callback_fn_dg_write)
// 		g_callback_fn_dg_write((unsigned char*) g_txBuf, g_txN);
	g_txN = 0;
}

void dg_prt_clr(void)
{
	g_txN = 0;
}
#endif



#if 0	// terminal code
#include "d_flash.h"
void dg_vt100_clear(void)				{ dg_printf("\x1B[2J"); }
void dg_vt100_home(void)				{ dg_printf("\x1B[H"); }
void dg_vt100_row(unsigned int row)		{ dg_printf("\x1B[%dH", row); }
void dg_vt100_belowParams(void)			{ dg_printf("\x1B[%dH", g_numLines+2); }
//void dg_vt100_pos(row,col)				{ dg_printf("\x1B[%dH\x1B[%dC", row, col); }



void init_term(void)
{
	g_numLines = 0;
	g_refresh_term = 0;	
	
// 	g_term_compare_cnt = (int)(refreshPeriodSec/loopPeriodSec);
}

void term_update(void)
{
	char buf[MAX_VALUE_LENGTH];
	char c, mt;
	int i, n;
	
	// Read in Rx data if any
	n = DBG_READ((unsigned char*)buf, MAX_VALUE_LENGTH);
// 	n = g_callback_fn_dg_read((unsigned char*)buf, MAX_VALUE_LENGTH);
	
	if(n)
	{
// 		// Printout User Input
// 		dg_vt100_row(18);
// 		for( i=0; i<n; i++ )
// 			dg_printf("%d ", buf[i]);
// 		dg_printf("    \n");
// 		return;

		buf[0] = tolower(buf[0]);

		if( ((g_term[g_selLine].numType == R_X08 || g_term[g_selLine].numType == R_X16 || g_term[g_selLine].numType == R_X32) &&
		    (buf[0]>='a' && buf[0]<='f')) || 
			buf[0]=='.' || buf[0]=='-' || (buf[0]>='0' && buf[0]<='9') )
		{	// Is a number	
			n = min(n, MAX_VALUE_LENGTH-g_valueSize);
			memcpy( &g_value[g_valueSize], buf, n);		
			g_valueSize += n;
			
			// Parse current value as a Number
			switch(g_term[g_selLine].numType)
			{
				case R_X08:		sscanf(g_value, "%x", &i);	g_term[g_selLine].u08 = (U8)i;	break;
				case R_U08:		sscanf(g_value, "%u", &i);	g_term[g_selLine].u08 = (U8)i;	break;
				case R_S08:		sscanf(g_value, "%d", &i);	g_term[g_selLine].s08 = (S8)i;	break;
				case R_X16:		sscanf(g_value, "%x", &i);	g_term[g_selLine].u16 = (U8)i;	break;
				case R_U16:		sscanf(g_value, "%u", &i);	g_term[g_selLine].u16 = (U8)i;	break;
				case R_S16:		sscanf(g_value, "%d", &i);	g_term[g_selLine].s16 = (S8)i;	break;
				case R_X32:		sscanf(g_value, "%x", &g_term[g_selLine].u32);	break;
				case R_U32:		sscanf(g_value, "%u", &g_term[g_selLine].u32);	break;
				case R_S32:		sscanf(g_value, "%d", &g_term[g_selLine].s32);	break;
				case R_F32:		sscanf(g_value, "%f", &g_term[g_selLine].f32);	break;
				case R_F64:		sscanf(g_value, "%lf", &g_term[g_selLine].f64);	break;
				case R_F32_R2D:	sscanf(g_value, "%f", &g_term[g_selLine].f32);	g_term[g_selLine].f32 *= C_DEG2RAD_F;	break;
				default:		break;
			}
		}
		else
		{	// Is a command
			switch(buf[0])
			{
				case 13:	// "Enter" to Accept entry
					if( g_term[g_selLine].memType == M_RAM )
					{	// RAM
						switch(g_term[g_selLine].numType)
						{
							case R_X08:
							case R_U08:		*g_term[g_selLine].u08p  = g_term[g_selLine].u08;	break;
							case R_S08:		*g_term[g_selLine].s08p  = g_term[g_selLine].s08;	break;
							case R_X16:
							case R_U16:		*g_term[g_selLine].u16p = g_term[g_selLine].u16;	break;
							case R_S16:		*g_term[g_selLine].s16p = g_term[g_selLine].s16;	break;
							case R_X32:
							case R_U32:		*g_term[g_selLine].u32p = g_term[g_selLine].u32;	break;
							case R_S32:		*g_term[g_selLine].s32p = g_term[g_selLine].s32;	break;
							case R_F32_R2D:
							case R_F32:		*g_term[g_selLine].f32p = g_term[g_selLine].f32;	break;
							case R_F64:		*g_term[g_selLine].f64p = g_term[g_selLine].f64;	break;
							default:		break;
						}						
					}
					else
					{	// Flash
						
						// Disable gimbal control when writing to flashing.
// 						g_gimbal.modeDes = MODE_HOMING;
						
						switch(g_term[g_selLine].numType)
						{
							case R_X08:
							case R_U08:		flash_write_U08( (void*)(g_term[g_selLine].u08p), g_term[g_selLine].u08 );	break;
							case R_S08:		flash_write_S08( (void*)(g_term[g_selLine].s08p), g_term[g_selLine].s08 );	break;
							case R_X16:
							case R_U16:		flash_write_U16( (void*)(g_term[g_selLine].u16p), g_term[g_selLine].u16 );	break;
							case R_S16:		flash_write_S16( (void*)(g_term[g_selLine].s16p), g_term[g_selLine].s16 );	break;
							case R_X32:
							case R_U32:		flash_write_U32( (void*)(g_term[g_selLine].u32p), g_term[g_selLine].u32 );	break;
							case R_S32:		flash_write_S32( (void*)(g_term[g_selLine].s32p), g_term[g_selLine].s32 );	break;
							case R_F32_R2D:
							case R_F32:		flash_write_F32( (void*)(g_term[g_selLine].f32p), g_term[g_selLine].f32 );	break;
							case R_F64:		flash_write_F64( (void*)(g_term[g_selLine].f64p), g_term[g_selLine].f64 );	break;
							default:		break;
						}						
					}
// 					setup_display(g_menu);
					break;

				case ' ':	// Emergency Stop
// 					g_gimbal.modeDes = MODE_DISABLED;
 					g_enableTerm=1;
					break;

				case 'c':	// Clear terminal and move to Home (top-left) position
					dg_vt100_clear();
					dg_vt100_home();
					break;

				case 'o':	// Toggle terminal output On/Off
					if(g_enableTerm)
					{
 						g_enableTerm=0;
						dg_vt100_clear();
						dg_vt100_home();
					}						 
					else
	 					g_enableTerm=1;
					break;
					
				case 'q':	// Reset to default menu and Setup Terminal
// 					setup_display(g_menu=DEFAULT_MENU);
					break;
					
				case '[':	// Up Arrow - Move Up
					g_selLine = max(--g_selLine, 0);
					// Skip blank lines and strings
					switch( g_term[g_selLine].numType )
					{
						case R_BLANK:
						case R_STR:
							if(g_selLine==0)
								g_selLine = min(++g_selLine, g_numLines-1);	
							else
								g_selLine = max(--g_selLine, 0);
							break;
						default:
							break;
					}									
					break;
					
				case ']':	// Down Arrow - Move Down
					g_selLine = min(++g_selLine, g_numLines-1);	
					// Skip blank lines and strings
					switch( g_term[g_selLine].numType )
					{
						case R_BLANK:
						case R_STR:
							if(g_selLine==g_numLines-1)
								g_selLine = max(--g_selLine, 0);
							else
								g_selLine = min(++g_selLine, g_numLines-1);	
							break;
						default:
							break;
					}										
					break;
								
				case 27:
					switch( buf[1] )
					{
						default:	// Alt Key Combinations
							c = buf[1] - 48;
							
							if( c>=0 && c<=9 )
							{
								g_menu = c;
// 								setup_display(g_menu);								
							}
							break;
							
						case 91:	// Special Keys
							switch( buf[2] )
							{
								case 65:	// Up Arrow - Move Up
									g_selLine = max(--g_selLine, 0);
									// Skip blank lines and strings
									switch( g_term[g_selLine].numType )
									{
										case R_BLANK:
										case R_STR:
											if(g_selLine==0)
												g_selLine = min(++g_selLine, g_numLines-1);	
											else
												g_selLine = max(--g_selLine, 0);
											break;
										default:
											break;
									}									
									break;							
								
								case 66:	// Down Arrow - Move Down
									g_selLine = min(++g_selLine, g_numLines-1);	
									// Skip blank lines and strings
									switch( g_term[g_selLine].numType )
									{
										case R_BLANK:
										case R_STR:
											if(g_selLine==g_numLines-1)
												g_selLine = max(--g_selLine, 0);
											else
												g_selLine = min(++g_selLine, g_numLines-1);	
											break;
										default:
											break;
									}										
									break;

								case 49:	// Function Keys
									if( buf[3]<=53 )		// F1 - F5
										g_menu = buf[3] - 48;
									else					// F6 - F8
										g_menu = buf[3] - 49;
								
// 									setup_display(g_menu);
									break;
								case 50:
									if( buf[3]<=49 )		// F9 - F10
										g_menu = buf[3] - 39;
									else					// F11 - F12
										g_menu = buf[3] - 40;
									
// 									setup_display(g_menu);
									break;
							}
							break;
						}						
// 						// Printout User Input
// 						dg_vt100_row(18);
// 						dg_printf("DEBUG F%d ", g_menu);
// 						dg_printf("    \n");
// 						return;

					break;
					
				case 127:		// Backspace
					g_valueSize = max(--g_valueSize, 0);
					g_value[g_valueSize] = 0;
					// Parse current value
					switch(g_term[g_selLine].numType)
					{
						case R_X08:		sscanf(g_value, "%x", &i);	g_term[g_selLine].u08 = (U8)i;	break;
						case R_U08:		sscanf(g_value, "%u", &i);	g_term[g_selLine].u08 = (U8)i;	break;
						case R_S08:		sscanf(g_value, "%d", &i);	g_term[g_selLine].s08 = (S8)i;	break;
						case R_X16:		sscanf(g_value, "%x", &i);	g_term[g_selLine].u16 = (U8)i;	break;
						case R_U16:		sscanf(g_value, "%u", &i);	g_term[g_selLine].u16 = (U8)i;	break;
						case R_S16:		sscanf(g_value, "%d", &i);	g_term[g_selLine].s16 = (S8)i;	break;
						case R_X32:		sscanf(g_value, "%x", &g_term[g_selLine].u32);	break;
						case R_U32:		sscanf(g_value, "%u", &g_term[g_selLine].u32);	break;
						case R_S32:		sscanf(g_value, "%d", &g_term[g_selLine].s32);	break;
						case R_F32:		sscanf(g_value, "%f", &g_term[g_selLine].f32);	break;
						case R_F32_R2D:	sscanf(g_value, "%f", &g_term[g_selLine].f32);	g_term[g_selLine].f32 *= C_DEG2RAD_F;	break;
						case R_F64:		sscanf(g_value, "%lf", &g_term[g_selLine].f64);	break;
						default:		break;
					}
			}

			// Remove entry after every command except backspace
			if(buf[0]!=127)
			{
				memset((void*)g_value, 0, sizeof(g_value));
				g_valueSize = 0;
			}
		}
		//dg_printf("\nselLine %d", g_selLine);
		//dg_vt100_home();
		g_refresh_term = 1;
	}

	// Only redrawing terminal if flag is set
// 	if(!g_refresh_term && g_term_update_cnt++<g_term_compare_cnt )
// 		return;
// 	g_term_update_cnt = 0;

	// Limit Selected Line	
	g_selLine = max(min(g_selLine, g_numLines-1), 0);

	// This keeps data from coming out the serial port if not needed		
	if(!g_enableTerm)
		return;
			
	// Redraw each line if needed
	for( i=0; i<g_numLines; i++)
	{
		dg_vt100_row(i+PARAM_STARTING_LINE);
		
		// Selected Line Indicator
		if(i == g_selLine)
			if(g_valueSize)		c = '>';
			else				c = '*';
		else					c = ' ';
		
		// Memory Type Indicator
		if( g_term[i].memType == M_FLASH)	mt = '.';
		else								mt = ' ';
		
		if( i==g_selLine && g_valueSize>0)
		{	// Preview new value
			switch(g_term[i].numType)
			{
				case R_X08:		dg_printf("%c         0x%2x %c%s", c, g_term[i].u08, mt, g_term[i].name);	break;
				case R_U08:		dg_printf("%c %12u %c%s", c, g_term[i].u08, mt, g_term[i].name);	break;
				case R_S08:		dg_printf("%c %12d %c%s", c, g_term[i].s08, mt, g_term[i].name);	break;
				case R_X16:		dg_printf("%c       0x%4x %c%s", c, g_term[i].u16, mt, g_term[i].name);	break;
				case R_U16:		dg_printf("%c %12u %c%s", c, g_term[i].u16, mt, g_term[i].name);	break;
				case R_S16:		dg_printf("%c %12d %c%s", c, g_term[i].s16, mt, g_term[i].name);	break;
				case R_X32:		dg_printf("%c   0x%8x %c%s", c, g_term[i].u32, mt, g_term[i].name);	break;
				case R_U32:		dg_printf("%c %12u %c%s", c, g_term[i].u32, mt, g_term[i].name);	break;
				case R_S32:		dg_printf("%c %12d %c%s", c, g_term[i].s32, mt, g_term[i].name);	break;
				case R_F32:		dg_printf("%c %12f %c%s", c, g_term[i].f32, mt, g_term[i].name);	break;
				case R_F32_R2D:	dg_printf("%c %12f %c%s", c, g_term[i].f32*C_DEG2RAD_F, mt, g_term[i].name);	break;
				case R_F64:		dg_printf("%c %12lf %c%s", c, g_term[i].f64, mt, g_term[i].name);	break;
				default:		break;
			}
		}
		else
		{	// Print current value
			switch(g_term[i].numType)
			{
				case R_X08:		dg_printf("%c         0x%2x %c%s", c, *g_term[i].u08p, mt, g_term[i].name);	break;
				case R_U08:		dg_printf("%c %12u %c%s", c, *g_term[i].u08p, mt, g_term[i].name);	break;
				case R_S08:		dg_printf("%c %12d %c%s", c, *g_term[i].s08p, mt, g_term[i].name);	break;
				case R_X16:		dg_printf("%c       0x%4x %c%s", c, *g_term[i].u16p, mt, g_term[i].name);	break;
				case R_S16:		dg_printf("%c %12d %c%s", c, *g_term[i].s16p, mt, g_term[i].name);	break;
				case R_U16:		dg_printf("%c %12u %c%s", c, *g_term[i].u16p, mt, g_term[i].name);	break;
				case R_X32:		dg_printf("%c   0x%8x %c%s", c, *g_term[i].u32p, mt, g_term[i].name);	break;
				case R_U32:		dg_printf("%c %12u %c%s", c, *g_term[i].u32p, mt, g_term[i].name);	break;
				case R_S32:		dg_printf("%c %12d %c%s", c, *g_term[i].s32p, mt, g_term[i].name);	break;
				case R_F32:		dg_printf("%c %12f %c%s", c, *g_term[i].f32p, mt, g_term[i].name);	break;
				case R_F32_R2D:	dg_printf("%c %12f %c%s", c, *g_term[i].f32p*C_DEG2RAD_F, mt, g_term[i].name);	break;
				case R_F64:		dg_printf("%c %12lf %c%s", c, *g_term[i].f64p, mt, g_term[i].name);	break;
				case R_STR:		dg_printf("%c %s %s",     c,  g_term[i].s08p,     g_term[i].name);	break;
				default:		break;
			}
		}
	}

	// Return cursor to top
	//dg_vt100_home();
	//dg_vt100_row(g_numLines+2);
	
	// Release print semaphore
	g_refresh_term = 0;
}


void term_add(void *ptr, char* name, memory_type_t memType, number_type_t numType)
{
	int nameLen;
	
	if(g_numLines >= MAX_NUM_LINES)
		return;
		
	nameLen = min(strlen(name), MAX_NAME_LENGTH);

	// Add Memory Type
	g_term[g_numLines].memType = memType;

	// Add Number Type
	g_term[g_numLines].numType = numType;
	
	// Add Data
	switch(numType)
	{
		case R_STR:			
		case R_X08:
		case R_U08:			g_term[g_numLines].u08p = (unsigned char*)ptr;	break;
		case R_S08:			g_term[g_numLines].s08p = (char*)ptr;	        break;
		case R_X16:
		case R_U16:			g_term[g_numLines].u16p = (unsigned short*)ptr; break;
		case R_S16:			g_term[g_numLines].s16p = (short*)ptr;	        break;
		case R_X32:
		case R_U32:			g_term[g_numLines].u32p = (unsigned int*)ptr;	break;
		case R_S32:			g_term[g_numLines].s32p = (int*)ptr;	        break;
		case R_F32:
		case R_F32_R2D:		g_term[g_numLines].f32p = (float*)ptr;	        break;
		case R_F64:			g_term[g_numLines].f64p = (double*)ptr;	        break;
		default:			break;
	}

	// Add Name	
	memset((void*)g_term[g_numLines].name, 0, MAX_NAME_LENGTH);
	memcpy((void*)g_term[g_numLines].name, name, nameLen);
	
	// Increment Number of Lines
	g_numLines++;
	
	g_refresh_term = 1;
}


void term_addBlankLine(void)
{
	if(g_numLines >= MAX_NUM_LINES)
		return;

	// Add Number Type
	g_term[g_numLines].numType = R_BLANK;
	
	// Increment Number of Lines
	g_numLines++;
	
	g_refresh_term = 1;
}

const char blankStr[] = " ";			
void term_addLabel( char* name )					{ term_addVarStr(  (void*)blankStr, name ); }
void term_addVarStr( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_STR ); }
void term_addVarX08( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_X08 ); }
void term_addVarU08( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_U08 ); }
void term_addVarS08( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_S08 ); }
void term_addVarX16( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_X16 ); }
void term_addVarU16( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_U16 ); }
void term_addVarS16( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_S16 ); }
void term_addVarX32( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_X32 ); }
void term_addVarU32( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_U32 ); }
void term_addVarS32( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_S32 ); }
void term_addVarF32( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_F32 ); }
void term_addVarF64( void *ptr, char* name )		{ term_add( ptr, name, M_RAM, R_F64 ); }
void term_addVarF32Rad2Deg( void *ptr, char* name )	{ term_add( ptr, name, M_RAM, R_F32_R2D ); }

void term_addFlaX08( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_X08 ); }
void term_addFlaU08( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_U08 ); }
void term_addFlaS08( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_S08 ); }
void term_addFlaX16( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_X16 ); }
void term_addFlaU16( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_U16 ); }
void term_addFlaS16( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_S16 ); }
void term_addFlaX32( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_X32 ); }
void term_addFlaU32( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_U32 ); }
void term_addFlaS32( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_S32 ); }
void term_addFlaF32( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_F32 ); }
void term_addFlaF64( void *ptr, char* name )		{ term_add( ptr, name, M_FLASH, R_F64 ); }
void term_addFlaF32Rad2Deg( void *ptr, char* name )	{ term_add( ptr, name, M_FLASH, R_F32_R2D ); }

#endif	// terminal code