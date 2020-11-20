/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>
#include "compiler.h"
//#include "system_same70.h"
#include "board.h"
//#include "conf_stdio.h" // enable/disabled printf


/** Board oscillator settings */
#define BOARD_FREQ_SLCK_XTAL            (32768U)
#define BOARD_FREQ_SLCK_BYPASS          (32768U)
#define BOARD_FREQ_MAINCK_XTAL          (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS        (12000000U)
#define BOARD_FREQ_CPU					3000000		// CPU target frequency

/** Master clock frequency */
#define BOARD_MCK                       CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
// #define BOARD_OSC_STARTUP_US            15625
#define BOARD_OSC_STARTUP_US            13000
// #define BOARD_OSC_STARTUP_US            31250

/*----------------------------------------------------------------------------*/
/**
 * \page same70_xplained_info "SAME70-XPLD - Board informations"
 * This page lists several definition related to the board description.
 *
 * \section Definitions
 * - \ref BOARD_NAME
 */

/** Name of the board */
#define BOARD_NAME "IS_EVB-2"
/** Board definition */
// #define same70xpld
/** Family definition (already defined) */
#define same70
/** Core definition */
#define cortexm7

/*----------------------------------------------------------------------------*/



/*! \name Pin Assignments by General Name
 */
//! @{

// Hardware Versions
// SAMS70N20B <=IMX-3.1
// SAME70Q20B >=IMX3.2
// SAME70Q20B EVB-2

// EVB-2 Hardware Detection Pins
#define EVB_HDW_DETECT_0_GPIO       (PIO_PC0_IDX)
#define EVB_HDW_DETECT_1_GPIO       (PIO_PC1_IDX)
#define EVB_HDW_DETECT_2_GPIO       (PIO_PC2_IDX)

#define HDW_DETECT_VER_EVB_2_0_0	0
#define HDW_DETECT_VER_EVB_2_0_1	1


// uINS Ser0 UART
#define UART_INS_SER0_RXD_PIN       (PIO_PA5_IDX)
#define UART_INS_SER0_RXD_FLAGS     (IOPORT_MODE_MUX_C)
#define UART_INS_SER0_TXD_PIN       (PIO_PA6_IDX)
#define UART_INS_SER0_TXD_FLAGS     (IOPORT_MODE_MUX_C)
// uINS Ser1 UART Functionality
#define UART_INS_SER1_RXD_PIN       (PIO_PA9_IDX)
#define UART_INS_SER1_RXD_FLAGS     (IOPORT_MODE_MUX_A)
#define UART_INS_SER1_TXD_PIN       (PIO_PA10_IDX)
#define UART_INS_SER1_TXD_FLAGS     (IOPORT_MODE_MUX_A)
// uINS Ser1 SPI Functionality
#define SPI_INS_MISO_PIN		    (PIO_PD20_IDX)
#define SPI_INS_MISO_FLAGS		    (IOPORT_MODE_MUX_B)
#define SPI_INS_MOSI_PIN		    (PIO_PD21_IDX)
#define SPI_INS_MOSI_FLAGS		    (IOPORT_MODE_MUX_B)
#define SPI_INS_SCLK_PIN		    (PIO_PD22_IDX)
#define SPI_INS_SCLK_FLAGS		    (IOPORT_MODE_MUX_B)
#define SPI_INS_CS_PIN			    (PIO_PD12_IDX)
#define SPI_INS_CS_FLAGS		    (IOPORT_MODE_MUX_C)
#define SPI_INS_EN				    (PIO_PC20_IDX)

// uINS Data Ready
#define INS_DATA_RDY_PIN_IDX		(PIO_PD2_IDX)	
#define INS_DATA_RDY_PIN_MASK		(PIO_PD2)
#define INS_DATA_RDY_PIN_ID			(ID_PIOD)
#define INS_DATA_RDY_PIN_PIO		(PIOD) 

// uINS Reset
#define INS_RESET_PIN_PIN		    (PIO_PA2_IDX)

// RS232/RS422/RS485
#define UART_SP330_RXD_PIN		    (PIO_PD28_IDX)
#define UART_SP330_RXD_FLAGS	    (IOPORT_MODE_MUX_A)
#define UART_SP330_TXD_PIN		    (PIO_PD30_IDX)
#define UART_SP330_TXD_FLAGS	    (IOPORT_MODE_MUX_A)
#define SP330_NSLEW_PIN			    (PIO_PA18_IDX)          // Data rate limit to 250 kbps when low
#define SP330_NSHDN_PIN			    (PIO_PA19_IDX)
#define SP330_N485_RXEN_PIN			(PIO_PA20_IDX)          // RS845 enable receiver when low
#define SP330_NFULL_DPLX_PIN		(PIO_PC18_IDX)          // RS485 full duplex when low
#define SP330_485_N232_PIN		    (PIO_PD4_IDX)           // RS232 low / RS485 high

// SD Card Interface
#define SD_CLK_PIN				    (PIO_PA25_IDX)
#define SD_CLK_FLAGS			    (IOPORT_MODE_MUX_D)
#define SD_CMD_PIN				    (PIO_PA28_IDX)
#define SD_CMD_FLAGS			    (IOPORT_MODE_MUX_C)
#define SD_D0_PIN				    (PIO_PA30_IDX)
#define SD_D0_FLAGS				    (IOPORT_MODE_MUX_C)
#define SD_D1_PIN				    (PIO_PA31_IDX)
#define SD_D1_FLAGS				    (IOPORT_MODE_MUX_C)
#define SD_D2_PIN				    (PIO_PA25_IDX)
#define SD_D2_FLAGS				    (IOPORT_MODE_MUX_C)
#define SD_D3_PIN				    (PIO_PA27_IDX)
#define SD_D3_FLAGS				    (IOPORT_MODE_MUX_C)
#define SD_DETECT				    (PIO_PC16_IDX)

// ATWINC WiFi SPI Interface
#define SPI_WIFI_MISO_PIN		    (PIO_PC26_IDX)
#define SPI_WIFI_MISO_FLAGS		    (IOPORT_MODE_MUX_C)
#define SPI_WIFI_MOSI_PIN		    (PIO_PC27_IDX)
#define SPI_WIFI_MOSI_FLAGS		    (IOPORT_MODE_MUX_C)
#define SPI_WIFI_SCLK_PIN		    (PIO_PC24_IDX)
#define SPI_WIFI_SCLK_FLAGS		    (IOPORT_MODE_MUX_C)
#define SPI_WIFI_CS_PIN			    (PIO_PC25_IDX)
#define SPI_WIFI_CS_FLAGS		    (IOPORT_MODE_MUX_C)
#define WIFI_RST_PIN			    (PIO_PC21_IDX)
#define WIFI_CHIPEN_PIN			    (PIO_PC22_IDX)
#define WIFI_IRQN_PIN			    (PIO_PC23_IDX)

// ATWINC BTLE UART Interface
#define UART_BTLE_RXD_PIN		    (PIO_PD15_IDX)
#define UART_BTLE_RXD_FLAGS		    (IOPORT_MODE_MUX_B)
#define UART_BTLE_TXD_PIN		    (PIO_PD16_IDX)
#define UART_BTLE_TXD_FLAGS		    (IOPORT_MODE_MUX_B)
#define UART_BTLE_RTS_PIN		    (PIO_PD18_IDX)
#define UART_BTLE_RTS_FLAGS		    (IOPORT_MODE_MUX_B)
#define UART_BTLE_CTS_PIN		    (PIO_PD19_IDX)
#define UART_BTLE_CTS_FLAGS		    (IOPORT_MODE_MUX_B)

// External Radio UART
#define UART_EXT_RADIO_RXD_PIN		(PIO_PD25_IDX)
#define UART_EXT_RADIO_RXD_FLAGS	(IOPORT_MODE_MUX_C | IOPORT_MODE_PULLUP)
#define UART_EXT_RADIO_TXD_PIN		(PIO_PD26_IDX)
#define UART_EXT_RADIO_TXD_FLAGS	(IOPORT_MODE_MUX_C)
#define EXT_RADIO_RST				(PIO_PD7_IDX)

// Xbee UART
#define UART_XBEE_RXD_PIN		    (PIO_PB0_IDX)
#define UART_XBEE_RXD_FLAGS		    (IOPORT_MODE_MUX_C)
#define UART_XBEE_TXD_PIN		    (PIO_PB1_IDX)
#define UART_XBEE_TXD_FLAGS		    (IOPORT_MODE_MUX_C)
#if 0   // Normal (EVB-2.0.1)
#define UART_XBEE_N_CTS_PIN		    (PIO_PB2_IDX)
#define UART_XBEE_N_CTS_FLAGS		(IOPORT_MODE_MUX_C)
#define UART_XBEE_N_RTS_PIN		    (PIO_PB3_IDX)
#define UART_XBEE_N_RTS_FLAGS		(IOPORT_MODE_MUX_C)
#else   // temporary fix (EVB-2.0.0)
#define UART_XBEE_N_CTS_PIN		    (PIO_PB3_IDX)
#define UART_XBEE_N_CTS_FLAGS		(IOPORT_MODE_MUX_C)
#define UART_XBEE_N_RTS_PIN		    (PIO_PB2_IDX)
#define UART_XBEE_N_RTS_FLAGS		(IOPORT_MODE_MUX_C)
#endif
#define UART_XBEE_N_DTR_PIN		    (PIO_PD1_IDX)
#define UART_XBEE_N_DTR_FLAGS	    (IOPORT_MODE_MUX_D)
#define XBEE_SUPPLY_EN_PIN			(PIO_PD17_IDX)
#define XBEE_VUSB_DISABLE_PIN		(PIO_PC3_IDX)
#define XBEE_RST_PIN			    (PIO_PA29_IDX)
#define XBEE_SLEEP_RQ_PIN			(PIO_PD1_IDX)

// USB
#define USB_PORT_NUM		EVB2_PORT_USB
	
// CAN Transceiver
#define CAN_RXD_PIN				    (PIO_PC12_IDX)
#define CAN_RXD_FLAGS			    (IOPORT_MODE_MUX_C)
#define CAN_TXD_PIN				    (PIO_PC14_IDX)
#define CAN_TXD_FLAGS			    (IOPORT_MODE_MUX_C)

// I2C Bus
#define I2C_0_SCL_PIN			    (PIO_PA3_IDX)
#define I2C_0_SCL_FLAGS			    (IOPORT_MODE_MUX_A)
#define I2C_0_SDA_PIN			    (PIO_PA4_IDX)
#define I2C_0_SDA_FLAGS			    (IOPORT_MODE_MUX_A)

// GPS Time Pulse Input
#define GPS_TP_PIN				    (PIO_PC9_IDX)


// GPIO Pins    - GPIO pins are all tied to multiple SAME70 pins.  Below is just one of these pins.
#define GPIO_1_PIN				    (PIO_PA15_IDX)
#define GPIO_2_PIN				    (PIO_PA16_IDX)
#define GPIO_3_PIN				    (PIO_PA23_IDX)
#define GPIO_4_PIN				    (PIO_PA24_IDX)
#define GPIO_5_PIN				    (PIO_PB13_IDX)
#define GPIO_6_PIN				    (PIO_PC30_IDX)
#define GPIO_7_PIN				    (PIO_PE3_IDX)
#define GPIO_8_PIN				    (PIO_PE4_IDX)
#define GPIO_9_PIN				    (PIO_PE0_IDX)
#define GPIO_10_PIN				    (PIO_PE1_IDX)
// GPIO Inverted UART Control
#define GPIO_UART_INV_PIN		    (PIO_PE2_IDX)

// GPIO UART
#define GPIO_H8_UART_RXD_PIN		(PIO_PA21_IDX)
#define GPIO_H8_UART_RXD_FLAGS	    (IOPORT_MODE_MUX_A | IOPORT_MODE_PULLUP)
// #define GPIO_H8_UART_RXD_FLAGS	    (IOPORT_MODE_MUX_A)
#define GPIO_H8_UART_TXD_PIN		(PIO_PB4_IDX)
#define GPIO_H8_UART_TXD_FLAGS	    (IOPORT_MODE_MUX_D)


// USB ID
#define M_USB_ID_PIN			    (PIO_PA17_IDX)

#define LED_OFF(led)                ioport_set_pin_level(led,IOPORT_PIN_LEVEL_HIGH)
#define LED_ON(led)                 ioport_set_pin_level(led,IOPORT_PIN_LEVEL_LOW)
#define LED_TOGGLE(led)             ioport_toggle_pin_level(led)
// EVB - LED - Config
#define LED_CFG_RED_PIN			    (PIO_PD6_IDX)
#define LED_CFG_GRN_PIN			    (PIO_PA13_IDX)
#define LED_CFG_BLU_PIN			    (PIO_PD3_IDX)
#define LED_CFG_RED()               { LED_ON(LED_CFG_RED_PIN); LED_OFF(LED_CFG_GRN_PIN); LED_OFF(LED_CFG_BLU_PIN); }
#define LED_CFG_GREEN()             { LED_OFF(LED_CFG_RED_PIN); LED_ON(LED_CFG_GRN_PIN); LED_OFF(LED_CFG_BLU_PIN); }
#define LED_CFG_BLUE()              { LED_OFF(LED_CFG_RED_PIN); LED_OFF(LED_CFG_GRN_PIN); LED_ON(LED_CFG_BLU_PIN); }
#define LED_CFG_CYAN()              { LED_OFF(LED_CFG_RED_PIN); LED_ON(LED_CFG_GRN_PIN); LED_ON(LED_CFG_BLU_PIN); }
#define LED_CFG_YELLOW()            { LED_ON(LED_CFG_RED_PIN); LED_ON(LED_CFG_GRN_PIN); LED_OFF(LED_CFG_BLU_PIN); }
#define LED_CFG_PURPLE()            { LED_ON(LED_CFG_RED_PIN); LED_OFF(LED_CFG_GRN_PIN); LED_ON(LED_CFG_BLU_PIN); }
#define LED_CFG_WHITE()             { LED_ON(LED_CFG_RED_PIN); LED_ON(LED_CFG_GRN_PIN); LED_ON(LED_CFG_BLU_PIN); }
#define LED_CFG_OFF()               { LED_OFF(LED_CFG_RED_PIN); LED_OFF(LED_CFG_GRN_PIN); LED_OFF(LED_CFG_BLU_PIN); }

#define LED_COLOR_RED()				LED_CFG_RED();
#define LED_COLOR_GREEN()			LED_CFG_GREEN();
#define LED_COLOR_BLUE()			LED_CFG_BLUE();
#define LED_COLOR_CYAN()			LED_CFG_CYAN();
#define LED_COLOR_YELLOW()			LED_CFG_YELLOW();
#define LED_COLOR_PURPLE()			LED_CFG_PURPLE();
#define LED_COLOR_WHITE()			LED_CFG_WHITE();

// EVB - LED - Logger
#define LED_LOG_RED_PIN			    (PIO_PA11_IDX)
#define LED_LOG_GRN_PIN			    (PIO_PD23_IDX)
#define LED_LOG_BLU_PIN			    (PIO_PA12_IDX)
#define LED_LOG_RED()               { LED_ON(LED_LOG_RED_PIN); LED_OFF(LED_LOG_GRN_PIN); LED_OFF(LED_LOG_BLU_PIN); }
#define LED_LOG_GREEN()             { LED_OFF(LED_LOG_RED_PIN); LED_ON(LED_LOG_GRN_PIN); LED_OFF(LED_LOG_BLU_PIN); }
#define LED_LOG_BLUE()              { LED_OFF(LED_LOG_RED_PIN); LED_OFF(LED_LOG_GRN_PIN); LED_ON(LED_LOG_BLU_PIN); }
#define LED_LOG_CYAN()              { LED_OFF(LED_LOG_RED_PIN); LED_ON(LED_LOG_GRN_PIN); LED_ON(LED_LOG_BLU_PIN); }
#define LED_LOG_YELLOW()            { LED_ON(LED_LOG_RED_PIN); LED_ON(LED_LOG_GRN_PIN); LED_OFF(LED_LOG_BLU_PIN); }
#define LED_LOG_PURPLE()            { LED_ON(LED_LOG_RED_PIN); LED_OFF(LED_LOG_GRN_PIN); LED_ON(LED_LOG_BLU_PIN); }
#define LED_LOG_WHITE()             { LED_ON(LED_LOG_RED_PIN); LED_ON(LED_LOG_GRN_PIN); LED_ON(LED_LOG_BLU_PIN); }
#define LED_LOG_OFF()               { LED_OFF(LED_LOG_RED_PIN); LED_OFF(LED_LOG_GRN_PIN); LED_OFF(LED_LOG_BLU_PIN); }

#define LEDS_ALL_ON()               { LED_ON(LED_LOG_RED_PIN); LED_ON(LED_LOG_GRN_PIN); LED_ON(LED_LOG_BLU_PIN); }
#define LEDS_ALL_OFF()              { LED_OFF(LED_LOG_RED_PIN); LED_OFF(LED_LOG_GRN_PIN); LED_OFF(LED_LOG_BLU_PIN); }
#define LEDS_ALL_TOGGLE()           { LED_TOGGLE(LED_LOG_RED_PIN); LED_TOGGLE(LED_LOG_GRN_PIN); LED_TOGGLE(LED_LOG_BLU_PIN); }



// LED - INS Communications
#define LED_INS_RXD_PIN			    (PIO_PC8_IDX)       // green
#define LED_INS_TXD_PIN			    (PIO_PC28_IDX)      // red
// LED - Xbee Communications
#define LED_XBEE_RXD_PIN		    (PIO_PC10_IDX)      // green
#define LED_XBEE_TXD_PIN		    (PIO_PC11_IDX)      // red
// LED - Wifi Communications
#define LED_WIFI_RXD_PIN		    (PIO_PC17_IDX)      // green
#define LED_WIFI_TXD_PIN		    (PIO_PD10_IDX)      // red

// Buttons
#define BUTTON_CFG_PIN              (g_hdw_detect==0?PIO_PC5_IDX:PIO_PC3_IDX)       // EVB 2.0.0 = PC5, all others are PC3
#define BUTTON_CFG_FLAGS            (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE)
#define BUTTON_CFG_SENSE            (IOPORT_SENSE_RISING)
#define BUTTON_LOG_PIN              (g_hdw_detect==0?PIO_PC6_IDX:PIO_PC7_IDX)       // EVB 2.0.0 = PC6, all others are PC7
#define BUTTON_LOG_FLAGS            (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE)
#define BUTTON_LOG_SENSE            (IOPORT_SENSE_RISING)

#define JTAG_TMS_SWDIO_PIN		    (PIO_PB6_IDX)
#define JTAG_TCK_SWCLK_PIN		    (PIO_PB7_IDX)
#define JTAG_TDO_SWO_PIN		    (PIO_PB5_IDX)
#define JTAG_TDI_PIN			    (PIO_PB4_IDX)


// ioport helper macros below acquired from same70_xplained/init.c
#ifndef ioport_set_port_peripheral_mode
#define ioport_set_port_peripheral_mode(port, masks, mode)	\
	do {\
		ioport_set_port_mode(port, masks, mode);\
		ioport_disable_port(port, masks);\
	} while (0)
#endif // ioport_set_port_peripheral_mode

#ifndef ioport_set_pin_peripheral_mode
#define ioport_set_pin_peripheral_mode(pin, mode) \
	do {\
		ioport_set_pin_mode(pin, mode);\
		ioport_disable_pin(pin);\
	} while (0)
#endif // ioport_set_pin_peripheral_mode

// The following "ioport_enable_pin(pin);" line is necessary to allow tristate. (whj)
#ifndef ioport_set_pin_input_mode
#define ioport_set_pin_input_mode(pin, mode, sense) \
	do {\
 		ioport_enable_pin(pin);\
		ioport_set_pin_dir(pin, IOPORT_DIR_INPUT);\
		ioport_set_pin_mode(pin, mode);\
		ioport_set_pin_sense_mode(pin, (ioport_sense)sense);\
	} while (0)
#endif // ioport_set_pin_input_mode

#ifndef ioport_set_pin_output_mode
#define ioport_set_pin_output_mode(pin, level) \
	do {\
 		ioport_enable_pin(pin);\
		ioport_set_pin_level(pin, level);\
		ioport_set_pin_dir(pin, IOPORT_DIR_OUTPUT);\
	} while (0)
#endif // ioport_set_pin_output_mode

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// START OF same70_xplained.h defaults 
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// #define CONSOLE_UART        (USART1)
// #define CONSOLE_UART_ID     (ID_USART1)
// /** USART1 pins definitions, PA21,PB4. */
// #define USART1_RXD_GPIO     (PIO_PA21_IDX)
// #define USART1_RXD_FLAGS    (IOPORT_MODE_MUX_A | IOPORT_MODE_PULLUP)
// #define USART1_TXD_GPIO     (PIO_PB4_IDX)
// #define USART1_TXD_FLAGS    (IOPORT_MODE_MUX_D)

// /** USART0 pins definitions, PB0,PB1. */
// #define USART0_RXD_GPIO   (PIO_PB0_IDX
// #define USART0_RXD_FLAGS  (IOPORT_MODE_MUX_C | IOPORT_MODE_PULLUP)
// #define USART0_TXD_GPIO   (PIO_PB1_IDX)
// #define USART0_TXD_FLAGS  (IOPORT_MODE_MUX_C)

#define PIN_USART0_SCK_IDX    (PIO_PB13_IDX)
#define PIN_USART0_SCK_FLAGS  (IOPORT_MODE_MUX_C)

/** USART0 pin CTS */
#define PIN_USART0_CTS_IDX    (PIO_PB2_IDX)
#define PIN_USART0_CTS_FLAGS  (IOPORT_MODE_MUX_C)

/** USART0 pin RTS */
#define PIN_USART0_RTS_IDX    (PIO_PB3_IDX)
#define PIN_USART0_RTS_FLAGS  (IOPORT_MODE_MUX_C)

/** HSMCI pins definition. */
/*! Number of slot connected on HSMCI interface */
#define SD_MMC_HSMCI_MEM_CNT            1
#define SD_MMC_HSMCI_SLOT_0_SIZE        4
/** HSMCI MCCDA pin definition. */
#define PIN_HSMCI_MCCDA_GPIO            (PIO_PA28_IDX)
#define PIN_HSMCI_MCCDA_FLAGS           (IOPORT_MODE_MUX_C)
/** HSMCI MCCK pin definition. */
#define PIN_HSMCI_MCCK_GPIO             (PIO_PA25_IDX)
#define PIN_HSMCI_MCCK_FLAGS            (IOPORT_MODE_MUX_D)
/** HSMCI MCDA0 pin definition. */
#define PIN_HSMCI_MCDA0_GPIO            (PIO_PA30_IDX)
#define PIN_HSMCI_MCDA0_FLAGS           (IOPORT_MODE_MUX_C)
/** HSMCI MCDA1 pin definition. */
#define PIN_HSMCI_MCDA1_GPIO            (PIO_PA31_IDX)
#define PIN_HSMCI_MCDA1_FLAGS           (IOPORT_MODE_MUX_C)
/** HSMCI MCDA2 pin definition. */
#define PIN_HSMCI_MCDA2_GPIO            (PIO_PA26_IDX)
#define PIN_HSMCI_MCDA2_FLAGS           (IOPORT_MODE_MUX_C)
/** HSMCI MCDA3 pin definition. */
#define PIN_HSMCI_MCDA3_GPIO            (PIO_PA27_IDX)
#define PIN_HSMCI_MCDA3_FLAGS           (IOPORT_MODE_MUX_C)

/** SD/MMC card detect pin definition. */
#define PIN_HSMCI_CD                    {PIO_PC16, PIOD, ID_PIOD, PIO_INPUT, PIO_PULLUP}
#define SD_MMC_0_CD_GPIO                (PIO_PC16_IDX)
#define SD_MMC_0_CD_PIO_ID              ID_PIOD
#define SD_MMC_0_CD_FLAGS               (IOPORT_MODE_PULLUP)
#define SD_MMC_0_CD_DETECT_VALUE        0


//! \name SW0 definitions
//@{
// #define SW0_PIN                   (PIO_PA11_IDX)
// #define SW0_ACTIVE                (IOPORT_PIN_LEVEL_LOW)
// #define SW0_INACTIVE              (!SW0_ACTIVE)
// #define SW0_SUPC_INPUT            2

/**
 * Wrapper macros for SW0, to ensure common naming across all Xplained
 * boards.
 */
// #define PIN_SW0      {PIO_PA11, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE}
// #define PIN_SW0_MASK PIO_PA11
// #define PIN_SW0_PIO  PIOA
// #define PIN_SW0_ID   ID_PIOA
// #define PIN_SW0_TYPE PIO_INPUT
// #define PIN_SW0_ATTR (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE)
//@}

/**
 * \name LED #0 definitions
 *
 * Wrapper macros for LED0, to ensure common naming across all Xplained
 * boards.
 */
//@{
// #define LED_0_NAME                "LED0 (yellow)"
// #define LED_0_PIN                 LED0_GPIO
// #define LED_0_ACTIVE              LED0_ACTIVE_LEVEL
// #define LED_0_INACTIVE            LED0_INACTIVE_LEVEL

// #define PIN_LED_0       {PIO_PC8, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}
// #define PIN_LED_0_MASK  PIO_PC8
// #define PIN_LED_0_PIO   PIOC
// #define PIN_LED_0_ID    ID_PIOC
// #define PIN_LED_0_TYPE  PIO_OUTPUT_1
// #define PIN_LED_0_ATTR  PIO_DEFAULT
//@}

/* TC-- Timer Count */
// #define PIN_TC0_TIOA0        (PIO_PA0_IDX)
// #define PIN_TC0_TIOA0_MUX    (IOPORT_MODE_MUX_B)
// #define PIN_TC0_TIOA0_FLAGS  (IOPORT_MODE_MUX_B)
// 
// #define PIN_TC0_TIOA0_PIO    PIOA
// #define PIN_TC0_TIOA0_MASK   PIO_PA0
// #define PIN_TC0_TIOA0_ID     ID_PIOA
// #define PIN_TC0_TIOA0_TYPE   PIO_PERIPH_B
// #define PIN_TC0_TIOA0_ATTR   PIO_DEFAULT
// 
// #define PIN_TC3_TIOA11	(PIO_PD21_IDX)
// #define PIN_TC3_TIOA11_MUX	(IOPORT_MODE_MUX_C)
// #define PIN_TC3_TIOA11_FLAGS	(IOPORT_MODE_MUX_C)
// 
// #define PIN_TC3_TIOA11_PIO	PIOD
// #define PIN_TC3_TIOA11_MASK	PIO_PD21
// #define PIN_TC3_TIOA11_ID	ID_PIOD
// #define PIN_TC3_TIOA11_TYPE	PIO_PERIPH_C
// #define PIN_TC3_TIOA11_ATTR	PIO_DEFAULT
// 
// //! Number of on-board LEDs
// #define BOARD_NUM_OF_LED 1

/**
 * Push button #0 definition. Attributes = pull-up + debounce + interrupt on
 * rising edge.
 */
// #define BUTTON_0_NAME             "SW0"
// #define BUTTON_0_PIN              SW0_PIN
// #define BUTTON_0_ACTIVE           SW0_ACTIVE
// #define BUTTON_0_INACTIVE         SW0_INACTIVE
// #define BUTTON_0_SUPC_INPUT       SW0_SUPC_INPUT
// #define GPIO_PUSH_BUTTON_0        BUTTON_0_PIN
// 
// #define PUSHBUTTON_1_NAME        "SW0"
// #define PUSHBUTTON_1_WKUP_LINE   (2)
// #define PUSHBUTTON_1_WKUP_FSTT   (PMC_FSMR_FSTT2)
// #define GPIO_PUSH_BUTTON_1       (PIO_PA11_IDX)
// #define GPIO_PUSH_BUTTON_1_FLAGS (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE)
// #define GPIO_PUSH_BUTTON_1_SENSE (IOPORT_SENSE_RISING)
// 
// #define PIN_PUSHBUTTON_1       {PIO_PA11, PIOA, ID_PIOA, PIO_INPUT, \
// 								PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE}
// #define PIN_PUSHBUTTON_1_MASK  PIO_PA11
// #define PIN_PUSHBUTTON_1_PIO   PIOA
// #define PIN_PUSHBUTTON_1_ID    ID_PIOA
// #define PIN_PUSHBUTTON_1_TYPE  PIO_INPUT
// #define PIN_PUSHBUTTON_1_ATTR  (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE)
// #define PIN_PUSHBUTTON_1_IRQn  PIOA_IRQn
// 
// /** List of all push button definitions. */
// #define PINS_PUSHBUTTONS    {PIN_PUSHBUTTON_1}

// //! \name Extension header #1 pin definitions
// //@{
// #define EXT1_PIN_3                PIO_PC31_IDX
// #define EXT1_PIN_4                PIO_PA19_IDX
// #define EXT1_PIN_5                PIO_PB3_IDX
// #define EXT1_PIN_6                PIO_PB2_IDX
// #define EXT1_PIN_7                PIO_PA0_IDX
// #define EXT1_PIN_8                PIO_PC30_IDX
// #define EXT1_PIN_9                PIO_PD28_IDX
// #define EXT1_PIN_10               PIO_PC17_IDX
// #define EXT1_PIN_11               PIO_PA3_IDX
// #define EXT1_PIN_12               PIO_PA4_IDX
// #define EXT1_PIN_13               PIO_PB0_IDX
// #define EXT1_PIN_14               PIO_PB1_IDX
// #define EXT1_PIN_15               PIO_PD25_IDX
// #define EXT1_PIN_16               PIO_PD21_IDX
// #define EXT1_PIN_17               PIO_PD20_IDX
// #define EXT1_PIN_18               PIO_PD22_IDX
// //@}
// 
// //! \name Extension header #1 pin definitions by function
// //@{
// #define EXT1_PIN_ADC_0            EXT1_PIN_3
// #define EXT1_PIN_ADC_1            EXT1_PIN_4
// #define EXT1_PIN_GPIO_0           EXT1_PIN_5
// #define EXT1_PIN_GPIO_1           EXT1_PIN_6
// #define EXT1_PIN_PWM_0            EXT1_PIN_7
// #define EXT1_PIN_PWM_1            EXT1_PIN_8
// #define EXT1_PIN_IRQ              EXT1_PIN_9
// #define EXT1_PIN_TWI_SDA          EXT1_PIN_11
// #define EXT1_PIN_TWI_SCL          EXT1_PIN_12
// #define EXT1_PIN_UART_RX          EXT1_PIN_13
// #define EXT1_PIN_UART_TX          EXT1_PIN_14
// #define EXT1_PIN_SPI_SS_1         EXT1_PIN_10
// #define EXT1_PIN_SPI_SS_0         EXT1_PIN_15
// #define EXT1_PIN_SPI_MOSI         EXT1_PIN_16
// #define EXT1_PIN_SPI_MISO         EXT1_PIN_17
// #define EXT1_PIN_SPI_SCK          EXT1_PIN_18
// //@}
// 
// //! \name Extension header #2 pin definitions
// //@{
// #define EXT2_PIN_3                PIO_PD30_IDX
// #define EXT2_PIN_4                PIO_PC13_IDX
// #define EXT2_PIN_5                PIO_PA6_IDX
// #define EXT2_PIN_6                PIO_PD11_IDX
// #define EXT2_PIN_7                PIO_PC19_IDX
// #define EXT2_PIN_8                PIO_PD26_IDX
// #define EXT2_PIN_9                PIO_PA2_IDX
// #define EXT2_PIN_10               PIO_PA24_IDX
// #define EXT2_PIN_11               PIO_PA3_IDX
// #define EXT2_PIN_12               PIO_PA4_IDX
// #define EXT2_PIN_13               PIO_PA21_IDX
// #define EXT2_PIN_14               PIO_PB4_IDX
// #define EXT2_PIN_15               PIO_PD27_IDX
// #define EXT2_PIN_16               PIO_PD21_IDX
// #define EXT2_PIN_17               PIO_PD20_IDX
// #define EXT2_PIN_18               PIO_PD22_IDX
// //@}
// 
// //! \name Extension header #2 pin definitions by function
// //@{
// #define EXT2_PIN_ADC_0            EXT2_PIN_3
// #define EXT2_PIN_ADC_1            EXT2_PIN_4
// #define EXT2_PIN_GPIO_0           EXT2_PIN_5
// #define EXT2_PIN_GPIO_1           EXT2_PIN_6
// #define EXT2_PIN_PWM_0            EXT2_PIN_7
// #define EXT2_PIN_PWM_1            EXT2_PIN_8
// #define EXT2_PIN_IRQ              EXT2_PIN_9
// #define EXT2_PIN_TWI_SDA          EXT2_PIN_11
// #define EXT2_PIN_TWI_SCL          EXT2_PIN_12
// #define EXT2_PIN_UART_RX          EXT2_PIN_13
// #define EXT2_PIN_UART_TX          EXT2_PIN_14
// #define EXT2_PIN_SPI_SS_1         EXT2_PIN_10
// #define EXT2_PIN_SPI_SS_0         EXT2_PIN_15
// #define EXT2_PIN_SPI_MOSI         EXT2_PIN_16
// #define EXT2_PIN_SPI_MISO         EXT2_PIN_17
// #define EXT2_PIN_SPI_SCK          EXT2_PIN_18
// //@}

// /** PCK0 pin definition (PA6) */
// #define PIN_PCK0         (PIO_PA6_IDX)
// #define PIN_PCK0_MUX     (IOPORT_MODE_MUX_B)
// #define PIN_PCK0_FLAGS   (IOPORT_MODE_MUX_B)
// #define PIN_PCK0_PORT    IOPORT_PIOA
// #define PIN_PCK0_MASK    PIO_PA6B_PCK0
// #define PIN_PCK0_PIO     PIOA
// #define PIN_PCK0_ID      ID_PIOA
// #define PIN_PCK0_TYPE    PIO_PERIPH_B
// #define PIN_PCK0_ATTR    PIO_DEFAULT
// 
// 
// /** TWI0 pins definition */
// #define TWIHS0_DATA_GPIO   PIO_PA3_IDX
// #define TWIHS0_DATA_FLAGS  (IOPORT_MODE_MUX_A)
// #define TWIHS0_CLK_GPIO    PIO_PA4_IDX
// #define TWIHS0_CLK_FLAGS   (IOPORT_MODE_MUX_A)
// 
// /** SPI0 pins definition */
// #define SPI0_MISO_GPIO    PIO_PD20_IDX
// #define SPI0_MISO_FLAGS  (IOPORT_MODE_MUX_B)
// #define SPI0_MOSI_GPIO    PIO_PD21_IDX
// #define SPI0_MOSI_FLAGS  (IOPORT_MODE_MUX_B)
// #define SPI0_NPCS0_GPIO   PIO_PB2_IDX
// #define SPI0_NPCS0_FLAGS  (IOPORT_MODE_MUX_D)
// #define SPI0_NPCS1_GPIO   PIO_PD25_IDX
// #define SPI0_NPCS1_FLAGS  (IOPORT_MODE_MUX_B)
// #define SPI0_NPCS2_GPIO   PIO_PD12_IDX
// #define SPI0_NPCS2_FLAGS  (IOPORT_MODE_MUX_C)
// #define SPI0_NPCS3_GPIO   PIO_PD27_IDX
// #define SPI0_NPCS3_FLAGS  (IOPORT_MODE_MUX_B)
// #define SPI0_SPCK_GPIO    PIO_PD22_IDX
// #define SPI0_SPCK_FLAGS  (IOPORT_MODE_MUX_B)

// /** QSPI pins definition */
// #define QSPI_QSCK_GPIO    PIO_PA14_IDX
// #define QSPI_QSCK_FLAGS   (IOPORT_MODE_MUX_A)
// #define QSPI_QCS_GPIO     PIO_PA11_IDX
// #define QSPI_QCS_FLAGS    (IOPORT_MODE_MUX_A)
// #define QSPI_QIO0_GPIO    PIO_PA13_IDX
// #define QSPI_QIO0_FLAGS   (IOPORT_MODE_MUX_A)
// #define QSPI_QIO1_GPIO    PIO_PA12_IDX
// #define QSPI_QIO1_FLAGS   (IOPORT_MODE_MUX_A)
// #define QSPI_QIO2_GPIO    PIO_PA17_IDX
// #define QSPI_QIO2_FLAGS   (IOPORT_MODE_MUX_A)
// #define QSPI_QIO3_GPIO    PIO_PD31_IDX
// #define QSPI_QIO3_FLAGS   (IOPORT_MODE_MUX_A)

// /** AFEC channel for potentiometer */
// #define AFEC_CHANNEL_POTENTIOMETER  AFEC_CHANNEL_0
// 
// #define MCAN_MODULE              MCAN1
/*----------------------------------------------------------------------------*/
/**
 * \page same70_xpld_CAN "SAME70-XPLD - CAN"
 * This page lists definitions related to CAN0 and CAN1.
 *
 * CAN
 * - \ref PIN_CAN0_TRANSCEIVER_RXEN
 * - \ref PIN_CAN0_TRANSCEIVER_RS
 * - \ref PIN_CAN0_TXD
 * - \ref PIN_CAN0_RXD
 * - \ref PINS_CAN0
 *
 * - \ref PIN_CAN1_TRANSCEIVER_RXEN
 * - \ref PIN_CAN1_TRANSCEIVER_RS
 * - \ref PIN_CAN1_TXD
 * - \ref PIN_CAN1_RXD
 * - \ref PINS_CAN1
 */
// /** CAN0 transceiver PIN RS. */
// #define PIN_CAN0_TR_RS_IDX        PIO_PE0_IDX
// #define PIN_CAN0_TR_RS_FLAGS      IOPORT_DIR_OUTPUT
// 
// /** CAN0 transceiver PIN EN. */
// #define PIN_CAN0_TR_EN_IDX        PIO_PE1_IDX
// #define PIN_CAN0_TR_EN_FLAGS      IOPORT_DIR_OUTPUT
// 
// /** CAN0 PIN RX. */
// #define PIN_CAN0_RX_IDX           PIO_PB3_IDX
// #define PIN_CAN0_RX_FLAGS         IOPORT_MODE_MUX_A
// 
// /** CAN0 PIN TX. */
// #define PIN_CAN0_TX_IDX           PIO_PB2_IDX
// #define PIN_CAN0_TX_FLAGS         IOPORT_MODE_MUX_A
// 
// /** CAN1 transceiver PIN RS. */
// #define PIN_CAN1_TR_RS_IDX        PIO_PE2_IDX
// #define PIN_CAN1_TR_RS_FLAGS      IOPORT_DIR_OUTPUT
// 
// /** CAN1 transceiver PIN EN. */
// #define PIN_CAN1_TR_EN_IDX        PIO_PE3_IDX
// #define PIN_CAN1_TR_EN_FLAGS      IOPORT_DIR_OUTPUT
// 
// /** CAN1 PIN RX. */
// #define PIN_CAN1_RX_IDX           PIO_PC12_IDX
// #define PIN_CAN1_RX_FLAGS         IOPORT_MODE_MUX_C
// 
// /** CAN1 PIN TX. */
// #define PIN_CAN1_TX_IDX           PIO_PC14_IDX
// #define PIN_CAN1_TX_FLAGS         IOPORT_MODE_MUX_C
// 
// /** PWM LED0 pin definitions. */
// #define PIN_PWM_LED0_GPIO    PIO_PA23_IDX
// #define PIN_PWM_LED0_FLAGS   (IOPORT_MODE_MUX_B)
// #define PIN_PWM_LED0_CHANNEL PWM_CHANNEL_0
// 
// /** PWM LED1 pin definitions. */
// #define PIN_PWM_LED1_GPIO    PIO_PA24_IDX
// #define PIN_PWM_LED1_FLAGS   (IOPORT_MODE_MUX_B)
// #define PIN_PWM_LED1_CHANNEL PWM_CHANNEL_1

/*----------------------------------------------------------------------------*/
// /** GMAC HW configurations */
// #define BOARD_GMAC_PHY_ADDR   0
// 
// #define PIN_GMAC_RESET_MASK   PIO_PC10
// #define PIN_GMAC_RESET_PIO    PIOC
// #define PIN_GMAC_INT_MASK     PIO_PA14
// #define PIN_GMAC_INT_PIO      PIOA
// #define PIN_GMAC_PERIPH       PIO_PERIPH_A
// #define PIN_GMAC_PIO          PIOD
// #define PIN_GMAC_MASK         (PIO_PD0A_GTXCK | PIO_PD1A_GTXEN | PIO_PD2A_GTX0 | \
// 						       PIO_PD3A_GTX1 | PIO_PD4A_GRXDV | PIO_PD5A_GRX0 |  \
// 						       PIO_PD6A_GRX1 | PIO_PD7A_GRXER | PIO_PD8A_GMDC | \
// 						       PIO_PD9A_GMDIO)
// 
// /** Board configuration of the AT24MAC EEPROM */
// #define BOARD_AT24MAC_TWIHS               TWIHS0
// //#define BOARD_AT24MAC_ADDRESS             (0xBE >> 1)
// #define BOARD_AT24MAC_TWIHS_CLK           (400000UL)
// #define BOARD_AT24MAC_PAGE_SIZE           16
// #define BOARD_AT24MAC_TWIHS_INSTANCE      TWIHS0
// #define BOARD_AT24MAC_ADDRESS             (0xAE >> 1)
// #define BOARD_CLK_TWIHS_EEPROM            PIO_PA4

// /** EBI pins configuration for LCD */
// /** LCD reset pin */
// #define PIN_EBI_RESET_MASK	  PIO_PC13
// #define PIN_EBI_RESET_PIO	  PIOC
// #define PIN_EBI_RESET_TYPE    PIO_OUTPUT_1
// #define PIN_EBI_RESET_ATTRI   PIO_DEFAULT
// 
// /** LCD command/data select pin */
// #define PIN_EBI_CDS_MASK	  PIO_PC30
// #define PIN_EBI_CDS_PIO		  PIOC
// #define PIN_EBI_CDS_TYPE	  PIO_OUTPUT_1
// #define PIN_EBI_CDS_ATTRI	  PIO_DEFAULT
// 
// /** LCD data pin */
// #define PIN_EBI_DATAL_MASK     0xFF
// #define PIN_EBI_DATAL_PIO	   PIOC
// #define PIN_EBI_DATAL_TYPE	   PIO_PERIPH_A
// #define PIN_EBI_DATAL_ATTRI    PIO_PULLUP
// 
// #define PIN_EBI_DATAH_0_MASK   0x3F
// #define PIN_EBI_DATAH_0_PIO	   PIOE
// #define PIN_EBI_DATAH_0_TYPE   PIO_PERIPH_A
// #define PIN_EBI_DATAH_0_ATTRI  PIO_PULLUP
// 
// #define PIN_EBI_DATAH_1_MASK   (PIO_PA15A_D14|PIO_PA16A_D15)
// #define PIN_EBI_DATAH_1_PIO	   PIOA
// #define PIN_EBI_DATAH_1_TYPE   PIO_PERIPH_A
// #define PIN_EBI_DATAH_1_ATTRI  PIO_PULLUP
// 
// /** LCD WE pin */
// #define PIN_EBI_NWE_MASK   PIO_PC8A_NWE
// #define PIN_EBI_NWE_PIO	   PIOC
// #define PIN_EBI_NWE_TYPE   PIO_PERIPH_A
// #define PIN_EBI_NWE_ATTRI  PIO_PULLUP
// 
// /** LCD RD pin */
// #define PIN_EBI_NRD_MASK   PIO_PC11A_NRD
// #define PIN_EBI_NRD_PIO	   PIOC
// #define PIN_EBI_NRD_TYPE   PIO_PERIPH_A
// #define PIN_EBI_NRD_ATTRI  PIO_PULLUP
// 
// /** LCD CS pin (NCS3) */
// #define PIN_EBI_CS_MASK   PIO_PD19A_NCS3
// #define PIN_EBI_CS_PIO	  PIOD
// #define PIN_EBI_CS_TYPE   PIO_PERIPH_A
// #define PIN_EBI_CS_ATTRI  PIO_PULLUP
// 
// /** Back-light pin definition. */
// #define PIN_EBI_BACKLIGHT_MASK   PIO_PC9B_TIOB7
// #define PIN_EBI_BACKLIGHT_PIO	  PIOC
// #define PIN_EBI_BACKLIGHT_TYPE   PIO_PERIPH_B
// #define PIN_EBI_BACKLIGHT_ATTRI  PIO_DEFAULT

// /*! \name GPIO Connections of VBUS monitoring
//  */
// //! @{
// #define USB_VBUS_FLAGS         (PIO_INPUT | PIO_PULLUP)
// #define USB_VBUS_PIN             PIO_PC9_IDX  /* As IO pin input */
// #define USB_VBUS_PIN_IRQn ( PIOC_IRQn)
// #define USB_VBUS_PIO_ID       ID_PIOC
// #define USB_VBUS_PIO_MASK  PIO_PC9
// //! @}
// 
// /*! \name GPIO Connections of ID detecting
//  */
// //! @{
// #define USB_ID_FLAGS             (PIO_INPUT | PIO_PULLUP)
// #define USB_ID_PIN               PIO_PC16_IDX /* As IO pin input */
// #define USB_ID_PIN_IRQn     (PIOC_IRQn)
// #define USB_ID_PIO_ID         ID_PIOC
// #define USB_ID_PIO_MASK    PIO_PC16
// //! @}

// /** WM8904 Slave address */
// #define WM8904_SLAVE_ADDRESS        (0x34 >> 1)
// 
// /** TWI interface for WM8904 */
// #define WM8904_TWIHS  TWIHS0
// 
// /** WM8904 pins definition */
// #define WM8904_TK_PIO       PIO_PB1_IDX
// #define WM8904_TK_FLAGS     PIO_PERIPH_D
// #define WM8904_TF_PIO       PIO_PB0_IDX
// #define WM8904_TF_FLAGS     PIO_PERIPH_D
// #define WM8904_TD_PIO       PIO_PD26_IDX
// #define WM8904_TD_FLAGS     PIO_PERIPH_B
// #define WM8904_RK_PIO       PIO_PA22_IDX
// #define WM8904_RK_FLAGS     PIO_PERIPH_A
// #define WM8904_RF_PIO       PIO_PD24_IDX
// #define WM8904_RF_FLAGS     PIO_PERIPH_B
// #define WM8904_RD_PIO       PIO_PA10_IDX
// #define WM8904_RD_FLAGS     PIO_PERIPH_C
// #define WM8904_PCK2_PIO     PIO_PA18_IDX
// #define WM8904_PCK2_FLAGS   PIO_PERIPH_B

// /**  Board SDRAM size for MT48LC16M16A2 */
// #define BOARD_SDRAM_SIZE        (2 * 1024 * 1024)
// 
// /** Address for transferring command bytes to the SDRAM. */
// #define BOARD_SDRAM_ADDR     0x70000000
// 
// /**  SDRAM pins definitions */
// #define SDRAM_BA0_PIO        PIO_PA20_IDX
// #define SDRAM_SDCK_PIO       PIO_PD23_IDX
// #define SDRAM_SDCKE_PIO      PIO_PD14_IDX
// #define SDRAM_SDCS_PIO       PIO_PC15_IDX
// #define SDRAM_RAS_PIO        PIO_PD16_IDX
// #define SDRAM_CAS_PIO        PIO_PD17_IDX
// #define SDRAM_SDWE_PIO       PIO_PD29_IDX
// #define SDRAM_NBS0_PIO       PIO_PC18_IDX
// #define SDRAM_NBS1_PIO       PIO_PD15_IDX
// #define SDRAM_A2_PIO         PIO_PC20_IDX
// #define SDRAM_A3_PIO         PIO_PC21_IDX
// #define SDRAM_A4_PIO         PIO_PC22_IDX
// #define SDRAM_A5_PIO         PIO_PC23_IDX
// #define SDRAM_A6_PIO         PIO_PC24_IDX
// #define SDRAM_A7_PIO         PIO_PC25_IDX
// #define SDRAM_A8_PIO         PIO_PC26_IDX
// #define SDRAM_A9_PIO         PIO_PC27_IDX
// #define SDRAM_A10_PIO        PIO_PC28_IDX
// #define SDRAM_A11_PIO        PIO_PC29_IDX
// #define SDRAM_SDA10_PIO      PIO_PD13_IDX
// #define SDRAM_D0_PIO         PIO_PC0_IDX
// #define SDRAM_D1_PIO         PIO_PC1_IDX
// #define SDRAM_D2_PIO         PIO_PC2_IDX
// #define SDRAM_D3_PIO         PIO_PC3_IDX
// #define SDRAM_D4_PIO         PIO_PC4_IDX
// #define SDRAM_D5_PIO         PIO_PC5_IDX
// #define SDRAM_D6_PIO         PIO_PC6_IDX
// #define SDRAM_D7_PIO         PIO_PC7_IDX
// #define SDRAM_D8_PIO         PIO_PE0_IDX
// #define SDRAM_D9_PIO         PIO_PE1_IDX
// #define SDRAM_D10_PIO        PIO_PE2_IDX
// #define SDRAM_D11_PIO        PIO_PE3_IDX
// #define SDRAM_D12_PIO        PIO_PE4_IDX
// #define SDRAM_D13_PIO        PIO_PE5_IDX
// #define SDRAM_D14_PIO        PIO_PA15_IDX
// #define SDRAM_D15_PIO        PIO_PA16_IDX
// 
// #define SDRAM_BA0_FLAGS      PIO_PERIPH_C
// #define SDRAM_SDCK_FLAGS     PIO_PERIPH_C
// #define SDRAM_SDCKE_FLAGS    PIO_PERIPH_C
// #define SDRAM_SDCS_FLAGS     PIO_PERIPH_A
// #define SDRAM_RAS_FLAGS      PIO_PERIPH_C
// #define SDRAM_CAS_FLAGS      PIO_PERIPH_C
// #define SDRAM_SDWE_FLAGS     PIO_PERIPH_C
// #define SDRAM_NBS0_FLAGS     PIO_PERIPH_A
// #define SDRAM_NBS1_FLAGS     PIO_PERIPH_C
// #define SDRAM_A_FLAGS        PIO_PERIPH_A
// #define SDRAM_SDA10_FLAGS    PIO_PERIPH_C
// #define SDRAM_D_FLAGS        PIO_PERIPH_A

// /** LCD SPI configuration */
// #define BOARD_ILI9488_SPI         SPI0
// #define BOARD_ILI9488_SPI_IRQN    SPI0_IRQn
// #define BOARD_ILI9488_SPI_NPCS    3
// 
// /** LCD SPI pins definition */
// #define LCD_SPI_MISO_PIO      PIO_PD20_IDX
// #define LCD_SPI_MISO_FLAGS    (PIO_PERIPH_B | PIO_DEFAULT)
// #define LCD_SPI_MOSI_PIO      PIO_PD21_IDX
// #define	LCD_SPI_MOSI_FLAGS    (PIO_PERIPH_B | PIO_DEFAULT)
// #define LCD_SPI_SPCK_PIO      PIO_PD22_IDX
// #define	LCD_SPI_SPCK_FLAGS    (PIO_PERIPH_B | PIO_DEFAULT)
// #define LCD_SPI_NPCS_PIO      PIO_PD27_IDX
// #define LCD_SPI_NPCS_FLAGS    (PIO_PERIPH_B | PIO_DEFAULT)
// 
// #define LCD_SPI_RESET_PIO     PIO_PA24_IDX
// #define LCD_SPI_RESET_FLAGS   (PIO_OUTPUT_1 | PIO_DEFAULT)
// #define LCD_SPI_CDS_PIO       PIO_PA6_IDX
// #define	LCD_SPI_CDS_FLAGS     (PIO_OUTPUT_1 | PIO_DEFAULT)
// #define LCD_SPI_BACKLIGHT_PIO     PIO_PC19_IDX
// #define	LCD_SPI_BACKLIGHT_FLAGS   (PIO_OUTPUT_1 | PIO_DEFAULT)


// /** TWI interface for maXTouch XPRO */
// #define MAXTOUCH_XPRO_TWIHS       TWIHS0
// 
// #define MAXTOUCH_XPRO_CHG_PIO     PIO_PA2_IDX
// 
// /** BNO055 external interrupt pin definition */
// #define PIN_BNO055_EXT_INIERRUPT       {PIO_PD28, PIOD, ID_PIOD, PIO_INPUT, \
// 										PIO_DEFAULT | PIO_IT_RISE_EDGE}
// #define PIN_BNO055_EXT_INIERRUPT_MASK  PIO_PD28
// #define PIN_BNO055_EXT_INIERRUPT_PIO   PIOD
// #define PIN_BNO055_EXT_INIERRUPT_ID    ID_PIOD
// #define PIN_BNO055_EXT_INIERRUPT_TYPE  PIO_INPUT
// #define PIN_BNO055_EXT_INIERRUPT_ATTR  (PIO_DEFAULT | PIO_IT_RISE_EDGE)
// #define PIN_BNO055_EXT_INIERRUPT_IRQn  PIOD_IRQn
// 
// #define BOARD_BNO055_TWIHS              TWIHS0
// #define BOARD_BNO055_ID_TWIHS        ID_TWIHS0
// 
// /** TWIHS ID for simulated EEPROM application to use */
// #define BOARD_AT30TSE_ID_TWIHS         ID_TWIHS0
// /** TWIHS Base for simulated TWI EEPROM application to use */
// #define BOARD_AT30TSE_TWIHS       TWIHS0

/*----------------------------------------------------------------------------*/
#endif // USER_BOARD_H
