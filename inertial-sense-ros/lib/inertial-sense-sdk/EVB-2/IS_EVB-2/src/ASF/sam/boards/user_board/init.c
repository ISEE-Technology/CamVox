/**
 * \file
 *
 * \brief SAME70-XPLD board init.
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "compiler.h"
#include "board.h"
#include "conf_board.h"
#include "ioport.h"
#include "pio.h"
#include "../../../../../../../src/data_sets.h"
#include "../../../../../../../hw-libs/drivers/d_usartDMA.h"
#include "../../../../drivers/d_time.h"
#include "../../../../spiTouINS.h"
#include "../../../../xbee.h"
#include "../../../../wifi.h"
#include "../../../../globals.h"
#include "../../../../CAN.h"
#ifdef CONF_BOARD_CONFIG_MPU_AT_INIT
#include "mpu.h"
#endif

/**
 * \brief Set peripheral mode for IOPORT pins.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param port IOPORT port to configure
 * \param masks IOPORT pin masks to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_port_peripheral_mode(port, masks, mode) \
	do {\
		ioport_set_port_mode(port, masks, mode);\
		ioport_disable_port(port, masks);\
	} while (0)

/**
 * \brief Set peripheral mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_pin_peripheral_mode(pin, mode) \
	do {\
		ioport_set_pin_mode(pin, mode);\
		ioport_disable_pin(pin);\
	} while (0)

/**
 * \brief Set input mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * The "ioport_enable_pin(pin);" line is necessary to allow tristate. (whj)
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 * \param sense Sense for interrupt detection (\ref ioport_sense)
 */
#undef ioport_set_pin_input_mode
#define ioport_set_pin_input_mode(pin, mode, sense) \
	do {\
 		ioport_enable_pin(pin);\
		ioport_set_pin_dir(pin, IOPORT_DIR_INPUT);\
		ioport_set_pin_mode(pin, mode);\
		ioport_set_pin_sense_mode(pin, sense);\
	} while (0)


#ifdef CONF_BOARD_CONFIG_MPU_AT_INIT
/**
 *	Default memory map
 *	Address range        Memory region      Memory type   Shareability  Cache policy
 *	0x00000000- 0x1FFFFFFF Code             Normal        Non-shareable  WT
 *	0x20000000- 0x3FFFFFFF SRAM             Normal        Non-shareable  WBWA
 *	0x40000000- 0x5FFFFFFF Peripheral       Device        Non-shareable  -
 *	0x60000000- 0x7FFFFFFF RAM              Normal        Non-shareable  WBWA
 *	0x80000000- 0x9FFFFFFF RAM              Normal        Non-shareable  WT
 *	0xA0000000- 0xBFFFFFFF Device           Device        Shareable
 *	0xC0000000- 0xDFFFFFFF Device           Device        Non Shareable
 *	0xE0000000- 0xFFFFFFFF System           -                  -
 */

uint8_t g_hdw_detect;

/**
 * \brief Set up a memory region.
 */
static void _setup_memory_region( void )
{

	uint32_t dw_region_base_addr;
	uint32_t dw_region_attr;

	__DMB();

/**
 *	ITCM memory region --- Normal
 *	START_Addr:-  0x00000000UL
 *	END_Addr:-    0x00400000UL
 */
	dw_region_base_addr =
		ITCM_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_DEFAULT_ITCM_REGION;

	dw_region_attr =
		MPU_AP_PRIVILEGED_READ_WRITE |
		mpu_cal_mpu_region_size(ITCM_END_ADDRESS - ITCM_START_ADDRESS) | MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
 *	Internal flash memory region --- Normal read-only
 *	(update to Strongly ordered in write accesses)
 *	START_Addr:-  0x00400000UL
 *	END_Addr:-    0x00600000UL
 */

	dw_region_base_addr =
		IFLASH_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_DEFAULT_IFLASH_REGION;

	dw_region_attr =
// 		MPU_AP_READONLY |
        MPU_AP_FULL_ACCESS |        
		INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
		mpu_cal_mpu_region_size(IFLASH_END_ADDRESS - IFLASH_START_ADDRESS) | MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
 *	DTCM memory region --- Normal
 *	START_Addr:-  0x20000000L
 *	END_Addr:-    0x20400000UL
 */

	/* DTCM memory region */
	dw_region_base_addr =
		DTCM_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_DEFAULT_DTCM_REGION;

	dw_region_attr =
		MPU_AP_PRIVILEGED_READ_WRITE |
		mpu_cal_mpu_region_size(DTCM_END_ADDRESS - DTCM_START_ADDRESS) | MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
 *	SRAM Cacheable memory region --- Normal
 *	START_Addr:-  0x20400000UL
 *	END_Addr:-    0x2043FFFFUL
 */
	/* SRAM memory  region */
	dw_region_base_addr =
		SRAM_FIRST_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_DEFAULT_SRAM_REGION_1;

	dw_region_attr =
		MPU_AP_FULL_ACCESS    |
		INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
		mpu_cal_mpu_region_size(SRAM_FIRST_END_ADDRESS - SRAM_FIRST_START_ADDRESS) | MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);


/**
 *	Internal SRAM second partition memory region --- Normal
 *	START_Addr:-  0x20440000UL
 *	END_Addr:-    0x2045FFFFUL
 */
	/* SRAM memory region */
	dw_region_base_addr =
		SRAM_SECOND_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_DEFAULT_SRAM_REGION_2;

	dw_region_attr =
		MPU_AP_FULL_ACCESS    |
		INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
		mpu_cal_mpu_region_size(SRAM_SECOND_END_ADDRESS - SRAM_SECOND_START_ADDRESS) | MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);

#ifdef MPU_HAS_NOCACHE_REGION
	dw_region_base_addr =
        SRAM_NOCACHE_START_ADDRESS |
        MPU_REGION_VALID |
        MPU_NOCACHE_SRAM_REGION;

    dw_region_attr =
        MPU_AP_FULL_ACCESS    |
        INNER_OUTER_NORMAL_NOCACHE_TYPE( SHAREABLE ) |
        mpu_cal_mpu_region_size(NOCACHE_SRAM_REGION_SIZE) | MPU_REGION_ENABLE;

    mpu_set_region( dw_region_base_addr, dw_region_attr);
#endif

/**
 *	Peripheral memory region --- DEVICE Shareable
 *	START_Addr:-  0x40000000UL
 *	END_Addr:-    0x5FFFFFFFUL
 */
	dw_region_base_addr =
		PERIPHERALS_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_PERIPHERALS_REGION;

	dw_region_attr = MPU_AP_FULL_ACCESS |
		MPU_REGION_EXECUTE_NEVER |
		SHAREABLE_DEVICE_TYPE |
		mpu_cal_mpu_region_size(PERIPHERALS_END_ADDRESS - PERIPHERALS_START_ADDRESS) | MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);


/**
 *	External EBI memory  memory region --- Strongly Ordered
 *	START_Addr:-  0x60000000UL
 *	END_Addr:-    0x6FFFFFFFUL
 */
	dw_region_base_addr =
		EXT_EBI_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_EXT_EBI_REGION;

	dw_region_attr =
		MPU_AP_FULL_ACCESS |
		/* External memory Must be defined with 'Device' or 'Strongly Ordered' attribute for write accesses (AXI) */
		STRONGLY_ORDERED_SHAREABLE_TYPE |
		mpu_cal_mpu_region_size(EXT_EBI_END_ADDRESS - EXT_EBI_START_ADDRESS) | MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
 *	SDRAM cacheable memory region --- Normal
 *	START_Addr:-  0x70000000UL
 *	END_Addr:-    0x7FFFFFFFUL
 */
	dw_region_base_addr =
		SDRAM_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_DEFAULT_SDRAM_REGION;

	dw_region_attr =
		MPU_AP_FULL_ACCESS    |
		INNER_NORMAL_WB_RWA_TYPE( SHAREABLE ) |
		mpu_cal_mpu_region_size(SDRAM_END_ADDRESS - SDRAM_START_ADDRESS) | MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);

/**
 *	QSPI memory region --- Strongly ordered
 *	START_Addr:-  0x80000000UL
 *	END_Addr:-    0x9FFFFFFFUL
 */
	dw_region_base_addr =
		QSPI_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_QSPIMEM_REGION;

	dw_region_attr =
		MPU_AP_FULL_ACCESS |
		STRONGLY_ORDERED_SHAREABLE_TYPE |
		mpu_cal_mpu_region_size(QSPI_END_ADDRESS - QSPI_START_ADDRESS) |
		MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);


/**
 *	USB RAM Memory region --- Device
 *	START_Addr:-  0xA0100000UL
 *	END_Addr:-    0xA01FFFFFUL
 */
	dw_region_base_addr =
		USBHSRAM_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_USBHSRAM_REGION;

	dw_region_attr =
		MPU_AP_FULL_ACCESS |
		MPU_REGION_EXECUTE_NEVER |
		SHAREABLE_DEVICE_TYPE |
		mpu_cal_mpu_region_size(USBHSRAM_END_ADDRESS - USBHSRAM_START_ADDRESS) |
		MPU_REGION_ENABLE;

	mpu_set_region( dw_region_base_addr, dw_region_attr);


	/* Enable the memory management fault , Bus Fault, Usage Fault exception */
	SCB->SHCSR |= (SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);

	/* Enable the MPU region */
	mpu_enable( MPU_ENABLE | MPU_PRIVDEFENA);

	__DSB();
	__ISB();
}
#endif

#ifdef CONF_BOARD_ENABLE_TCM_AT_INIT
#if defined(__GNUC__)
extern char _itcm_lma, _sitcm, _eitcm;
#endif

/** \brief  TCM memory enable
* The function enables TCM memories
*/
static inline void tcm_enable(void)
{

	__DSB();
	__ISB();
	
	SCB->ITCMCR = (SCB_ITCMCR_EN_Msk  | SCB_ITCMCR_RMW_Msk | SCB_ITCMCR_RETEN_Msk);
	SCB->DTCMCR = ( SCB_DTCMCR_EN_Msk | SCB_DTCMCR_RMW_Msk | SCB_DTCMCR_RETEN_Msk);
	
	__DSB();
	__ISB();
}
#else
/** \brief  TCM memory Disable

	The function enables TCM memories
 */
static inline void tcm_disable(void) 
{

	__DSB();
	__ISB();
	SCB->ITCMCR &= ~(uint32_t)(1UL);
	SCB->DTCMCR &= ~(uint32_t)SCB_DTCMCR_EN_Msk;
	__DSB();
	__ISB();
}
#endif

__attribute__((optimize("O0")))
static void waitPinPullup(void)
{
    // wait a tiny bit for pins to pull
    for (volatile int i = 0; i < 200000; i++) {}
}


void refresh_CFG_LED(void)
{
    switch(g_flashCfg->cbPreset)
    {
        default:
        case EVB2_CB_PRESET_ALL_OFF:            LED_CFG_OFF();      break;
        case EVB2_CB_PRESET_RS232:              LED_CFG_GREEN();    break;
        case EVB2_CB_PRESET_RS232_XBEE:         LED_CFG_BLUE();     break;
        case EVB2_CB_PRESET_RS422_WIFI:         LED_CFG_PURPLE();   break;
        case EVB2_CB_PRESET_SPI_RS232:          LED_CFG_CYAN();     break;
        case EVB2_CB_PRESET_USB_HUB_RS232:      LED_CFG_YELLOW();   break;
        case EVB2_CB_PRESET_USB_HUB_RS422:      LED_CFG_WHITE();    break;
    }
}


void board_IO_config(void)
{	
	// uINS ser0
    if (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_TRISTATE_UINS_IO ||
		(g_flashCfg->uinsComPort != EVB2_PORT_UINS0 &&
		 g_flashCfg->uinsAuxPort != EVB2_PORT_UINS0))
    {	// I/O tristate
		ioport_set_pin_input_mode(UART_INS_SER0_TXD_PIN, 0, 0);
		ioport_set_pin_input_mode(UART_INS_SER0_RXD_PIN, 0, 0);
	}
	else
	{
#ifdef CONF_BOARD_SERIAL_UINS_SER0
		ioport_set_pin_peripheral_mode(UART_INS_SER0_RXD_PIN, UART_INS_SER0_RXD_FLAGS);
		ioport_set_pin_peripheral_mode(UART_INS_SER0_TXD_PIN, UART_INS_SER0_TXD_FLAGS);
		serInit(EVB2_PORT_UINS0, 921600, NULL, NULL);
#endif		
	}
	
    // uINS ser1
    if (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE)
    {
	    #ifdef CONF_BOARD_SPI_UINS
	    ioport_set_pin_peripheral_mode(SPI_INS_MISO_PIN, SPI_INS_MISO_FLAGS);
	    ioport_set_pin_peripheral_mode(SPI_INS_MOSI_PIN, SPI_INS_MOSI_FLAGS);
	    ioport_set_pin_peripheral_mode(SPI_INS_SCLK_PIN, SPI_INS_SCLK_FLAGS);

	    //CS will be handled by GPIO
	    ioport_set_pin_output_mode(SPI_INS_CS_PIN, IOPORT_PIN_LEVEL_HIGH);
	    ioport_enable_pin(SPI_INS_CS_PIN);

	    //Indicate to uINS that SPI is requested
	    ioport_set_pin_output_mode(SPI_INS_EN, IOPORT_PIN_LEVEL_LOW);
	    ioport_enable_pin(SPI_INS_EN);
	    
	    //Setup data ready pin
	    ioport_set_pin_dir(INS_DATA_RDY_PIN_IDX, IOPORT_DIR_INPUT);
	    ioport_set_pin_mode(INS_DATA_RDY_PIN_IDX, IOPORT_MODE_PULLDOWN);

	    spiTouINS_init();
	    #endif
    }	
    else if (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_TRISTATE_UINS_IO ||
		(g_flashCfg->uinsComPort != EVB2_PORT_UINS1 &&
		 g_flashCfg->uinsAuxPort != EVB2_PORT_UINS1))
	{	// I/O tristate - (Enable pin and set as input)
	    ioport_set_pin_input_mode(UART_INS_SER1_TXD_PIN, 0, 0);
	    ioport_set_pin_input_mode(UART_INS_SER1_RXD_PIN, 0, 0);

	    ioport_set_pin_input_mode(SPI_INS_MISO_PIN, 0, 0);
	    ioport_set_pin_input_mode(SPI_INS_MOSI_PIN, 0, 0);
	    ioport_set_pin_input_mode(SPI_INS_SCLK_PIN, 0, 0);
	    ioport_set_pin_input_mode(SPI_INS_CS_PIN, 0, 0);
	    ioport_set_pin_input_mode(SPI_INS_EN, 0, 0);
	}
    else
    {
#ifdef CONF_BOARD_SERIAL_UINS_SER1
        ioport_set_pin_peripheral_mode(UART_INS_SER1_TXD_PIN, UART_INS_SER1_TXD_FLAGS);
        ioport_set_pin_peripheral_mode(UART_INS_SER1_RXD_PIN, UART_INS_SER1_RXD_FLAGS);
        serInit(EVB2_PORT_UINS1, 921600, NULL, NULL);
		ioport_set_pin_input_mode(SPI_INS_SCLK_PIN, 0, 0);
		ioport_set_pin_input_mode(SPI_INS_CS_PIN, 0, 0);
		ioport_set_pin_input_mode(SPI_INS_EN, 0, 0);
#endif
    }        

#ifdef CONF_BOARD_SERIAL_SP330
    if (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SP330_RS422)
    {   // RS422 mode
        ioport_set_pin_output_mode(SP330_N485_RXEN_PIN, IOPORT_PIN_LEVEL_LOW);      // Enable RS485 receiver
        ioport_set_pin_output_mode(SP330_485_N232_PIN, IOPORT_PIN_LEVEL_HIGH);      // Enable RS422/RS485 mode
    }
    else
    {   // RS232 mode
	    ioport_set_pin_output_mode(SP330_N485_RXEN_PIN, IOPORT_PIN_LEVEL_HIGH);     // Disable RS485 receiver
    	ioport_set_pin_output_mode(SP330_485_N232_PIN, IOPORT_PIN_LEVEL_LOW);       // Enable RS232 mode
    }
    serSetBaudRate(EVB2_PORT_SP330, g_flashCfg->h3sp330BaudRate);
#endif

#ifdef CONF_BOARD_SERIAL_XBEE
    if (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_XBEE_ENABLE)
    {   // XBee enabled
		ioport_set_pin_peripheral_mode(UART_XBEE_RXD_PIN, UART_XBEE_RXD_FLAGS);
		ioport_set_pin_peripheral_mode(UART_XBEE_TXD_PIN, UART_XBEE_TXD_FLAGS);
		ioport_set_pin_peripheral_mode(UART_XBEE_N_CTS_PIN, UART_XBEE_N_CTS_FLAGS);
		serInit(EVB2_PORT_XBEE, 115200, NULL, NULL);
#if 0
		ioport_set_pin_peripheral_mode(UART_XBEE_N_RTS_PIN, UART_XBEE_N_RTS_FLAGS);
		ioport_set_pin_peripheral_mode(UART_XBEE_DTR_PIN, UART_XBEE_DTR_FLAGS);
#else
		ioport_set_pin_output_mode(UART_XBEE_N_DTR_PIN, IOPORT_PIN_LEVEL_LOW);          // Low assert
		ioport_set_pin_output_mode(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_LOW);          // Low assert
#endif		
// 	    ioport_set_pin_output_mode(XBEE_VUSB_DISABLE_PIN, IOPORT_PIN_LEVEL_LOW);    // Enable VBUS
        ioport_set_pin_output_mode(XBEE_RST_PIN, IOPORT_PIN_LEVEL_HIGH);            // Reset off (low asserted)
        ioport_set_pin_output_mode(XBEE_SUPPLY_EN_PIN, IOPORT_PIN_LEVEL_HIGH);      // Enable supply
        g_status.evbStatus |= EVB_STATUS_XBEE_ENABLED;
    }
    else
    {   // XBee disabled
		ioport_set_pin_input_mode(UART_XBEE_TXD_PIN, 0, 0);
		ioport_set_pin_input_mode(UART_XBEE_RXD_PIN, 0, 0);
		ioport_set_pin_input_mode(UART_XBEE_N_CTS_PIN, 0, 0);
		ioport_set_pin_input_mode(UART_XBEE_N_DTR_PIN, 0, 0);
		ioport_set_pin_input_mode(UART_XBEE_N_RTS_PIN, 0, 0);
        ioport_set_pin_input_mode(XBEE_RST_PIN, 0, 0);

        ioport_set_pin_output_mode(XBEE_SUPPLY_EN_PIN, IOPORT_PIN_LEVEL_LOW);       // Disable supply
        g_status.evbStatus &= ~EVB_STATUS_XBEE_ENABLED;

        LED_OFF(LED_XBEE_RXD_PIN);
        LED_OFF(LED_XBEE_TXD_PIN);
    }
    serSetBaudRate(EVB2_PORT_XBEE, 115200);

    switch(g_flashCfg->cbPreset)
    {
    case EVB2_CB_PRESET_RS232_XBEE:
	case EVB2_CB_PRESET_USB_HUB_RS232:
		xbee_init();
	
    }
#endif

#ifdef CONF_BOARD_SPI_ATWINC_WIFI
    if (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_WIFI_ENABLE )
    {   // WiFi enabled
        wifi_enable(true);  // power on wifi.  Connect to hot spot and TCP socket
    }
    else
    {   // WiFi disabled
        wifi_enable(false); // power off wifi
    }
#else
    LED_OFF(LED_WIFI_TXD_PIN);
    LED_OFF(LED_WIFI_RXD_PIN);    
#endif

    // Reset default baud rates
    if (!(g_flashCfg->cbOptions&EVB2_CB_OPTIONS_TRISTATE_UINS_IO))
    {	// I/O not tristate		
		serSetBaudRate(EVB2_PORT_UINS0, 921600);
		if (!(g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE))
		{
			serSetBaudRate(EVB2_PORT_UINS1, 921600);
		}
    }
    serSetBaudRate(EVB2_PORT_BLE, 115200);
    serSetBaudRate(EVB2_PORT_XRADIO, g_flashCfg->h4xRadioBaudRate);
    serSetBaudRate(EVB2_PORT_GPIO_H8, g_flashCfg->h8gpioBaudRate);
#ifdef CONF_BOARD_CAN1
	if (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_CAN_ENABLE)
	{
		serSetBaudRate(EVB2_PORT_UINS1, 921600);
		CAN_init();
	}
#endif
}



void board_init(void)
{
	// Hardware Detection - PCB version
	ioport_set_pin_dir(EVB_HDW_DETECT_0_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(EVB_HDW_DETECT_1_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(EVB_HDW_DETECT_2_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(EVB_HDW_DETECT_0_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(EVB_HDW_DETECT_1_GPIO, IOPORT_MODE_PULLUP);
	ioport_set_pin_mode(EVB_HDW_DETECT_2_GPIO, IOPORT_MODE_PULLUP);	
	waitPinPullup();
	
	g_hdw_detect = ~(0xF8 |
	    ((ioport_get_pin_level(EVB_HDW_DETECT_0_GPIO)&0x1) <<0) |
	    ((ioport_get_pin_level(EVB_HDW_DETECT_1_GPIO)&0x1) <<1) |
	    ((ioport_get_pin_level(EVB_HDW_DETECT_2_GPIO)&0x1) <<2));
	
	// Disable hardware detect pullup/pulldown (tri-state pins)
	ioport_set_pin_input_mode(EVB_HDW_DETECT_0_GPIO, 0, 0);
	ioport_set_pin_input_mode(EVB_HDW_DETECT_1_GPIO, 0, 0);
	ioport_set_pin_input_mode(EVB_HDW_DETECT_2_GPIO, 0, 0);

	//////////////////////////////////////////////////////////////////////////
	// Init system clocks
	// If running on PLL, disable it so second PLL initialization does not hang device
	if((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_PLLA_CLK)
	{
		pmc_switch_mck_to_mainck(0);
		pll_disable(PLLA_ID);
	}	
	
    pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL);   // set SLCK to use external 32kHz crystal rather than internal 32kHz RC oscillator.
    sysclk_init();


#ifndef CONF_BOARD_KEEP_WATCHDOG_AT_INIT
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
#endif

#ifdef CONF_BOARD_CONFIG_MPU_AT_INIT
	_setup_memory_region();
#endif

	SCB_EnableICache(); 

#if CONF_BOARD_ENABLE_DCACHE == 1
	SCB_EnableDCache();
#endif

#ifdef CONF_BOARD_ENABLE_TCM_AT_INIT
	/* TCM Configuration */
	EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB 
					| EEFC_FCR_FARG(8));
	EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_SGPB
					| EEFC_FCR_FARG(7));
	tcm_enable();
#if defined(__GNUC__)
	volatile char *dst = &_sitcm;
	volatile char *src = &_itcm_lma;
	/* copy code_TCM from flash to ITCM */
	while(dst < &_eitcm){
		*dst++ = *src++;
	}
#endif
#else
	/* TCM Configuration */
	EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB 
					| EEFC_FCR_FARG(8));
	EFC->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB 
					| EEFC_FCR_FARG(7));
	
	tcm_disable();
#endif

	// Real-time timer
	time_init();

	/* Initialize IOPORTs */
	ioport_init();

    //////////////////////////////////////////////////////////////////////////
    // LEDs - default to ALL on
	ioport_set_pin_output_mode(LED_CFG_RED_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_output_mode(LED_CFG_GRN_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_output_mode(LED_CFG_BLU_PIN, IOPORT_PIN_LEVEL_LOW);	
	ioport_set_pin_output_mode(LED_LOG_RED_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_output_mode(LED_LOG_GRN_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_output_mode(LED_LOG_BLU_PIN, IOPORT_PIN_LEVEL_LOW);

	ioport_set_pin_output_mode(LED_INS_RXD_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_output_mode(LED_INS_TXD_PIN, IOPORT_PIN_LEVEL_LOW);    
	ioport_set_pin_output_mode(LED_XBEE_RXD_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_output_mode(LED_XBEE_TXD_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_output_mode(LED_WIFI_RXD_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_output_mode(LED_WIFI_TXD_PIN, IOPORT_PIN_LEVEL_LOW);

#if 0
	// Turn off LEDs
    LED_CFG_OFF();
    LED_LOG_OFF();

	LED_OFF(LED_INS_RXD_PIN);
	LED_OFF(LED_INS_TXD_PIN);
	LED_OFF(LED_XBEE_RXD_PIN);
	LED_OFF(LED_XBEE_TXD_PIN);
	LED_OFF(LED_WIFI_RXD_PIN);
	LED_OFF(LED_WIFI_TXD_PIN);
#endif

    //////////////////////////////////////////////////////////////////////////
    // Push Buttons
	ioport_set_pin_input_mode(BUTTON_CFG_PIN, BUTTON_CFG_FLAGS,	BUTTON_CFG_SENSE);
	ioport_set_pin_input_mode(BUTTON_LOG_PIN, BUTTON_LOG_FLAGS,	BUTTON_LOG_SENSE);
    
    //////////////////////////////////////////////////////////////////////////
    // Serial Ports

    // UINS ser0 and ser1 - In board_IO_init() in main.cpp

#ifdef CONF_BOARD_SERIAL_EXT_RADIO      // External Radio
    ioport_set_pin_peripheral_mode(UART_EXT_RADIO_RXD_PIN, UART_EXT_RADIO_RXD_FLAGS);
    ioport_set_pin_peripheral_mode(UART_EXT_RADIO_TXD_PIN, UART_EXT_RADIO_TXD_FLAGS);
    serInit(EVB2_PORT_XRADIO, g_flashCfg->h4xRadioBaudRate, NULL, NULL);
//     ioport_set_pin_dir(EXT_RADIO_RST, IOPORT_DIR_OUTPUT);
//     ioport_set_pin_level(EXT_RADIO_RST, IOPORT_PIN_LEVEL_HIGH);         // Low assert
#endif

#ifdef CONF_BOARD_SERIAL_ATWINC_BLE     // ATWINC3400 Bluetooth
    ioport_set_pin_peripheral_mode(UART_BTLE_RXD_PIN, UART_BTLE_RXD_FLAGS);
    ioport_set_pin_peripheral_mode(UART_BTLE_TXD_PIN, UART_BTLE_TXD_FLAGS);
    ioport_set_pin_peripheral_mode(UART_BTLE_CTS_PIN, UART_XBEE_N_CTS_FLAGS);
    ioport_set_pin_peripheral_mode(UART_BTLE_RTS_PIN, UART_XBEE_N_RTS_FLAGS);
    serInit(EVB2_PORT_BLE, 115200, NULL, NULL);
#endif

#ifdef CONF_BOARD_SERIAL_SP330          // RS232/RS422/RS485 converter
    ioport_set_pin_peripheral_mode(UART_SP330_RXD_PIN, UART_SP330_RXD_FLAGS);
    ioport_set_pin_peripheral_mode(UART_SP330_TXD_PIN, UART_SP330_TXD_FLAGS);
    serInit(EVB2_PORT_SP330, g_flashCfg->h3sp330BaudRate, NULL, NULL);
	ioport_set_pin_output_mode(SP330_NSLEW_PIN, IOPORT_PIN_LEVEL_HIGH);         // Don't limit data rate to less than 250 Kbps
	ioport_set_pin_output_mode(SP330_NFULL_DPLX_PIN, IOPORT_PIN_LEVEL_LOW);    // RS485 full duplex (RS232 N/A)
	ioport_set_pin_output_mode(SP330_NSHDN_PIN, IOPORT_PIN_LEVEL_HIGH);         // Enable device
#endif

#ifdef CONF_BOARD_SERIAL_GPIO_H8       // GPIO TTL
	ioport_set_pin_output_mode(GPIO_UART_INV_PIN, IOPORT_PIN_LEVEL_LOW);    // Non-inverting	
    ioport_set_pin_peripheral_mode(GPIO_H8_UART_RXD_PIN, GPIO_H8_UART_RXD_FLAGS);
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
    ioport_set_pin_peripheral_mode(GPIO_H8_UART_TXD_PIN, GPIO_H8_UART_TXD_FLAGS);
    serInit(EVB2_PORT_GPIO_H8, g_flashCfg->h8gpioBaudRate, NULL, NULL);
	
// 	char buf[3] = "123";
// 	serWrite(EVB2_PORT_GPIO_H8, buf, 3, NULL);
#endif
    

    //////////////////////////////////////////////////////////////////////////
    // SPI Interface
#ifdef CONF_BOARD_SPI_ATWINC_WIFI       // ATWINC WIFI
	ioport_set_pin_peripheral_mode(SPI_WIFI_MISO_PIN, SPI_WIFI_MISO_FLAGS);
	ioport_set_pin_peripheral_mode(SPI_WIFI_MOSI_PIN, SPI_WIFI_MOSI_FLAGS);
	ioport_set_pin_peripheral_mode(SPI_WIFI_SCLK_PIN, SPI_WIFI_SCLK_FLAGS);
	ioport_set_pin_peripheral_mode(SPI_WIFI_CS_PIN, SPI_WIFI_CS_FLAGS);
	ioport_set_pin_output_mode(WIFI_RST_PIN, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_output_mode(WIFI_CHIPEN_PIN, IOPORT_PIN_LEVEL_LOW);          // Disable
	ioport_set_pin_input_mode(WIFI_IRQN_PIN, 0,	0);
#else
	ioport_set_pin_output_mode(WIFI_CHIPEN_PIN, IOPORT_PIN_LEVEL_LOW);          // Disable
#endif

    //////////////////////////////////////////////////////////////////////////
    // I2C Interface
#ifdef CONF_BOARD_I2C_UINS              // UINS I2C
	ioport_set_pin_peripheral_mode(I2C_0_SCL_PIN, I2C_0_SCL_FLAGS);
	ioport_set_pin_peripheral_mode(I2C_0_SDA_PIN, I2C_0_SDA_FLAGS);
#endif

#ifdef CONF_BOARD_CAN1
	/* Configure the CAN1 TX and RX pin. */
	ioport_set_pin_peripheral_mode(CAN_RXD_PIN, CAN_RXD_FLAGS);
	ioport_set_pin_peripheral_mode(CAN_TXD_PIN, CAN_TXD_FLAGS);
#endif

    //////////////////////////////////////////////////////////////////////////
    // SD Card Interface
#ifdef CONF_BOARD_SD_MMC_HSMCI
	/* Configure HSMCI pins */
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCCDA_GPIO, PIN_HSMCI_MCCDA_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCCK_GPIO, PIN_HSMCI_MCCK_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA0_GPIO, PIN_HSMCI_MCDA0_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA1_GPIO, PIN_HSMCI_MCDA1_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA2_GPIO, PIN_HSMCI_MCDA2_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA3_GPIO, PIN_HSMCI_MCDA3_FLAGS);

    /* Configure SD/MMC card detect pin */
	ioport_set_pin_input_mode(SD_MMC_0_CD_GPIO, SD_MMC_0_CD_FLAGS, 0);
#endif

    

    //////////////////////////////////////////////////////////////////////////
    /// Unused / Future Use 
    //////////////////////////////////////////////////////////////////////////
	
#ifdef CONF_BOARD_TWIHS0
	ioport_set_pin_peripheral_mode(TWIHS0_DATA_GPIO, TWIHS0_DATA_FLAGS);
	ioport_set_pin_peripheral_mode(TWIHS0_CLK_GPIO, TWIHS0_CLK_FLAGS);
#endif

#ifdef CONF_BOARD_CAN0
	/* Configure the CAN0 TX and RX pins. */
	ioport_set_pin_peripheral_mode(PIN_CAN0_RX_IDX, PIN_CAN0_RX_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_CAN0_TX_IDX, PIN_CAN0_TX_FLAGS);
	/* Configure the transiver0 RS & EN pins. */
	ioport_set_pin_dir(PIN_CAN0_TR_RS_IDX, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_CAN0_TR_EN_IDX, IOPORT_DIR_OUTPUT);
#endif

#ifdef CONF_BOARD_SPI
	ioport_set_pin_peripheral_mode(SPI0_MISO_GPIO, SPI0_MISO_FLAGS);
	ioport_set_pin_peripheral_mode(SPI0_MOSI_GPIO, SPI0_MOSI_FLAGS);
	ioport_set_pin_peripheral_mode(SPI0_SPCK_GPIO, SPI0_SPCK_FLAGS);

#ifdef CONF_BOARD_SPI_NPCS0
	ioport_set_pin_peripheral_mode(SPI0_NPCS0_GPIO, SPI0_NPCS0_FLAGS);
#endif

#ifdef CONF_BOARD_SPI_NPCS1
	ioport_set_pin_peripheral_mode(SPI0_NPCS1_GPIO, SPI0_NPCS1_FLAGS);
#endif
#endif

#ifdef CONF_BOARD_QSPI
	ioport_set_pin_peripheral_mode(QSPI_QSCK_GPIO, QSPI_QSCK_FLAGS);
	ioport_set_pin_peripheral_mode(QSPI_QCS_GPIO, QSPI_QCS_FLAGS);
	ioport_set_pin_peripheral_mode(QSPI_QIO0_GPIO, QSPI_QIO0_FLAGS);
	ioport_set_pin_peripheral_mode(QSPI_QIO1_GPIO, QSPI_QIO1_FLAGS);
	ioport_set_pin_peripheral_mode(QSPI_QIO2_GPIO, QSPI_QIO2_FLAGS);
	ioport_set_pin_peripheral_mode(QSPI_QIO3_GPIO, QSPI_QIO3_FLAGS);
#endif

#ifdef CONF_BOARD_PWM_LED0
	/* Configure PWM LED0 pin */
	ioport_set_pin_peripheral_mode(PIN_PWM_LED0_GPIO, PIN_PWM_LED0_FLAGS);
#endif

#ifdef CONF_BOARD_PWM_LED1
	/* Configure PWM LED1 pin */
	ioport_set_pin_peripheral_mode(PIN_PWM_LED1_GPIO, PIN_PWM_LED1_FLAGS);
#endif

#ifdef CONF_BOARD_USART_SCK
	/* Configure USART synchronous communication SCK pin */
	ioport_set_pin_peripheral_mode(PIN_USART0_SCK_IDX,PIN_USART0_SCK_FLAGS);
#endif

#ifdef CONF_BOARD_USART_CTS
	/* Configure USART synchronous communication CTS pin */
	ioport_set_pin_peripheral_mode(PIN_USART0_CTS_IDX,PIN_USART0_CTS_FLAGS);
#endif

#ifdef CONF_BOARD_USART_RTS
	/* Configure USART RTS pin */
	ioport_set_pin_peripheral_mode(PIN_USART0_RTS_IDX, PIN_USART0_RTS_FLAGS);
#endif

#ifdef CONF_BOARD_ILI9488
	/**LCD pin configure on EBI*/
	pio_configure(PIN_EBI_RESET_PIO, PIN_EBI_RESET_TYPE, PIN_EBI_RESET_MASK, PIN_EBI_RESET_ATTRI);
	pio_configure(PIN_EBI_CDS_PIO, PIN_EBI_CDS_TYPE, PIN_EBI_CDS_MASK, PIN_EBI_CDS_ATTRI);
	pio_configure(PIN_EBI_DATAL_PIO, PIN_EBI_DATAL_TYPE, PIN_EBI_DATAL_MASK, PIN_EBI_DATAL_ATTRI);
	pio_configure(PIN_EBI_DATAH_0_PIO, PIN_EBI_DATAH_0_TYPE, PIN_EBI_DATAH_0_MASK, PIN_EBI_DATAH_0_ATTRI);
	pio_configure(PIN_EBI_DATAH_1_PIO, PIN_EBI_DATAH_1_TYPE, PIN_EBI_DATAH_1_MASK, PIN_EBI_DATAH_1_ATTRI);
	pio_configure(PIN_EBI_NWE_PIO, PIN_EBI_NWE_TYPE, PIN_EBI_NWE_MASK, PIN_EBI_NWE_ATTRI);
	pio_configure(PIN_EBI_NRD_PIO, PIN_EBI_NRD_TYPE, PIN_EBI_NRD_MASK, PIN_EBI_NRD_ATTRI);
	pio_configure(PIN_EBI_CS_PIO, PIN_EBI_CS_TYPE, PIN_EBI_CS_MASK, PIN_EBI_CS_ATTRI);
	pio_configure(PIN_EBI_BACKLIGHT_PIO, PIN_EBI_BACKLIGHT_TYPE, PIN_EBI_BACKLIGHT_MASK, PIN_EBI_BACKLIGHT_ATTRI);
	pio_set(PIN_EBI_BACKLIGHT_PIO, PIN_EBI_BACKLIGHT_MASK);
#endif

#if (defined CONF_BOARD_USB_PORT)
# if defined(CONF_BOARD_USB_VBUS_DETECT)
	ioport_set_pin_dir(USB_VBUS_PIN, IOPORT_DIR_INPUT);
# endif
# if defined(CONF_BOARD_USB_ID_DETECT)
	ioport_set_pin_dir(USB_ID_PIN, IOPORT_DIR_INPUT);
# endif
#endif

#ifdef CONF_BOARD_SDRAMC
	pio_configure_pin(SDRAM_BA0_PIO, SDRAM_BA0_FLAGS);
	pio_configure_pin(SDRAM_SDCK_PIO, SDRAM_SDCK_FLAGS);
	pio_configure_pin(SDRAM_SDCKE_PIO, SDRAM_SDCKE_FLAGS);
	pio_configure_pin(SDRAM_SDCS_PIO, SDRAM_SDCS_FLAGS);
	pio_configure_pin(SDRAM_RAS_PIO, SDRAM_RAS_FLAGS);
	pio_configure_pin(SDRAM_CAS_PIO, SDRAM_CAS_FLAGS);
	pio_configure_pin(SDRAM_SDWE_PIO, SDRAM_SDWE_FLAGS);
	pio_configure_pin(SDRAM_NBS0_PIO, SDRAM_NBS0_FLAGS);
	pio_configure_pin(SDRAM_NBS1_PIO, SDRAM_NBS1_FLAGS);
	pio_configure_pin(SDRAM_A2_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A3_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A4_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A5_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A6_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A7_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A8_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A9_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A10_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_A11_PIO, SDRAM_A_FLAGS);  
	pio_configure_pin(SDRAM_SDA10_PIO, SDRAM_SDA10_FLAGS);
	pio_configure_pin(SDRAM_D0_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D1_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D2_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D3_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D4_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D5_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D6_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D7_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D8_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D9_PIO, SDRAM_D_FLAGS);  
	pio_configure_pin(SDRAM_D10_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D11_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D12_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D13_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D14_PIO, SDRAM_D_FLAGS); 
	pio_configure_pin(SDRAM_D15_PIO, SDRAM_D_FLAGS); 
	
	MATRIX->CCFG_SMCNFCS = CCFG_SMCNFCS_SDRAMEN;
#endif

#ifdef CONF_BOARD_ILI9488
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	pio_configure_pin(LCD_SPI_BACKLIGHT_PIO, LCD_SPI_BACKLIGHT_FLAGS);
	pio_set_pin_high(LCD_SPI_BACKLIGHT_PIO);

#endif
}
