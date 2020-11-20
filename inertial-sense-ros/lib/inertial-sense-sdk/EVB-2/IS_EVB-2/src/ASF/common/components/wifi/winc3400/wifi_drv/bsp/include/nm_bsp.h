/**
 *
 * \file
 *
 * \brief This module contains WINC3400 BSP APIs declarations.
 *
 * Copyright (c) 2017-2018 Microchip Technology Inc. and its subsidiaries.
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

/**@defgroup nm_bsp BSP
 */

/**@defgroup  BSPDefine Defines
 * @ingroup nm_bsp
 * @{
 */
#ifndef _NM_BSP_H_
#define _NM_BSP_H_

#define NMI_API
/*!< 
*        Attribute used to define memory section to map Functions in host memory.
*/
#define CONST const

#ifndef NULL
#define NULL ((void*)0)
#endif
/*!< 
*    Void Pointer to '0' in case of NULL is not defined. 
*/


#define BSP_MIN(x,y) ((x)>(y)?(y):(x))
/*!< 
*     Computes the minimum of \b x and \b y.
*/

 //@}

/**@defgroup  DataT  Enumeration/Typedefs
 * @ingroup nm_bsp
 * @{
 */
 
 /*!
*     Used for code portability.
*/

/*!
 * @typedef      void (*tpfNmBspIsr) (void);
 * @brief           Pointer to function.\n
 *                     Used as a data type of ISR function registered by \ref nm_bsp_register_isr
 * @return         None
 */
typedef void (*tpfNmBspIsr)(void);

  /*!
 * @ingroup DataTypes
 * @typedef      unsigned char	uint8;
 * @brief        Range of values between 0 to 255
 */
typedef unsigned char	uint8;

 /*!
 * @ingroup DataTypes
 * @typedef      unsigned short	uint16;
 * @brief        Range of values between 0 to 65535
 */
typedef unsigned short	uint16;

 /*!
 * @ingroup Data Types
 * @typedef      unsigned long	uint32;
 * @brief        Range of values between 0 to 4294967295
 */ 
typedef unsigned long	uint32;


  /*!
 * @ingroup Data Types
 * @typedef      signed char		sint8;
 * @brief        Range of values between -128 to 127
 */
typedef signed char		sint8;

 /*!
 * @ingroup DataTypes
 * @typedef      signed short	sint16;
 * @brief        Range of values between -32768 to 32767
 */
typedef signed short	sint16;

  /*!
 * @ingroup DataTypes
 * @typedef      signed long		sint32;
 * @brief        Range of values between -2147483648 to 2147483647
 */

typedef signed long		sint32;
 //@}

#ifndef CORTUS_APP


#ifdef __cplusplus
extern "C"{
#endif

/** \defgroup BSPAPI Functions
 *   @ingroup nm_bsp
 * @{
 */

/*!
 * @fn           sint8 nm_bsp_init(void);
 *               Initialization for BSP such as Reset and Chip Enable Pins for WINC, delays, register ISR, enable/disable IRQ for WINC, ...etc. You must use this function in the head of your application to 
 *               enable WINC and Host Driver to communicate with each other. 
 * @note         Implementation of this function is host dependent.
 * @warning      Omission will lead to unavailability of host communication.\n
 *  
 * @return       The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

 */
sint8 nm_bsp_init(void);
/*!
 * @fn           sint8 nm_bsp_deinit(void);
 *               De-initialization for BSP (\e Board \e Support \e Package)
 * @pre          Initialize \ref nm_bsp_init first
 * @note         Implementation of this function is host dependent.
 * @warning      Omission may lead to unknown behavior in case of soft reset.\n
 * @see          nm_bsp_init               
 * @return      The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

 */
sint8 nm_bsp_deinit(void);
 /*!
 * @fn           void nm_bsp_reset(void);
 *               Resetting NMC1500 SoC by setting CHIP_EN and RESET_N signals low, then after specific delay the function will put CHIP_EN high then RESET_N high,
 *               for the timing between signals please review the WINC data-sheet
 * @pre          Initialize \ref nm_bsp_init first
 * @note         Implementation of this function is host dependent and called by HIF layer.
 * @see          nm_bsp_init    
 * @return       None

 */
void nm_bsp_reset(void);
/*!
 * @fn           void nm_bsp_sleep(uint32);
 *               Sleep in units of milliseconds.\n
 *               This function is used by HIF Layer as required in various situations. 
 * @brief   
 * @param [in]   u32TimeMsec
 *               Time unit in milliseconds
 * @pre          Initialize \ref nm_bsp_init first
 * @warning      Maximum value must nor exceed 4294967295 milliseconds which is equal to 4294967.295 seconds.\n
 * @note         Implementation of this function is host dependent.
 * @see           nm_bsp_init               
 * @return       None
 */
void nm_bsp_sleep(uint32 u32TimeMsec);
/*!
 * @fn           void nm_bsp_register_isr(tpfNmBspIsr);
 *               Register ISR (Interrupt Service Routine) in the initialization of HIF (Host Interface) Layer. 
 *               When the interrupt trigger the BSP layer should call the pfisr function once inside the interrupt. 
 * @param [in]   pfIsr
 *               Pointer to ISR handler in HIF
 * @warning      Make sure that ISR for IRQ pin for WINC is disabled by default in your implementation.
 * @note         Implementation of this function is host dependent and called by HIF layer.
 * @see          tpfNmBspIsr
 * @return       None

 */
void nm_bsp_register_isr(tpfNmBspIsr pfIsr);
/*!
 * @fn           void nm_bsp_interrupt_ctrl(uint8);
 *               Synchronous enable/disable interrupts function
 * @brief        Enable/Disable interrupts
 * @param [in]   u8Enable
 *               '0' disable interrupts. '1' enable interrupts 
 * @see          tpfNmBspIsr           
 * @note         Implementation of this function is host dependent and called by HIF layer.
 * @return       None

 */
void nm_bsp_interrupt_ctrl(uint8 u8Enable);
  /**@}*/

#ifdef __cplusplus
}
#endif

#endif

/**
 * @addtogroup BSPDefine
 * @{
 */

#ifdef _NM_BSP_BIG_END
/*! Switch endianness of 32bit word (In the case that Host is BE) */
#define NM_BSP_B_L_32(x) \
((((x) & 0x000000FF) << 24) + \
(((x) & 0x0000FF00) << 8)  + \
(((x) & 0x00FF0000) >> 8)   + \
(((x) & 0xFF000000) >> 24))
/*! Switch endianness of 16bit word (In the case that Host is BE) */
#define NM_BSP_B_L_16(x) \
((((x) & 0x00FF) << 8) + \
(((x)  & 0xFF00) >> 8))
#else
/*! Retain endianness of 32bit word (In the case that Host is LE) */
#define NM_BSP_B_L_32(x)  (x)
/*! Retain endianness of 16bit word (In the case that Host is LE) */
#define NM_BSP_B_L_16(x)  (x)
#endif

/**@}
 */

#endif	/*_NM_BSP_H_*/
