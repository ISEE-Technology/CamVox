/**
 *
 * \file
 *
 * \brief This module contains M2M host interface APIs implementation.
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

#include "common/include/nm_common.h"
#include "driver/source/nmbus.h"
#include "driver/source/nmdrv.h"
#include "bsp/include/nm_bsp.h"
#include "m2m_hif.h"
#include "driver/include/m2m_types.h"
#include "driver/source/nmasic.h"
#include "driver/include/m2m_periph.h"

#if (defined NM_EDGE_INTERRUPT)&&(defined NM_LEVEL_INTERRUPT)
#error "only one type of interrupt NM_EDGE_INTERRUPT,NM_LEVEL_INTERRUPT"
#endif

#if !((defined NM_EDGE_INTERRUPT)||(defined NM_LEVEL_INTERRUPT))
#error "define interrupt type NM_EDGE_INTERRUPT,NM_LEVEL_INTERRUPT"
#endif

#ifndef CORTUS_APP
#define NMI_AHB_DATA_MEM_BASE  0x30000
#define NMI_AHB_SHARE_MEM_BASE 0xd0000

#define WIFI_HOST_RCV_CTRL_0	(0x1070)
#define WIFI_HOST_RCV_CTRL_1	(0x1084)
#define WIFI_HOST_RCV_CTRL_2    (0x1078)
#define WIFI_HOST_RCV_CTRL_3    (0x106c)
#define WAKE_VALUE				(0x5678)
#define SLEEP_VALUE				(0x4321)
#define WAKE_REG				(0x1074)

#define INTERRUPT_CORTUS_0_3000D0	(0x10a8)
#define INTERRUPT_CORTUS_1_3000D0	(0x10ac)
#define INTERRUPT_CORTUS_2_3000D0	(0x10b0)
#define INTERRUPT_CORTUS_3_3000D0	(0x10b4)


static volatile uint8 gu8ChipMode = 0;
static volatile uint8 gu8ChipSleep = 0;
static volatile uint8 gu8HifSizeDone = 0;
static volatile uint8 gu8Interrupt = 0;
static volatile uint8 gu8Yield = 0;

/*
	Special codes for managing HIF restriction to OTA rollback/switch only
*/
#define HIF_OTA_RB_ONLY				0xFFFF
#define HIFCODE_OTA_RB				((M2M_REQ_GROUP_OTA << 8) | M2M_OTA_REQ_ROLLBACK)
#define HIFCODE_OTA_SW				((M2M_REQ_GROUP_OTA << 8) | M2M_OTA_REQ_SWITCH_FIRMWARE)
#define HIFCODE_SSL_WRITECERT		((M2M_REQ_GROUP_SSL << 8) | M2M_SSL_REQ_WRITE_OWN_CERTS)
#define HIFCODE_WIFI_PASSIVESCAN	((M2M_REQ_GROUP_WIFI << 8) | M2M_WIFI_REQ_PASSIVE_SCAN)
/*
	List of new HIF messages (since last HIF major increase)
	Each entry is formed of ((GroupId << 8) | OpCode)
	Additionally, entry 0 used to indicate OTA RB/SW only.
*/
#define NEW_HIF_LIST HIF_OTA_RB_ONLY, HIFCODE_SSL_WRITECERT, HIFCODE_WIFI_PASSIVESCAN
/*
	Array of HIF messages which are not supported by Firmware.
	During hif_init() this array is rebased using an offset determined by Firmware HIF level.
*/
static uint16 gau16HifBlacklist[] = {NEW_HIF_LIST};
#define HIF_BLACKLIST_SZ (sizeof(gau16HifBlacklist)/sizeof(gau16HifBlacklist[0]))
static uint8 gu8HifBlOffset = 0;

tpfHifCallBack pfWifiCb = NULL;		/*!< pointer to Wi-Fi call back function */
tpfHifCallBack pfIpCb  = NULL;		/*!< pointer to Socket call back function */
tpfHifCallBack pfOtaCb = NULL;		/*!< pointer to OTA call back function */
tpfHifCallBack pfSigmaCb = NULL;
tpfHifCallBack pfHifCb = NULL;
tpfHifCallBack pfSSLCb = NULL;

static void isr(void)
{
	gu8Interrupt++;
#ifdef NM_LEVEL_INTERRUPT
	nm_bsp_interrupt_ctrl(0);
#endif
}

static sint8 hif_set_rx_done(void)
{
	uint32 reg;
	sint8 ret = M2M_SUCCESS;
#ifdef NM_EDGE_INTERRUPT
	nm_bsp_interrupt_ctrl(1);
#endif


	if (ISNMC3400(nmi_get_chipid())) {
		ret = nm_write_reg(INTERRUPT_CORTUS_0_3000D0,1);
		if(ret != M2M_SUCCESS)goto ERR1;
	} else 	{
		ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0,&reg);
		if(ret != M2M_SUCCESS)goto ERR1;
		//reg &= ~(1<<0);

		/* Set RX Done */
		reg |= (1<<1);		
		ret = nm_write_reg(WIFI_HOST_RCV_CTRL_0,reg);
		if(ret != M2M_SUCCESS)goto ERR1;
	}
	
#ifdef NM_LEVEL_INTERRUPT
	nm_bsp_interrupt_ctrl(1);
#endif
ERR1:
	return ret;
	
}
/**
*	@fn			static void m2m_hif_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr)
*	@brief		WiFi call back function
*	@param [in]	u8OpCode
*					HIF Opcode type.
*	@param [in]	u16DataSize
*					HIF data length.
*	@param [in]	u32Addr
*					HIF address.
*	@author
*	@date
*	@version	1.0
*/
static void m2m_hif_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr)
{


}
/**
*	@fn		NMI_API sint8 hif_chip_wake(void);
*	@brief	To Wakeup the chip.
*    @return		The function shall return ZERO for successful operation and a negative value otherwise. 
*/

sint8 hif_chip_wake(void)
{
	sint8 ret = M2M_SUCCESS;
	if(gu8ChipSleep == 0)
	{
		if((gu8ChipMode == M2M_PS_DEEP_AUTOMATIC)||(gu8ChipMode == M2M_PS_MANUAL))
		{
			ret = nm_clkless_wake();
			if(ret != M2M_SUCCESS)goto ERR1;
			ret = nm_write_reg(WAKE_REG, WAKE_VALUE);
			if(ret != M2M_SUCCESS)goto ERR1;
		}
		else
		{
		}
	}
	gu8ChipSleep++;
ERR1:
	return ret;
}
/*!
@fn	\
	NMI_API void hif_set_sleep_mode(uint8 u8Pstype);

@brief
	Set the sleep mode of the HIF layer.

@param [in]	u8Pstype
				Sleep mode.
				
@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/

void hif_set_sleep_mode(uint8 u8Pstype)
{
	gu8ChipMode = u8Pstype;
}
/*!
@fn	\
	NMI_API uint8 hif_get_sleep_mode(void);

@brief
	Get the sleep mode of the HIF layer.
				
@return		
	The function SHALL return the sleep mode of the HIF layer.
*/

uint8 hif_get_sleep_mode(void)
{
	return gu8ChipMode;
}
/**
*	@fn		NMI_API sint8 hif_chip_sleep(void);
*	@brief	To make the chip sleep.
*    @return		The function shall return ZERO for successful operation and a negative value otherwise. 
*/

sint8 hif_chip_sleep(void)
{
	sint8 ret = M2M_SUCCESS;

	if(gu8ChipSleep >= 1)
	{
		gu8ChipSleep--;
	}
	
	if(gu8ChipSleep == 0)
	{
		if((gu8ChipMode == M2M_PS_DEEP_AUTOMATIC)||(gu8ChipMode == M2M_PS_MANUAL))
		{
			uint32 reg = 0;
			ret = nm_write_reg(WAKE_REG, SLEEP_VALUE);
			if(ret != M2M_SUCCESS)goto ERR1;
			/* Clear bit 1 */
			ret = nm_read_reg_with_ret(0x1, &reg);
			if(ret != M2M_SUCCESS)goto ERR1;
			if(reg&0x2)
			{
				reg &=~(1 << 1);
				ret = nm_write_reg(0x1, reg);
			}
		}
		else
		{
		}
	}
ERR1:
	return ret;
}
/**
*   @fn		NMI_API sint8 hif_init(void * arg);
*   @brief	To initialize HIF layer.
*   @param [in]	arg
*				Pointer to the arguments.
*   @return		The function shall return ZERO for successful operation and a negative value otherwise. 
*/

sint8 hif_init(void * arg)
{
	pfWifiCb = NULL;
	pfIpCb = NULL;
	
	gu8ChipSleep = 0;
	gu8ChipMode = M2M_NO_PS;

	gu8Interrupt = 0;
	nm_bsp_register_isr(isr);

	gu8HifBlOffset = 0;
	hif_register_cb(M2M_REQ_GROUP_HIF,m2m_hif_cb);

	return M2M_SUCCESS;
}
/**
*	@fn		NMI_API sint8 hif_deinit(void * arg);
*	@brief	To Deinitialize HIF layer.
*    @param [in]	arg
*				Pointer to the arguments.
*    @return		The function shall return ZERO for successful operation and a negative value otherwise. 
*/
sint8 hif_deinit(void * arg)
{
	sint8 ret = M2M_SUCCESS;
#if 0
	uint32 reg = 0, cnt=0;
	while (reg != M2M_DISABLE_PS)
	{
		nm_bsp_sleep(1);
		reg = nm_read_reg(STATE_REG);
		if(++cnt > 1000)
		{
			M2M_DBG("failed to stop power save\n");
			break;
		}
	}
#endif
	ret = hif_chip_wake();

	gu8ChipMode = 0;
	gu8ChipSleep = 0;
	gu8HifSizeDone = 0;
	gu8Interrupt = 0;

	pfWifiCb = NULL;
	pfIpCb  = NULL;
	pfOtaCb = NULL;
	pfHifCb = NULL;

	return ret;
}
/**
*	@fn			sint8 hif_check_compatibility(uint16 u16HifInfo);
*	@brief
*				To check the compatibility of an image with the current driver.
*	@param [in]	u16HifInfo
*				HIF info of image to be checked.
*	@return		The function shall return ZERO for compatible image and a negative value otherwise.
*/
sint8 hif_check_compatibility(uint16 u16HifInfo)
{
	sint8 ret = M2M_ERR_FW_VER_MISMATCH;
	if((M2M_GET_HIF_BLOCK(u16HifInfo) == M2M_HIF_BLOCK_VALUE) && (M2M_GET_HIF_MAJOR(u16HifInfo) == M2M_HIF_MAJOR_VALUE))
	{
		ret = M2M_SUCCESS;
	}
	return ret;
}
/**
*	@fn			sint8 hif_enable_access(void);
*	@brief
*				To enable access to HIF layer, based on HIF level of Firmware.
*				This function reads HIF level directly from a register written by Firmware.
*	@return		The function shall return ZERO for full match operation and a negative value if operation is restricted.
*/
sint8 hif_enable_access(void)
{
	sint8 ret = M2M_SUCCESS;
	uint16 fw_hif_info = 0;

	ret = nm_get_hif_info(&fw_hif_info, NULL);
	if(ret == M2M_SUCCESS)
	{
		ret = hif_check_compatibility(fw_hif_info);
		if(ret == M2M_SUCCESS)
		{
			switch(M2M_GET_HIF_MINOR(fw_hif_info))
			{
				case 0:
					gu8HifBlOffset = 1;
				break;
				case 1:
					gu8HifBlOffset = 2;
				break;
				case 2:
					gu8HifBlOffset = 2;
				break;
				case 3:
					gu8HifBlOffset = 3;
				break;
				// Additional case to be added each time hif minor increments.
				// All additional cases to be removed in the event of a hif major increment.
				// Default catches all cases in which hif minor is greater in Firmware than in Driver.
				default:
					gu8HifBlOffset = HIF_BLACKLIST_SZ;
				break;
			}
		}
		else
		{
			gu8HifBlOffset = 0;
			M2M_ERR("HIF access limited to OTA Switch/Rollback only\n");
		}
	}
	return ret;
}
/**
*	@fn			sint8 hif_check_code(uint8 u8Gid, uint8 u8OpCode);
*	@brief
*				To check that a particular hif message is supported with the current driver/firmware pair.
*	@param [in]	u8Gid
*				Group ID.
*	@param [in]	u8Opcode
*				Operation ID.
*	@return		The function shall return ZERO for support and a negative value otherwise.
*/
sint8 hif_check_code(uint8 u8Gid, uint8 u8OpCode)
{
	uint8 u8BlId;
	uint16 u16HifCode = ((uint16)u8Gid<<8) | u8OpCode;
	if((u16HifCode == HIFCODE_OTA_RB) || (u16HifCode == HIFCODE_OTA_SW))
	{
		return M2M_SUCCESS;
	}
	if(gu8HifBlOffset == 0)
	{
		M2M_ERR("HIF OTA rb/sw only\n");
		return M2M_ERR_SEND;
	}
	for(u8BlId = gu8HifBlOffset; u8BlId < HIF_BLACKLIST_SZ; u8BlId++)
	{
		if(u16HifCode == gau16HifBlacklist[u8BlId])
		{
			M2M_ERR("HIF message unsupported\n");
			return M2M_ERR_SEND;
		}
	}
	return M2M_SUCCESS;
}
/**
*	@fn		NMI_API sint8 hif_send(uint8 u8Gid,uint8 u8Opcode,uint8 *pu8CtrlBuf,uint16 u16CtrlBufSize,
					   uint8 *pu8DataBuf,uint16 u16DataSize, uint16 u16DataOffset)
*	@brief	Send packet using host interface.

*	@param [in]	u8Gid
*				Group ID.
*	@param [in]	u8Opcode
*				Operation ID.
*	@param [in]	pu8CtrlBuf
*				Pointer to the Control buffer.
*	@param [in]	u16CtrlBufSize
				Control buffer size.
*	@param [in]	u16DataOffset
				Packet Data offset.
*	@param [in]	pu8DataBuf
*				Packet buffer Allocated by the caller.
*	@param [in]	u16DataSize
				Packet buffer size (including the HIF header).
*    @return		The function shall return ZERO for successful operation and a negative value otherwise. 
*/

sint8 hif_send(uint8 u8Gid,uint8 u8Opcode,uint8 *pu8CtrlBuf,uint16 u16CtrlBufSize,
			   uint8 *pu8DataBuf,uint16 u16DataSize, uint16 u16DataOffset)
{
	sint8		ret = M2M_ERR_SEND;
	volatile tstrHifHdr	strHif;

	strHif.u8Opcode		= u8Opcode&(~NBIT7);
	strHif.u8Gid		= u8Gid;
	strHif.u16Length	= M2M_HIF_HDR_OFFSET;
	if(pu8DataBuf != NULL)
	{
		strHif.u16Length += u16DataOffset + u16DataSize;
	}
	else
	{
		strHif.u16Length += u16CtrlBufSize;
	}

	ret = hif_check_code(strHif.u8Gid, strHif.u8Opcode);
	if(ret != M2M_SUCCESS)
	{
		goto ERR1;
	}

	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS)
	{
		volatile uint32 reg, dma_addr = 0;
		volatile uint16 cnt = 0;
		
		reg = 0UL;
		reg |= (uint32)u8Gid;
		reg |= ((uint32)u8Opcode<<8);
		reg |= ((uint32)strHif.u16Length<<16);
		ret = nm_write_reg(NMI_STATE_REG,reg);
		if(M2M_SUCCESS != ret) goto ERR1;

		reg = 0;
		reg |= (1<<1);
		ret = nm_write_reg(WIFI_HOST_RCV_CTRL_2, reg);
		if(M2M_SUCCESS != ret) goto ERR1;
	

		if (ISNMC3400(nmi_get_chipid())) {
			
			ret = nm_write_reg(INTERRUPT_CORTUS_1_3000D0, 1);
			if(M2M_SUCCESS != ret) goto ERR1;
		}
		
		dma_addr = 0;

		//nm_bsp_interrupt_ctrl(0);

		for(cnt = 0; cnt < 1000*5; cnt ++)
		{			
			ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_2,(uint32 *)&reg);
			if(ret != M2M_SUCCESS) break;

			if (!(reg & 0x2))
			{
				ret = nm_read_reg_with_ret(0x150400,(uint32 *)&dma_addr);
				if(ret != M2M_SUCCESS) {
					/*in case of read error clear the dma address and return error*/
					dma_addr = 0;
				}
				/*in case of success break */
				break;
			}
			//If we are struggling to get a response, start waiting longer
			if (cnt>=1000)
				nm_bsp_sleep(5);
		}
		//nm_bsp_interrupt_ctrl(1);

		if (dma_addr != 0) 
		{
			volatile uint32	u32CurrAddr;
			u32CurrAddr = dma_addr;
			strHif.u16Length=NM_BSP_B_L_16(strHif.u16Length);
			M2M_DBG("Writing into %lx %ld\n", dma_addr, strHif.u16Length);
			ret = nm_write_block(u32CurrAddr, (uint8*)&strHif, M2M_HIF_HDR_OFFSET);
			if(M2M_SUCCESS != ret) goto ERR1;
			u32CurrAddr += M2M_HIF_HDR_OFFSET;
			if(pu8CtrlBuf != NULL)
			{
				ret = nm_write_block(u32CurrAddr, pu8CtrlBuf, u16CtrlBufSize);
				if(M2M_SUCCESS != ret) goto ERR1;
				u32CurrAddr += u16CtrlBufSize;
			}
			if(pu8DataBuf != NULL)
			{
				u32CurrAddr += (u16DataOffset - u16CtrlBufSize);
				ret = nm_write_block(u32CurrAddr, pu8DataBuf, u16DataSize);
				if(M2M_SUCCESS != ret) goto ERR1;
				u32CurrAddr += u16DataSize;
			}
			
			reg = dma_addr << 2;
			
			/* MERGEBUG: TODO: Following line of code is to generate the interrupt which 
			   is not needed for 3400, need to check if there are any side effects of keeping it
			*/
			reg |= (1 << 1);
			ret = nm_write_reg(WIFI_HOST_RCV_CTRL_3, reg);
			if(M2M_SUCCESS != ret) goto ERR1;

			if (ISNMC3400(nmi_get_chipid())) {
				ret = nm_write_reg(INTERRUPT_CORTUS_2_3000D0, 1);
				if(M2M_SUCCESS != ret) goto ERR1;
			}
		}
		else
		{
			M2M_DBG("Failed to alloc rx size\n");
			ret =  M2M_ERR_MEM_ALLOC;
			goto ERR1;
		}

	}
	else
	{
		M2M_ERR("(HIF)Fail to wakup the chip\n");
		goto ERR1;
	}
	ret = hif_chip_sleep();

ERR1:
	return ret;
}
/**
*	@fn		hif_isr
*	@brief	Host interface interrupt service routine
*	@author	M. Abdelmawla
*	@date	15 July 2012
*	@return	1 in case of interrupt received else 0 will be returned
*	@version	1.0
*/ 
static sint8 hif_isr(void)
{
	sint8 ret = M2M_SUCCESS;
	uint32 reg;
	volatile tstrHifHdr strHif;
	

	//if(ret == M2M_SUCCESS)
	{
		ret = nm_read_reg_with_ret(0x1070, &reg);
		if(M2M_SUCCESS == ret)
		{
			if(reg & 0x1)	/* New interrupt has been received */
			{
				uint16 size;

				nm_bsp_interrupt_ctrl(0);
				/*Clearing RX interrupt*/
				ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0,&reg);
				if(ret != M2M_SUCCESS)goto ERR1;
				reg &= ~(1<<0);
				ret = nm_write_reg(WIFI_HOST_RCV_CTRL_0,reg);
				if(ret != M2M_SUCCESS)goto ERR1;
				/* read the rx size */	
				ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0, &reg);
				if(M2M_SUCCESS != ret)
				{
					M2M_ERR("(hif) WIFI_HOST_RCV_CTRL_0 bus fail\n");
					nm_bsp_interrupt_ctrl(1);
					goto ERR1;
				}
				gu8HifSizeDone = 0;
				size = (uint16)((reg >> 2) & 0xfff);	
				if (size > 0) {
					uint32 address = 0;
					/**
					start bus transfer
					**/
					ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_1, &address);
					if(M2M_SUCCESS != ret)
					{
						M2M_ERR("(hif) WIFI_HOST_RCV_CTRL_1 bus fail\n");
						nm_bsp_interrupt_ctrl(1);
						goto ERR1;
					}	
					ret = nm_read_block(address, (uint8*)&strHif, sizeof(tstrHifHdr));
					strHif.u16Length = NM_BSP_B_L_16(strHif.u16Length);
					if(M2M_SUCCESS != ret)
					{
						M2M_ERR("(hif) address bus fail\n");
						nm_bsp_interrupt_ctrl(1);
						goto ERR1;
					}
					if(strHif.u16Length != size)
					{
						if((size - strHif.u16Length) > 4)
						{
							M2M_ERR("(hif) Corrupted packet Size = %u <L = %u, G = %u, OP = %02X>\n",
								size, strHif.u16Length, strHif.u8Gid, strHif.u8Opcode);
							nm_bsp_interrupt_ctrl(1);
							ret = M2M_ERR_BUS_FAIL;
							goto ERR1;
						}
					}
					
					if(M2M_REQ_GROUP_WIFI == strHif.u8Gid)
					{
						if(pfWifiCb)
							pfWifiCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
					
					} 
					else if(M2M_REQ_GROUP_IP == strHif.u8Gid)
					{
						if(pfIpCb) 
							pfIpCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
					}
					else if(M2M_REQ_GROUP_OTA == strHif.u8Gid)
					{
						if(pfOtaCb) 
							pfOtaCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
					}
                    else if(M2M_REQ_GROUP_SIGMA == strHif.u8Gid)
					{
						if(pfSigmaCb)
							pfSigmaCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
					}
					else if(M2M_REQ_GROUP_SSL == strHif.u8Gid)
					{
						if(pfSSLCb)
							pfSSLCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
					}
					else
					{
						M2M_ERR("(hif) invalid group ID\n");
						ret = M2M_ERR_BUS_FAIL;
						goto ERR1;
					}
					#ifndef ENABLE_UNO_BOARD
					if(!gu8HifSizeDone)
					{
						M2M_ERR("(hif) host app didn't set RX Done\n");
						ret = hif_set_rx_done();
					}
					#endif
				}
				else
				{
					ret = M2M_ERR_RCV;
					M2M_ERR("(hif) Wrong Size\n");
					goto ERR1;
				}
			}
			else
			{
#ifndef WIN32
				M2M_ERR("(hif) False interrupt %lx",reg);
#endif
			}
		}
		else
		{
			M2M_ERR("(hif) Fail to Read interrupt reg\n");
		}
	}

ERR1:
	return ret;
}

/**
*	@fn		hif_handle_isr(void)
*	@brief	Handle interrupt received from NMC1500 firmware.
*   @return     The function SHALL return 0 for success and a negative value otherwise.
*/

void hif_yield(void)
{
	gu8Yield = 1;
}

sint8 hif_handle_isr(void)
{
	sint8 ret = M2M_SUCCESS;
	
	if (gu8Interrupt) {
		ret = hif_chip_wake();
		if(ret == M2M_SUCCESS)
		{
			gu8Yield = 0;
			while (gu8Interrupt && !gu8Yield) {
				/*must be at that place because of the race of interrupt increment and that decrement*/
				/*when the interrupt enabled*/
				gu8Interrupt--;
				while(1)
				{
					ret = hif_isr(); 
					if(ret == M2M_SUCCESS) {
						/*we will try forever until we get that interrupt*/
						/*Fail return errors here due to bus errors (reading expected values)*/
						break;
					} else {
						M2M_ERR("(HIF) Fail to handle interrupt %d try Again..\n",ret);
					}
				}
			}
			ret = hif_chip_sleep();
		}
		else {
			M2M_ERR("(hif) FAIL to wakeup the chip\n");
		}
	}
	return ret;
}
/*
*	@fn		hif_receive
*	@brief	Host interface interrupt service routine
*	@param [in]	u32Addr
*				Receive start address
*	@param [out]	pu8Buf
*				Pointer to receive buffer. Allocated by the caller
*	@param [in]	u16Sz
*				Receive buffer size
*	@param [in]	isDone
*				If you don't need any more packets send True otherwise send false
*    @return		The function shall return ZERO for successful operation and a negative value otherwise. 
*/ 
sint8 hif_receive(uint32 u32Addr, uint8 *pu8Buf, uint16 u16Sz, uint8 isDone)
{
	uint32 address, reg;
	uint16 size;
	sint8 ret = M2M_SUCCESS;

	if(u32Addr == 0 || pu8Buf == NULL || u16Sz == 0)
	{
		if(isDone)
		{
			gu8HifSizeDone = 1;
			
			/* set RX done */
			ret = hif_set_rx_done();

			if((u32Addr == 0) && (pu8Buf == NULL) && (u16Sz == 0))
				return M2M_SUCCESS;
		}
			
		ret = M2M_ERR_FAIL;
		M2M_ERR(" hif_receive: Invalid argument\n");
		goto ERR1;
	}

	ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0,&reg);
	if(ret != M2M_SUCCESS)goto ERR1;	


	size = (uint16)((reg >> 2) & 0xfff);	
	ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_1,&address);
	if(ret != M2M_SUCCESS)goto ERR1;	

	/* Receive the payload */
	ret = nm_read_block(u32Addr, pu8Buf, u16Sz);
	if(ret != M2M_SUCCESS)goto ERR1;

	if(u16Sz > size)
	{
		ret = M2M_ERR_FAIL;
		M2M_ERR("APP Requested Size is larger than the recived buffer size <%d><%d>\n",u16Sz, size);
		goto ERR1;
	}
	if((u32Addr < address)||((u32Addr + u16Sz)>(address+size)))
	{
		ret = M2M_ERR_FAIL;
		M2M_ERR("APP Requested Address beyond the recived buffer address and length\n");
		goto ERR1;
	}

	/* check if this is the last packet */
	if(isDone || (((address+size) - (u32Addr+u16Sz)) == 0) || 
		((4 - ((u32Addr+u16Sz) & 3)) == ((address+size) - (u32Addr+u16Sz))))	/* Length in the RCV CTRL 0 register is rounded off to 4 by the firmware,
																				   but length inside the HIF header is not, Hence consider done if number 
																				   of rounding bytes equal to length left to read */
	{
		gu8HifSizeDone = 1;
		
		/* set RX done */
		ret = hif_set_rx_done();
	}
	


ERR1:
	return ret;
}

/**
*	@fn		hif_register_cb
*	@brief	To set Callback function for every component
*	@param [in]	u8Grp
*				Group to which the Callback function should be set.
*	@param [in]	fn
*				function to be set
*    @return		The function shall return ZERO for successful operation and a negative value otherwise. 
*/

sint8 hif_register_cb(uint8 u8Grp,tpfHifCallBack fn)
{
	sint8 ret = M2M_SUCCESS;
	switch(u8Grp)
	{
		case M2M_REQ_GROUP_IP:
			pfIpCb = fn;
			break;
		case M2M_REQ_GROUP_WIFI:
			pfWifiCb = fn;
			break;
		case M2M_REQ_GROUP_OTA:
			pfOtaCb = fn;
			break;
		case M2M_REQ_GROUP_HIF:
			pfHifCb = fn;
			break;
		case M2M_REQ_GROUP_SIGMA:
			pfSigmaCb = fn;
			break;            
		case M2M_REQ_GROUP_SSL:
			pfSSLCb = fn;
			break;
		default:
			M2M_ERR("GRp ? %d\n",u8Grp);
			ret = M2M_ERR_FAIL;
			break;
	}
	return ret;
}

#endif
