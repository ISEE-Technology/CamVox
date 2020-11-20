/**
 *
 * \file
 *
 * \brief WINC3400 Flash Interface.
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

/**@defgroup FLASHAPI FLASH
*/

#ifndef __M2M_FLASH_H__
#define __M2M_FLASH_H__

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "common/include/nm_common.h"
#include "driver/include/m2m_types.h"

/**@defgroup  FlashDefs Defines
 * @ingroup FLASHAPI
 * @{*/


/*!	Bit 0 of u8Options parameter of @ref m2m_flash_updateimage.\n
 *	Request to write new firmware image to WINC inactive partition. */
#define FLASH_UPDATEIMAGE_OPTION_UPDATE		NBIT0
/*!	Bit 1 of u8Options parameter of @ref m2m_flash_updateimage.\n
 *	Request to mark image in WINC inactive partition as valid. If set along with
 *	@ref FLASH_UPDATEIMAGE_OPTION_UPDATE, the image is not marked valid if the writing fails. */
#define FLASH_UPDATEIMAGE_OPTION_VALIDATE	NBIT1
/*!	Bit 2 of u8Options parameter of @ref m2m_flash_updateimage.\n
 *	Request to switch the WINC active/inactive partitions. Use this to switch to a new image or to
 *	revert to an old one. The switch only succeeds if the image has been marked valid. */
#define FLASH_UPDATEIMAGE_OPTION_SWITCH		NBIT2

/*!	Return success, transfer succeeded. */
#define FLASH_SUCCESS				0
/*!	Return error. The parameters provided by the MCU application failed sanity checks. */
#define FLASH_ERR_PARAM				(-1)
/*!	Return error. An MCU application function of type @ref tpfDataAccessFn returned failure. */
#define FLASH_ERR_LOCAL_ACCESS		(-2)
/*!	Return error. There was an error while accessing WINC flash, or contents of WINC flash were not as expected. */
#define FLASH_ERR_WINC_ACCESS		(-3)
/*!	Return error. The transfer destination location was too small for the size of data to be transferred. */
#define FLASH_ERR_SIZE				(-4)
/*!	Return error. The data provided by the MCU application caused a failure. For example:\n
 *	- An item could not be added to a WINC flash store because the identifier matched an existing item.
 *	- An item could not be read or removed from a WINC flash store because the identifier did not match an existing item.
 *	- A firmware image could not be written to WINC flash because it did not match the format of WINC firmware images. */
#define FLASH_ERR_WINC_CONFLICT		(-5)
/*!	Return error. The MCU application did not call @ref m2m_flash_init during Wi-Fi initialization. */
#define FLASH_ERR_UNINIT			(-6)
/*!	Return error. The module hit an internal error, such as insufficient heap memory. */
#define FLASH_ERR_INTERNAL			(-7)

 /**@}
 */
/**@defgroup  FlashEnums Enumeration/Typedefs
 * @ingroup FLASHAPI
 * @{*/

/*!
@enum	tenuFlashDataFnCtl

@brief	Control parameter for @ref tpfDataAccessFn.
 */
typedef enum {
	/*!	Data access about to start. Check and save parameters as required, ready for subsequent
	 *	data accesses. */
	FLASH_DATA_FN_INITIALIZE,
	/*!	Sequential data access, using parameters previously provided by FLASH_DATA_FN_INITIALIZE. */
	FLASH_DATA_FN_DATA,
	/*!	Data access aborted. Did not complete and will not be continued. */
	FLASH_DATA_FN_TERMINATE,
	/*!	Data access complete. */
	FLASH_DATA_FN_COMPLETE
}tenuFlashDataFnCtl;

/*!
@enum	tenuDataFnRW

@brief	Indicates whether data is to be read or written
 */
typedef enum {
	/*!	Read data from application memory. */
	FLASH_DATA_FN_READ,
	/*!	Write data to application memory. */
	FLASH_DATA_FN_WRITE
}tenuDataFnRW;

/*!
@enum	tenuImageId

@brief	Indicates which image to access in WINC flash
 */
typedef enum {
	/*!	Access the image in the inactive partition. */
	FLASH_IMAGE_INACTIVE,
	/*!	Access the image in the active partition. */
	FLASH_IMAGE_ACTIVE
}tenuImageId;


/*!
@typedef \
	tpfDataAccessFn

@brief
	A function of this type is used for local data access. It can be implemented to handle simple
	RAM access with pointer/length, or it can be implemented to handle access to external memory.
	Functions of this type are provided to the module via various function APIs.
@see	m2m_flash_rootcert_add
@see	m2m_flash_rootcert_read
@see	m2m_flash_rootcert_readidx
@see	m2m_flash_updateimage
@see	m2m_flash_readimage

@param [in]		enuCtl
					Control parameter.
@param [in]		pvStr
					Generic pointer to structure. The structure type depends on enuCtl:
					- @ref tstrDataAccessInitParamsApp if enuCtl is @ref FLASH_DATA_FN_INITIALIZE.
					- @ref tstrDataAccessParamsApp if enuCtl is @ref FLASH_DATA_FN_DATA.
					- @ref NULL if enuCtl is @ref FLASH_DATA_FN_TERMINATE or @ref FLASH_DATA_FN_COMPLETE.

@note	The function is typically called by the module multiple times during a data transfer:
		Once with @ref FLASH_DATA_FN_INITIALIZE, then multiple times with @ref FLASH_DATA_FN_DATA, then
		once with @ref FLASH_DATA_FN_COMPLETE.

@return			Zero for success. Non-zero otherwise.

\section FlashExample1 Example
This example illustrates an implementation of a function of type @ref tpfDataAccessFn. \n
This example assumes the application's data buffer is in RAM, so we can use memcpy. Alternatively
memcpy can be replaced by dedicated functions which read/write the application buffer.

gpu8DataLocation is a pointer to the data buffer in RAM, set by the application. \n
gu32DataSize is the size of the data buffer, set by the application.

@code{.c}
sint8 data_access_ptr(tenuFlashDataFnCtl enuCtl, void *pvStr)
{
	// Return value.
	sint8			s8Ret = -1;
	// Fixed variables, set in case FLASH_DATA_FN_INITIALIZE, reset in case FLASH_DATA_FN_COMPLETE.
	static tenuDataFnRW	enuRW;
	static uint8		*pu8Location = NULL;
	static uint32		u32BytesRemaining = 0;

	switch (enuCtl)
	{
	case FLASH_DATA_FN_INITIALIZE:
		if (pvStr != NULL)
		{
			tstrDataAccessInitParamsApp *init_params = (tstrDataAccessInitParamsApp*)pvStr;
			if (init_params->u32TotalSize > gu32DataSize)
				break;
			// Set fixed variables.
			enuRW = init_params->enuRW;
			pu8Location = gpu8DataLocation;
			u32BytesRemaining = init_params->u32TotalSize;
			s8Ret = 0;
		}
		break;
	case FLASH_DATA_FN_DATA:
		if ((pvStr != NULL) && (pu8Location != NULL))
		{
			tstrDataAccessParamsApp *params = (tstrDataAccessParamsApp*)pvStr;
			// Some sanity checks which should never fail.
			if (u32BytesRemaining < params->u32DataSize)
				break;
			if (params->pu8Data == NULL)
				break;
			switch (enuRW)
			{
			case FLASH_DATA_FN_WRITE:
				memcpy(pu8Location, params->pu8Data, params->u32DataSize);
				s8Ret = 0;
				break;
			case FLASH_DATA_FN_READ:
				memcpy(params->pu8Data, pu8Location, params->u32DataSize);
				s8Ret = 0;
				break;
			}
			// Update fixed variables for sequential access.
			pu8Location += params->u32DataSize;
			u32BytesRemaining -= params->u32DataSize;
		}
		break;
	case FLASH_DATA_FN_TERMINATE:
	case FLASH_DATA_FN_COMPLETE:
		// Reset fixed variables.
		pu8Location = NULL;
		u32BytesRemaining = 0;
		s8Ret = 0;
		break;
	}
	return s8Ret;
}
@endcode
*/
typedef sint8 (*tpfDataAccessFn) (tenuFlashDataFnCtl enuCtl, void *pvStr);

/*!
@struct	\
	tstrDataAccessInitParamsApp
@brief
	This structure contains parameters for initializing a local data access (read or write).

@see	tpfDataAccessFn.
 */
typedef struct {
	/*!	Total size of data to be accessed in data location. */
	uint32			u32TotalSize;
	/*!	Flags indicating type of data access (read or write). */
	tenuDataFnRW	enuRW;
}tstrDataAccessInitParamsApp;

/*!
@struct	\
	tstrDataAccessParamsApp
@brief
	This structure contains data for local data access (read or write).
@see	tpfDataAccessFn.
 */
typedef struct {
	/*!	Buffer to be written from or read to. */
	uint8	*pu8Data;
	/*! Size of data to be written or read. */
	uint32	u32DataSize;
}tstrDataAccessParamsApp;

/*!
@struct	\
	tstrFlashState
@brief
	This structure contains information about an attempted transfer.
@see	m2m_flash_init
 */
typedef struct {
	/*!	Last access attempt. Identifier provided by MCU application. 0 if info not available. */
	uint16				u16LastAccessId;
	/*!	Was the last access attempt successful? */
	uint8				u8Success;
	/*!	Did the last access attempt result in modified WINC flash? */
	uint8				u8Changed;
	/*!	Has the module been initialized? */
	uint8				u8Init;
	/*!	Has the module been initialized more recently than the module last reset the WINC? */
	uint8				u8Reset;
}tstrFlashState;

/**@}
*/

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/** \defgroup FLASHFUNCTIONS Functions
*  @ingroup FLASHAPI
*/

/**@{*/
/*!
@fn	\
	sint8 m2m_flash_updateimage(uint8 u8Options, uint16 u16Id, tpfDataAccessFn pfSourceFn, uint32 u32SourceSize);

@brief	Write/validate/switch a WINC firmware image.

@param [in]		u8Options
					Flags indicating the required combination of write / validate / switch.
@param [in]		u16Id
					Transfer identifier, not interpreted by this module.
@param [in]		pfSourceFn
					Function for the module to call to obtain the image from the MCU application.
@param [in]		u32SourceSize
					Size of the image being provided by the MCU application.

@see	FLASH_UPDATEIMAGE_OPTION_UPDATE
@see	FLASH_UPDATEIMAGE_OPTION_VALIDATE
@see	FLASH_UPDATEIMAGE_OPTION_SWITCH

@return		Zero indicates success.
			Negative values indicate failure.
*/
sint8 m2m_flash_updateimage(uint8 u8Options, uint16 u16Id, tpfDataAccessFn pfSourceFn, uint32 u32SourceSize);
/*!
@fn	\
	sint8 m2m_flash_readimage(tenuImageId enuImageId, uint16 u16Id, tpfDataAccessFn pfDestFn, uint32 u32DestSize);

@brief	Read a WINC firmware image.

@param [in]		enuImageId
					Which partition to read image from.
@param [in]		u16Id
					Transfer identifier, not interpreted by this module.
@param [in]		pfDestFn
					Function for the module to call to send the image to the MCU application.
@param [in]		u32DestSize
					Size of the memory available to the MCU application for storing the image.

@return		Zero indicates success.
			Negative values indicate failure.
*/
sint8 m2m_flash_readimage(tenuImageId enuImageId, uint16 u16Id, tpfDataAccessFn pfDestFn, uint32 u32DestSize);
/*!
@fn	\
sint8 m2m_flash_rootcert_add(uint16 u16Id, tpfDataAccessFn pfSourceFn, uint32 u32SourceSize);

@brief	Add an entry to the WINC TLS root certificate store.

@param [in]		u16Id
					Transfer identifier, not interpreted by this module.
@param [in]		pfSourceFn
					Function for the module to call to obtain the certificate entry from the MCU application.
@param [in]		u32SourceSize
					Size of the certificate entry being provided by the MCU application.

@note		The data location to be accessed via pfSourceFn
			contains the entry to be added. The format of a TLS root certificate entry is:
			@code{.c}
			Name		Offset			Length		Contents
			Header		0				36			tstrRootCertEntryHeader
			Key			36				KeyLength	Public key and padding
			Padding		36+KeyLength	PadLength	0xFF
			@endcode
			For RSA public keys, the format of Key is:
			@code{.c}
			Name		Offset			Length
			Modulus		36				ModLength	RSA modulus, with leading 0s stripped off
			Exponent	36+ModLength	ExpLength	Public exponent, with leading 0s stripped off
			@endcode
			In this case PadLength is the amount of padding that would be required for 4-byte alignment of Modulus
			plus the amount of padding that would be required for 4-byte alignment of Exponent.\n
			For ECDSA public keys, the format of Key is:
			@code{.c}
			Name		Offset			Length
			X Coord		36				CoordLength	Public key X-coordinate
			Y Coord		36+CoordLength	CoordLength	Public key Y-coordinate
			@endcode
			In this case PadLength is the amount of padding that would be required for 4-byte alignment of X Coord
			plus the amount of padding that would be required for 4-byte alignment of Y Coord.

@return		Zero indicates success.
			Negative values indicate failure.
*/
sint8 m2m_flash_rootcert_add(uint16 u16Id, tpfDataAccessFn pfFn, uint32 u32Size);

/*!
@fn	\
sint8 m2m_flash_rootcert_remove(uint16 u16Id, uint8 *pu8Identifier, uint32 u32IdentifierSz);

@brief	Remove an entry from the WINC TLS root certificate store.

@param [in]		u16Id
					Transfer identifier, not interpreted by this module.
@param [in]		pu8Identifier
					Identifier for the entry to be removed. This is the SHA1 digest of the certificate issuer name.
@param [in]		u32IdentifierSz
					Size of the identifier (20).

@return		Zero indicates success.
			Negative values indicate failure.
*/
sint8 m2m_flash_rootcert_remove(uint16 u16Id, uint8 *pu8Identifier, uint32 u32IdentifierSz);
/*!
@fn	\
	sint8 m2m_flash_rootcert_read(uint16 u16Id, tpfDataAccessFn pfDestFn, uint32 u32DestSize, uint8 *pu8Identifier, uint32 u32IdentifierSz);

@brief	Read an entry from the WINC TLS root certificate store, referenced by entry identifier.

@param [in]		u16Id
					Transfer identifier, not interpreted by this module.
@param [in]		pfDestFn
					Function for the module to call to send the certificate entry to the MCU application.
@param [in]		u32DestSize
					Size of the memory available to the MCU application for storing the entry.
@param [in]		pu8Identifier
					Identifier for the entry to be read. This is the SHA1 digest of the certificate issuer name.
@param [in]		u32IdentifierSz
					Size of the identifier (20).

@return		Zero indicates success.
			Negative values indicate failure.
*/
sint8 m2m_flash_rootcert_read(uint16 u16Id, tpfDataAccessFn pfDestFn, uint32 u32DestSize, uint8 *pu8Identifier, uint32 u32IdentifierSz);
/*!
@fn	\
	sint8 m2m_flash_rootcert_readidx(uint16 u16Id, tpfDataAccessFn pfDestFn, uint32 u32DestSize, uint8 u8Index);

@brief	Read an entry from the WINC TLS root certificate store, referenced by entry index.

@param [in]		u16Id
					Transfer identifier, not interpreted by this module.
@param [in]		pfDestFn
					Function for the module to call to send the certificate entry to the MCU application.
@param [in]		u32DestSize
					Size of the memory available to the MCU application for storing the entry.
@param [in]		u8Index
					Zero-based index of the entry to be read.

@return		Zero indicates success.
			Negative values indicate failure.
*/
sint8 m2m_flash_rootcert_readidx(uint16 u16Id, tpfDataAccessFn pfDestFn, uint32 u32DestSize, uint8 u8Index);

/*!
@fn	\
	void m2m_flash_get_state(tstrFlashState *pstrState);

@brief	Get information about the module current state and about the most recent access attempt.

@param [out]	pstrState
					Module state and most recent access attempt info.

@return	None.
*/
void m2m_flash_get_state(tstrFlashState *pstrState);


/*!
@fn	\
	sint8 m2m_flash_init(void);

@brief	Initialize the module.

@warning	This API must be called at a particular point during MCU application initialization.

@see	m2m_wifi_init
@see	m2m_wifi_init_hold
@see	m2m_wifi_init_start

\section FlashExample2 Example
This example demonstrates how to add @ref m2m_flash_init and @ref m2m_flash_get_state
to MCU application initialization code.

Original code:
@code{.c}
{
	tstrWifiInitParam param = {...}
	sint8 status = 0;

	status = m2m_wifi_init(param);
	if (status != 0)
	{
		// Wifi init failed.
	}
}
@endcode
New code:
@code{.c}
{
	tstrWifiInitParam param = {...}
	sint8 status = 0;

	status = m2m_wifi_init_hold();
	if (status == 0)
	{
		tstrFlashState	strFlashState;

		m2m_flash_get_state(&strFlashState);
		printf("FlashAccess:%x:%d%d\n", strFlashState.u16LastAccessId, strFlashState.u8Success, strFlashState.u8Changed);

		switch (strFlashState.u16LastAccessId)
		{
			// Application code dependent on meaning of u16LastAccessId.
		}
		m2m_flash_init();
		status = m2m_wifi_init_start(param);
	}
	if (status != 0)
	{
		// Wifi init failed.
	}
}
@endcode

@return		Zero for successful initialization of module. Negative value otherwise.
*/
sint8 m2m_flash_init(void);

 /**@}*/
#endif /* __M2M_FLASH_H__ */
