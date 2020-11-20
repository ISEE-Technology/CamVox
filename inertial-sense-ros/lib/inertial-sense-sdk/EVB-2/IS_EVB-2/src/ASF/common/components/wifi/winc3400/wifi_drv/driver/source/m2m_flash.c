/**
 *
 * \file
 *
 * \brief WINC Flash Interface.
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



/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "driver/include/m2m_flash.h"
#include "driver/include/m2m_wifi.h"
#include "driver/source/nmflash.h"
#include "driver/source/m2m_hif.h"
#include "spi_flash/include/spi_flash.h"
#include "nmdrv.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
GLOBALS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
static tpfDataAccessFn	gpfAppFn = NULL;
static uint8			gau8ItemIdentifier[20] = {0};

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTIONS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "ISConstants.h"
#define malloc	MALLOC
#define free	FREE

static sint8 transfer_init(void)
{
	sint8 ret = FLASH_RETURN_OK;

	/* Check module was initialized. */
	if (gu8Init == 0)
		ret = FLASH_ERR_UNINIT;
	return ret;
}
static sint8 init_access(void)
{
	sint8	ret = FLASH_RETURN_OK;

	gu8Reset = 0;
	if (m2m_wifi_reinit_hold() != M2M_SUCCESS)
		ret = FLASH_ERR_WINC_ACCESS;

	return ret;
}
static sint8 commit_access(tstrFlashAccess *pstrFlashAccess)
{
	sint8						ret = FLASH_RETURN_OK;
	sint8						status = M2M_ERR_FAIL;
	tstrFlashAccessPersistent	*pstrPersistentInfo = &pstrFlashAccess->strPersistentInfo;
	
	/*
	 *	To begin with, flash is unchanged. Later, when first flash erase/write occurs, this flag
	 *	will be cleared.
	 */
	pstrPersistentInfo->u8ModeFlags |= FLASH_MODE_FLAGS_UNCHANGED;
	if (pstrPersistentInfo->u8ModeFlags & FLASH_MODE_FLAGS_CS_SWITCH)
	{
		uint8 target = 0;
		if (image_get_target(&target) != M2M_SUCCESS)
		{
			ret = FLASH_ERR_WINC_ACCESS;
			goto ERR;
		}
		if (target > 0)
			pstrPersistentInfo->u8ModeFlags |= FLASH_MODE_FLAGS_CS_SWITCH_TARGET;
	}

	status = spi_flash_read((uint8*)pstrPersistentInfo, HOST_CONTROL_FLASH_OFFSET, FLASH_SIG_STA_SZ);
	if (status == M2M_SUCCESS)
	{
		if ((pstrPersistentInfo->u32Signature != FLASH_SIGNATURE) || (pstrPersistentInfo->enuTransferStatus != FLASH_STATUS_EMPTY))
			status = spi_flash_erase(HOST_CONTROL_FLASH_OFFSET, HOST_CONTROL_FLASH_SZ);
	}
	if (status == M2M_SUCCESS)
	{
		pstrPersistentInfo->u32Signature = FLASH_SIGNATURE;
		pstrPersistentInfo->enuTransferStatus = FLASH_STATUS_NOT_ACTIVE;
		status = winc_flash_write_verify((uint8*)pstrPersistentInfo, HOST_CONTROL_FLASH_OFFSET, FLASH_SIG_STA_SZ);
		if (status == M2M_SUCCESS)
		{
			status = winc_flash_write_verify((uint8*)pstrPersistentInfo, HOST_CONTROL_FLASH_OFFSET, sizeof(tstrFlashAccessPersistent));
			if (status == M2M_SUCCESS)
			{
				pstrPersistentInfo->enuTransferStatus = FLASH_STATUS_ACTIVE;
				status = winc_flash_write_verify((uint8*)pstrPersistentInfo, HOST_CONTROL_FLASH_OFFSET, FLASH_SIG_STA_SZ);
				gu16LastAccessId = pstrPersistentInfo->u16AppId;
				gu8Success = 0;
				gu8Changed = 0;
			}
		}
	}
	if (status != M2M_SUCCESS)
	{
		ret = FLASH_ERR_WINC_ACCESS;
		goto ERR;
	}
	ret = transfer_run(pstrFlashAccess);
ERR:
	return ret;
}
static sint8 register_app_fn(tpfDataAccessFn pfFn)
{
	sint8 ret = FLASH_RETURN_OK;
	if (pfFn == NULL)
		ret = FLASH_ERR_PARAM;
	gpfAppFn = pfFn;
	return ret;
}
static sint8 app_data_access(tenuFlashDataFnCtl enuCtl, void *pvStr)
{
	tstrDataAccessInitParamsApp	init_params_app;
	tstrDataAccessParamsApp		params_app;
	switch (enuCtl)
	{
	case FLASH_DATA_FN_INITIALIZE:
		{
			tstrDataAccessInitParams	*init_params = (tstrDataAccessInitParams*)pvStr;
			init_params_app.u32TotalSize = init_params->u32TotalSize;
			if (init_params->u8Flags & FLASH_FN_FLAGS_READ)
				init_params_app.enuRW = FLASH_DATA_FN_READ;
			else if (init_params->u8Flags & FLASH_FN_FLAGS_WRITE)
				init_params_app.enuRW = FLASH_DATA_FN_WRITE;
			pvStr = &init_params_app;
		}
		break;
	case FLASH_DATA_FN_DATA:
		{
			tstrDataAccessParams	*params = (tstrDataAccessParams*)pvStr;
			params_app.pu8Data = params->pu8Buf + params->u32DataOffset;
			params_app.u32DataSize = params->u32DataSize;
			pvStr = &params_app;
		}
		break;
	case FLASH_DATA_FN_COMPLETE:
	case FLASH_DATA_FN_TERMINATE:
		break;
	}
	return gpfAppFn(enuCtl, pvStr);
}
sint8 m2m_flash_readimage(uint8 enuImageId, uint16 u16Id, tpfDataAccessFn pfDestFn, uint32 u32DestSize)
{
	sint8	ret = FLASH_RETURN_OK;

	M2M_INFO("FA RdImage %d\n", enuImageId);
	ret = transfer_init();
	if (ret < 0)
		goto ERR;
	ret = register_app_fn(pfDestFn);
	if (ret < 0)
		goto ERR;
	if (u32DestSize < OTA_IMAGE_SIZE)
	{
		ret = FLASH_ERR_SIZE;
		goto ERR;
	}
	if (enuImageId > FLASH_IMAGE_INACTIVE)
	{
		ret = FLASH_ERR_PARAM;
		goto ERR;
	}

	ret = init_access();
	if (ret == FLASH_RETURN_OK)
	{
		/* Set parameters for whole transfer. */
		tstrFlashAccess strFlashAccess;
		m2m_memset((uint8*)&strFlashAccess, 0, sizeof(tstrFlashAccess));

		strFlashAccess.strPersistentInfo.u16AppId = u16Id;

		strFlashAccess.pfDestinationFn = app_data_access;
		switch (enuImageId)
		{
		case FLASH_IMAGE_ACTIVE:
			set_internal_info(&strFlashAccess.pfSourceFn, MEM_ID_WINC_ACTIVE);
			break;
		case FLASH_IMAGE_INACTIVE:
			set_internal_info(&strFlashAccess.pfSourceFn, MEM_ID_WINC_INACTIVE);
			break;
		}
		strFlashAccess.u32Size = OTA_IMAGE_SIZE;

		ret = commit_access(&strFlashAccess);
	}
ERR:
	M2M_INFO("FAState:%d\n", ret);
	return ret;
}

sint8 m2m_flash_updateimage(uint8 u8Options, uint16 u16Id, tpfDataAccessFn pfSourceFn, uint32 u32SourceSize)
{
	sint8	ret = FLASH_RETURN_OK;

	M2M_INFO("FA Image %d\n", u8Options);
	ret = transfer_init();
	if (ret < 0)
		goto ERR;
	if (u8Options & FLASH_UPDATEIMAGE_OPTION_UPDATE)
	{
		uint8						au8ImageStart[4];
		uint8						au8ImageCheck[] = {'N','M','I','S'};
		tstrDataAccessInitParams	init_params = {sizeof(au8ImageStart), FLASH_FN_FLAGS_READ};
		sint8						status = M2M_SUCCESS;

		/* Check input parameters. */
		ret = register_app_fn(pfSourceFn);
		if (ret < 0)
			goto ERR;
		if (u32SourceSize != OTA_IMAGE_SIZE)
		{
			ret = FLASH_ERR_PARAM;
			goto ERR;
		}
		status = app_data_access(FLASH_DATA_FN_INITIALIZE, &init_params);
		if (status == M2M_SUCCESS)
		{
			tstrDataAccessParams	params = {au8ImageStart, sizeof(au8ImageStart), 0, sizeof(au8ImageStart)};
			status = app_data_access(FLASH_DATA_FN_DATA, &params);
		}
		if (status != M2M_SUCCESS)
		{
			ret = FLASH_ERR_LOCAL_ACCESS;
			goto ERR;
		}
		if (m2m_memcmp(au8ImageStart, au8ImageCheck, sizeof(au8ImageStart)))
		{
			ret = FLASH_ERR_WINC_CONFLICT;
			goto ERR;
		}
	}

	ret = init_access();
	if (ret == FLASH_RETURN_OK)
	{
		/* Set parameters for whole transfer. */
		tstrFlashAccess strFlashAccess;
		m2m_memset((uint8*)&strFlashAccess, 0, sizeof(tstrFlashAccess));

		strFlashAccess.strPersistentInfo.u16AppId = u16Id;
		if (u8Options & FLASH_UPDATEIMAGE_OPTION_UPDATE)
		{
			strFlashAccess.strPersistentInfo.u8AccessFlags = FLASH_ACCESS_WINC_MASK | FLASH_ACCESS_OPTION_ERASE_FIRST;

			strFlashAccess.pfSourceFn = app_data_access;
			set_internal_info(&strFlashAccess.pfDestinationFn, MEM_ID_WINC_INACTIVE);
			strFlashAccess.u32Size = OTA_IMAGE_SIZE;
		}
		else
			strFlashAccess.u32Size = 0;
		if (u8Options & FLASH_UPDATEIMAGE_OPTION_VALIDATE)
			strFlashAccess.strPersistentInfo.u8ModeFlags |= FLASH_MODE_FLAGS_CS_VALIDATE_IMAGE;
		if (u8Options & FLASH_UPDATEIMAGE_OPTION_SWITCH)
			strFlashAccess.strPersistentInfo.u8ModeFlags |= FLASH_MODE_FLAGS_CS_SWITCH;
		strFlashAccess.strPersistentInfo.u8ModeFlags |= FLASH_MODE_FLAGS_CS;

		ret = commit_access(&strFlashAccess);
	}
ERR:
	M2M_INFO("FAState:%d\n", ret);
	return ret;
}

static sint8 m2m_flash_rootcert_access(tenuFlashAccessItemMode enuMode, uint8 u8ModeOptions, uint8 u8AccessOptions, uint16 u16Id, tpfDataAccessFn pfFn, uint32 u32Size)
{
	sint8						ret = FLASH_RETURN_OK;
	tstrRootCertEntryHeader		strRootCertEntry;
	uint16						u16EntrySz = 0;

	M2M_INFO("FA Rootcert %d\n", enuMode);
	ret = transfer_init();
	if (ret < 0)
		goto ERR;

	switch (enuMode)
	{
	case FLASH_ITEM_ADD:
		{
			sint8						status = M2M_SUCCESS;
			tstrDataAccessInitParams	init_params = {sizeof(strRootCertEntry), FLASH_FN_FLAGS_READ};

			// Read the entry header
			if (u32Size < sizeof(strRootCertEntry))
			{
				ret = FLASH_ERR_PARAM;
				goto ERR;
			}
			status = pfFn(FLASH_DATA_FN_INITIALIZE, &init_params);
			if (status == M2M_SUCCESS)
			{
				tstrDataAccessParams	params = {(uint8*)&strRootCertEntry, sizeof(strRootCertEntry), 0, sizeof(strRootCertEntry)};
				status = pfFn(FLASH_DATA_FN_DATA, &params);
			}
			if (status != M2M_SUCCESS)
			{
				ret = FLASH_ERR_LOCAL_ACCESS;
				goto ERR;
			}
			// Check source size matches size calculated from entry header.
			status = rootcert_get_size(&strRootCertEntry, &u16EntrySz);
			if ((status != M2M_SUCCESS) || (u32Size != u16EntrySz))
			{
				ret = FLASH_ERR_PARAM;
				goto ERR;
			}
		}
		break;
	case FLASH_ITEM_READ:
	case FLASH_ITEM_REMOVE:
		m2m_memcpy(strRootCertEntry.au8SHA1NameHash, gau8ItemIdentifier, sizeof(gau8ItemIdentifier));
		m2m_memset(gau8ItemIdentifier, 0, sizeof(gau8ItemIdentifier));
		break;
	case FLASH_ITEM_READIDX:
		// Hack strRootCertEntry to carry the index from u8ModeOptions.
		*(uint32*)&strRootCertEntry = u8ModeOptions;
		break;
	default:
		/* No other item modes supported. */
		ret = FLASH_ERR_PARAM;
		goto ERR;
		break;
	}

	ret = init_access();
	if (ret == FLASH_RETURN_OK)
	{
		/* Now we can access the items in flash. */
		uint8	*pu8Buff = malloc(M2M_TLS_ROOTCER_FLASH_SZ);
		uint32	u32Offset = 0;
		if (pu8Buff == NULL)
		{
			ret = FLASH_ERR_INTERNAL;
			goto ERR;
		}
		ret = rootcert_access(enuMode, &strRootCertEntry, &u16EntrySz, pu8Buff, &u32Offset);
		if (ret == FLASH_RETURN_OK)
		{
			/* Set parameters for whole transfer, according to enuMode. */
			sint8						status = M2M_SUCCESS;
			tstrDataAccessInitParams	init_params = {u16EntrySz, 0};
			tstrDataAccessParams		data_params = {pu8Buff + u32Offset, u16EntrySz, 0, u16EntrySz};
			tstrFlashAccess				strFlashAccess;

			m2m_memset((uint8*)&strFlashAccess, 0, sizeof(tstrFlashAccess));
			strFlashAccess.strPersistentInfo.u16AppId = u16Id;
			strFlashAccess.strPersistentInfo.u8AccessFlags = u8AccessOptions;

			switch (enuMode)
			{
			case FLASH_ITEM_ADD:
				init_params.u8Flags = FLASH_FN_FLAGS_READ;
				status = pfFn(FLASH_DATA_FN_INITIALIZE, &init_params);
				if (status == M2M_SUCCESS)
					status = pfFn(FLASH_DATA_FN_DATA, &data_params);
				if (status != M2M_SUCCESS)
				{
					ret = FLASH_ERR_LOCAL_ACCESS;
					pfFn(FLASH_DATA_FN_TERMINATE, NULL);
					break;
				}
				u32Offset += u16EntrySz;
				// intentional fallthrough.
			case FLASH_ITEM_REMOVE:
				status = spi_flash_erase(M2M_BACKUP_FLASH_OFFSET, M2M_BACKUP_FLASH_SZ);
				if (status == M2M_SUCCESS)
					status = winc_flash_write_verify(pu8Buff, M2M_BACKUP_FLASH_OFFSET, u32Offset);
				if (status != M2M_SUCCESS)
				{
					ret = FLASH_ERR_WINC_ACCESS;
					break;
				}
				set_internal_info(NULL, M2M_TLS_ROOTCER_FLASH_OFFSET);
				strFlashAccess.strPersistentInfo.u8ModeFlags |= FLASH_MODE_FLAGS_DATA_IN_BACKUP;
				break;
			case FLASH_ITEM_READ:
			case FLASH_ITEM_READIDX:
				// Check source size is sufficient for reading entry.
				if (u32Size < u16EntrySz)
				{
					ret = FLASH_ERR_SIZE;
					break;
				}
				init_params.u8Flags = FLASH_FN_FLAGS_WRITE;
				status = pfFn(FLASH_DATA_FN_INITIALIZE, &init_params);
				if (status == M2M_SUCCESS)
					status = pfFn(FLASH_DATA_FN_DATA, &data_params);
				if (status != M2M_SUCCESS)
				{
					ret = FLASH_ERR_LOCAL_ACCESS;
					pfFn(FLASH_DATA_FN_TERMINATE, NULL);
					break;
				}
				break;
			}
			if (ret == 0)
			{
				ret = commit_access(&strFlashAccess);
				if (enuMode != FLASH_ITEM_REMOVE)
					pfFn(FLASH_DATA_FN_COMPLETE, NULL);
			}
		}
		free(pu8Buff);
	}
ERR:
	M2M_INFO("FAState:%d\n", ret);
	return ret;
}
sint8 m2m_flash_rootcert_add(uint16 u16Id, tpfDataAccessFn pfSourceFn, uint32 u32SourceSize)
{
	sint8 ret = FLASH_RETURN_OK;

	ret = register_app_fn(pfSourceFn);
	if (ret == FLASH_RETURN_OK)
		ret = m2m_flash_rootcert_access(FLASH_ITEM_ADD, 0, FLASH_ACCESS_OPTION_COMPARE_AFTER, u16Id, app_data_access, u32SourceSize);
	return ret;
}
sint8 m2m_flash_rootcert_remove(uint16 u16Id, uint8 *pu8Identifier, uint32 u32IdentifierSz)
{
	sint8 ret = FLASH_ERR_PARAM;

	if ((pu8Identifier != NULL) && (u32IdentifierSz == 20))
	{
		m2m_memcpy(gau8ItemIdentifier, pu8Identifier, u32IdentifierSz);
		ret = m2m_flash_rootcert_access(FLASH_ITEM_REMOVE, 0, FLASH_ACCESS_OPTION_COMPARE_AFTER, u16Id, NULL, 0);
	}
	return ret;
}
sint8 m2m_flash_rootcert_read(uint16 u16Id, tpfDataAccessFn pfDestFn, uint32 u32DestSize, uint8 *pu8Identifier, uint32 u32IdentifierSz)
{
	sint8 ret = FLASH_RETURN_OK;

	ret = register_app_fn(pfDestFn);
	if (ret == FLASH_RETURN_OK)
	{
		ret = FLASH_ERR_PARAM;
		if ((pu8Identifier != NULL) && (u32IdentifierSz == 20))
		{
			m2m_memcpy(gau8ItemIdentifier, pu8Identifier, u32IdentifierSz);
			ret = m2m_flash_rootcert_access(FLASH_ITEM_READ, 0, 0, u16Id, app_data_access, u32DestSize);
		}
	}
	return ret;
}
sint8 m2m_flash_rootcert_readidx(uint16 u16Id, tpfDataAccessFn pfDestFn, uint32 u32DestSize, uint8 u8Index)
{
	sint8 ret = FLASH_RETURN_OK;

	ret = register_app_fn(pfDestFn);
	if (ret == FLASH_RETURN_OK)
		ret = m2m_flash_rootcert_access(FLASH_ITEM_READIDX, u8Index, 0, u16Id, app_data_access, u32DestSize);
	return ret;
}

void m2m_flash_get_state(tstrFlashState *pstrState)
{
	if (gu8Reset == 0)
	{
		sint8						status = M2M_ERR_FAIL;
		tstrFlashAccessPersistent	strSavedFlashAccess;

		status = spi_flash_read((uint8*)&strSavedFlashAccess, HOST_CONTROL_FLASH_OFFSET, sizeof(tstrFlashAccessPersistent));
		if ((status == M2M_SUCCESS) && (strSavedFlashAccess.u32Signature == FLASH_SIGNATURE))
		{
			switch (strSavedFlashAccess.enuTransferStatus)
			{
			case FLASH_STATUS_ACTIVE:
				if (strSavedFlashAccess.u8ModeFlags & FLASH_MODE_FLAGS_CS_SWITCH)
				{
					// Check to see if switch happened before we were interrupted. If so we had actually completed.
					uint8 target;
					if (image_get_target(&target) == M2M_SUCCESS)
					{
						if ((target == 0) && (strSavedFlashAccess.u8ModeFlags & FLASH_MODE_FLAGS_CS_SWITCH_TARGET))
							gu8Success = 1;
						if ((target > 0) && !(strSavedFlashAccess.u8ModeFlags & FLASH_MODE_FLAGS_CS_SWITCH_TARGET))
							gu8Success = 1;
					}
				}
				gu16LastAccessId = strSavedFlashAccess.u16AppId;
				gu8Changed = !(strSavedFlashAccess.u8ModeFlags & FLASH_MODE_FLAGS_UNCHANGED);
				if (gu8Success == 1)
				{
					strSavedFlashAccess.enuTransferStatus = FLASH_STATUS_DONE;
					winc_flash_write_verify((uint8*)&strSavedFlashAccess, HOST_CONTROL_FLASH_OFFSET, FLASH_SIG_STA_SZ);
				}
				break;
			case FLASH_STATUS_DONE:
				gu16LastAccessId = strSavedFlashAccess.u16AppId;
				gu8Changed = !(strSavedFlashAccess.u8ModeFlags & FLASH_MODE_FLAGS_UNCHANGED);
				gu8Success = 1;
				break;
			default:
				break;
			}
		}
	}
	m2m_memset((uint8*)pstrState, 0, sizeof(tstrFlashState));
	if (gu16LastAccessId)
	{
		pstrState->u16LastAccessId = gu16LastAccessId;
		pstrState->u8Success = gu8Success;
		pstrState->u8Changed = gu8Changed;
	}
	pstrState->u8Init = gu8Init;
	pstrState->u8Reset = gu8Reset;
}
sint8 m2m_flash_init(void)
{
	if (gu8Reset == 0)
	{
		// WINC backup recovery may be needed.
		if (recover_backup() == FLASH_RETURN_OK)
		{
			gu8Init = 1;
			gu8Reset = 1;
			return M2M_SUCCESS;
		}
	}
	return M2M_ERR_FAIL;
}
