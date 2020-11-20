/**
 *
 * \file
 *
 * \brief WINC3400 Crypto API
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef __M2M_CRYPTO_H__
#define __M2M_CRYPTO_H__


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


#include "common/include/nm_common.h"
#include "driver/include/m2m_types.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#define M2M_SHA256_CONTEXT_BUFF_LEN		128
#define M2M_SHA256_DIGEST_LEN			32
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*!
@struct	\
	tstrM2mSha256Ctxt

@brief
	SHA256 context data
*/
typedef struct sha256ctxt{
	uint32	au32Sha256CtxtBuff[M2M_SHA256_CONTEXT_BUFF_LEN/sizeof(uint32)];
} tstrM2mSha256Ctxt;


/*!
@enum	\
	tenuRsaSignStatus

@brief
	RSA Signature status: pass or fail.
*/
typedef enum{
	M2M_RSA_SIGN_OK,
	M2M_RSA_SIGN_FAIL
} tenuRsaSignStatus;


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


#ifdef __cplusplus
     extern "C" {
#endif

/*!
@fn	\
	sint8 m2m_sha256_hash_init(tstrM2mSha256Ctxt *psha256Ctxt);
	
@brief	SHA256 hash initialization

@param[in]	psha256Ctxt
				Pointer to a sha256 context allocated by the caller.
*/
sint8 m2m_sha256_hash_init(tstrM2mSha256Ctxt *psha256Ctxt);


/*!
@fn	\
	sint8 m2m_sha256_hash_update(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Data, uint16 u16DataLength);
	
@brief	SHA256 hash update

@param [in]	psha256Ctxt
				Pointer to the sha256 context.
				
@param [in]	pu8Data
				Buffer holding the data submitted to the hash.
				
@param [in]	u16DataLength
				Size of the data bufefr in bytes.
*/
sint8 m2m_sha256_hash_update(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Data, uint16 u16DataLength);


/*!
@fn	\
	sint8 m2m_sha256_hash_finish(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Sha256Digest);
	
@brief	SHA256 hash finalization

@param[in]	psha256Ctxt
				Pointer to a sha256 context allocated by the caller.
				
@param [in] pu8Sha256Digest
				Buffer allocated by the caller which will hold the resultant SHA256 Digest. It must be allocated no less than M2M_SHA256_DIGEST_LEN.
*/
sint8 m2m_sha256_hash_finish(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Sha256Digest);




/*!
@fn	\
	sint8 m2m_rsa_sign_verify(uint8 *pu8N, uint16 u16NSize, uint8 *pu8E, uint16 u16ESize, uint8 *pu8SignedMsgHash, \
		uint16 u16HashLength, uint8 *pu8RsaSignature);
	
@brief	RSA Signature Verification

	The function shall request the RSA Signature verification from the WINC Firmware for the given message. The signed message shall be 
	compressed to the corresponding hash algorithm before calling this function.
	The hash type is identified by the given hash length. For example, if the hash length is 32 bytes, then it is SHA256.

@param[in]	pu8N
				RSA Key modulus n.
				
@param[in]	u16NSize
				Size of the RSA modulus n in bytes.
				
@param[in]	pu8E
				RSA public exponent.
				
@param[in]	u16ESize
				Size of the RSA public exponent in bytes.

@param[in]	pu8SignedMsgHash
				The hash digest of the signed message.
				
@param[in]	u16HashLength
				The length of the hash digest.
				
@param[out] pu8RsaSignature
				Signature value to be verified.
*/
sint8 m2m_rsa_sign_verify(uint8 *pu8N, uint16 u16NSize, uint8 *pu8E, uint16 u16ESize, uint8 *pu8SignedMsgHash, 
						  uint16 u16HashLength, uint8 *pu8RsaSignature);


/*!
@fn	\
	sint8 m2m_rsa_sign_gen(uint8 *pu8N, uint16 u16NSize, uint8 *pu8d, uint16 u16dSize, uint8 *pu8SignedMsgHash, \
		uint16 u16HashLength, uint8 *pu8RsaSignature);
	
@brief	RSA Signature Generation

	The function shall request the RSA Signature generation from the WINC Firmware for the given message. The signed message shall be 
	compressed to the corresponding hash algorithm before calling this function.
	The hash type is identified by the given hash length. For example, if the hash length is 32 bytes, then it is SHA256.

@param[in]	pu8N
				RSA Key modulus n.
				
@param[in]	u16NSize
				Size of the RSA modulus n in bytes.
				
@param[in]	pu8d
				RSA private exponent.
				
@param[in]	u16dSize
				Size of the RSA private exponent in bytes.

@param[in]	pu8SignedMsgHash
				The hash digest of the signed message.
				
@param[in]	u16HashLength
				The length of the hash digest.
				
@param[out] pu8RsaSignature
				Pointer to a user buffer allocated by teh caller shall hold the generated signature.
*/
sint8 m2m_rsa_sign_gen(uint8 *pu8N, uint16 u16NSize, uint8 *pu8d, uint16 u16dSize, uint8 *pu8SignedMsgHash, 
					   uint16 u16HashLength, uint8 *pu8RsaSignature);
#ifdef __cplusplus
}
#endif


#endif /* __M2M_CRYPTO_H__ */