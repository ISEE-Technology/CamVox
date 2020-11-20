/**
 *
 * \file
 *
 * \brief WINC3400 IoT Application Interface Internal Types.
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

#ifndef __M2M_WIFI_TYPES_H__
#define __M2M_WIFI_TYPES_H__


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifndef	_BOOT_
#ifndef _FIRMWARE_
#include "common/include/nm_common.h"
#else
#include "m2m_common.h"
#endif
#endif


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/**@defgroup  VERSION VERSION
 */
/**@defgroup  VERSIONDEF Defines
 @ingroup VERSION
 */

/**@{*/

/*
	Layout of HIF_LEVEL fields
	|	x	x	x	x	x	x	|	x	x	x	x	x	x	x	x	|
	|			13:8			|				7:0					|
	|			HIF_MAJOR		|				HIF_MINOR			|
	|						HIF_LEVEL								|
	
	HIF compatibility (checked by driver after wifi boot).

	Check Firmware HIF_MAJOR
		|___
		|	Does not match Driver	- Recommend either:
		|								> checking compatibility of Cortus image stored in inactive partition
		|									(m2m_wifi_check_ota_rb()) then swapping to it using OTA rollback or switch.
		|								> updating Host with out of band method.
		|___
			Matches Driver			- All APIs can be attempted.
				|					- Cortus image can be updated (OTA download)
				|					- New Host image can be retrieved (socket recv)
				|
			Check Firmware HIF_MINOR
				|___
				|	Less than Driver	- Driver may refuse to execute some APIs (API would return M2M_ERR_SEND).
				|						- Recommend updating Cortus image using OTA download to use newer Firmware.
				|___
				|	Equal to Driver		- Driver supports all APIs.
				|___
					Greater than Driver	- Driver supports all APIs.
										- Recommend reviewing potential benefit of updating Host to use newer Driver.

	When an OTA update involving HIF_MAJOR change is required, the following order is recommended:
	1. m2m_ota_start_update()		to store new Cortus image in inactive partition.
	2. Socket recv()				to retrieve new Host image (including new WINC Driver).
	3. Application code				to make Host image available in host flash.
	4. m2m_ota_switch_firmware()	to switch active/inactive partition (will take effect after system_reset()).
	5. system_reset()				to restart with new WINC Driver / Firmware.
	Note that in the event of (unintentional) system reset after step (3), the remaining steps (4) and (5) should still be
	run after the reset.
*/
/* Selection of HIF_INFO
*/
#define M2M_HIF_INFO_SHIFT		(0)
#define M2M_HIF_INFO_MASK		(0xffff)
/* Subdivision of HIF_INFO
*/ 
#define M2M_HIF_BLOCK_SHIFT		(14)
#define M2M_HIF_BLOCK_MASK		(0x3)
#define M2M_HIF_LEVEL_SHIFT		(0)
#define M2M_HIF_LEVEL_MASK		(0x3fff)
/* Subdivision of HIF_LEVEL
*/
#define M2M_HIF_MAJOR_SHIFT		(8)
#define M2M_HIF_MAJOR_MASK		(0x3f)
#define M2M_HIF_MINOR_SHIFT		(0)
#define M2M_HIF_MINOR_MASK		(0xff)

#define M2M_GET_HIF_INFO(hif_ver)	((uint16)(((hif_ver) >> M2M_HIF_INFO_SHIFT) & M2M_HIF_INFO_MASK))
#define M2M_GET_HIF_BLOCK(hif_info)	((uint8)(((hif_info) >> M2M_HIF_BLOCK_SHIFT) & M2M_HIF_BLOCK_MASK))
#define M2M_GET_HIF_LEVEL(hif_info)	((uint16)(((hif_info) >> M2M_HIF_LEVEL_SHIFT) & M2M_HIF_LEVEL_MASK))
#define M2M_GET_HIF_MAJOR(hif_info)	((uint8)(((hif_info) >> M2M_HIF_MAJOR_SHIFT) & M2M_HIF_MAJOR_MASK))
#define M2M_GET_HIF_MINOR(hif_info)	((uint8)(((hif_info) >> M2M_HIF_MINOR_SHIFT) & M2M_HIF_MINOR_MASK))

#define M2M_MAKE_HIF_INFO(hif_level) ( \
	(((uint16)(hif_level) & M2M_HIF_LEVEL_MASK)			<< M2M_HIF_LEVEL_SHIFT)	|	\
	(((uint16)M2M_HIF_BLOCK_VALUE & M2M_HIF_BLOCK_MASK) << M2M_HIF_BLOCK_SHIFT)	)

/*======*======*======*======*
  HIF LEVEL
 *======*======*======*======*/

#define M2M_HIF_BLOCK_VALUE								(2)
/*!< Drv/Fw blocking compatibility check.
 */
#define M2M_HIF_MAJOR_VALUE								(1)
/*!< Drv/Fw major compatibility check.
*/
#define M2M_HIF_MINOR_VALUE								(3)
/*!< Drv/Fw minor compatibility check.
*/
#define M2M_HIF_LEVEL	(													\
	((M2M_HIF_MAJOR_VALUE & M2M_HIF_MAJOR_MASK) << M2M_HIF_MAJOR_SHIFT)	|	\
	((M2M_HIF_MINOR_VALUE & M2M_HIF_MINOR_MASK) << M2M_HIF_MINOR_SHIFT)		\
	)
/*!< HIF Level (Major/Minor) for Drv/Fw compatibility.
*/

/*======*======*======*======*
  DRIVER VERSION NO INFO
 *======*======*======*======*/

#define M2M_DRIVER_VERSION_MAJOR_NO 					(1)
/*!< Driver Major release version number.
*/
#define M2M_DRIVER_VERSION_MINOR_NO						(0)
/*!< Driver Minor release version number.
*/
#define M2M_DRIVER_VERSION_PATCH_NO						(8)
/*!< Driver patch release version number.
*/

/**@}*/

/**@defgroup  WLANDefines Defines
 * @ingroup m2m_wifi
 */
/**@{*/

#define M2M_BUFFER_MAX_SIZE								(1600UL - 4)
/*!< Maximum size for the shared packet buffer.
 */


#define M2M_MAC_ADDRES_LEN								6
/*!< Size of the MAC address.
 */

#define M2M_ETHERNET_HDR_OFFSET							34
/*!< Offset of the ethernet header within the WLAN Tx Buffer.
 */


#define M2M_ETHERNET_HDR_LEN							14
/*!< Length of the ethernet header in bytes.
*/


#define M2M_MAX_SSID_LEN 								33
/*!< Maximum size for the Wi-Fi SSID including the NULL termination.
 */


#define M2M_MAX_PSK_LEN									65
/*!< Maximum size for the WPA PSK including the NULL termination.
 */

#define M2M_DEVICE_NAME_MAX								48
/*!< Maximum Size for the device name including the NULL termination.
 */
 

#define M2M_LISTEN_INTERVAL 							1
/*!< The STA uses the Listen Interval parameter to indicate to the AP how
	many beacon intervals it shall sleep before it retrieves the queued frames
	from the AP. 
*/


#define M2M_1X_USR_NAME_MAX								21
/*!< The maximum size of the user name including the NULL termination. 
	It is used for RADIUS authentication in case of connecting the device to
	an AP secured with WPA-Enterprise. 
*/


#define M2M_1X_PWD_MAX									41
/*!< The maximum size of the password including the NULL termination. 
	It is used for RADIUS authentication in case of connecting the device to
	an AP secured with WPA-Enterprise. 
*/

#define M2M_CUST_IE_LEN_MAX								252
/*!< Maximum size for the Information Element.
*/


#define M2M_CONFIG_CMD_BASE									1
/*!< The base value of all the Host configuration commands opcodes.
*/
#define M2M_STA_CMD_BASE									40
/*!< The base value of all the Station mode host commands opcodes.
*/
#define M2M_AP_CMD_BASE										70
/*!< The base value of all the Access Point mode host commands opcodes.
*/
#define M2M_P2P_CMD_BASE									90
/*!< The base value of all the P2P mode host commands opcodes.
*/
#define M2M_OTA_CMD_BASE									100
/*!< The base value of all the OTA mode host commands opcodes.
*/
#define M2M_SERVER_CMD_BASE									120
/*!< The base value of all the Power Save mode host commands codes.
*/


#define WEP_40_KEY_STRING_SIZE 							((uint8)10)
/*!< Indicates the WEP key size for the 40 bit string passphrase.
*/

#define WEP_104_KEY_STRING_SIZE 						((uint8)26)
/*!< Indicates the WEP key size for the 104 bit string passphrase.
*/
#define WEP_KEY_MAX_INDEX								((uint8)4)
/*!< Indicates the maximum key index value for WEP authentication.
*/
#define M2M_SCAN_DEFAULT_NUM_SLOTS							(2)
/*!< The default number of scan slots used by the WINC board.
*/
#define M2M_SCAN_DEFAULT_SLOT_TIME							(20)
/*!< The active scan slot default duration in ms.
*/
#define M2M_SCAN_DEFAULT_PASSIVE_SLOT_TIME					(300)
/*!< The passive scan slot default duration in ms.
*/
#define M2M_SCAN_DEFAULT_NUM_PROBE							(2)
/*!< The default number of probes per slot.
*/
#define M2M_FASTCONNECT_DEFAULT_RSSI_THRESH					(-45)
/*!< The default threshold RSSI for fast reconnection to an AP.
*/
#define M2M_SCAN_FAIL									((uint8)1)
/*!< Indicates the WINC board has failed to perform the scan operation.
*/
#define M2M_JOIN_FAIL									((uint8)2)
/*!< Indicates the WINC board has failed to join the BSS.
*/
#define M2M_AUTH_FAIL									((uint8)3)
/*!< Indicates the WINC board has failed to authenticate with the AP.
*/
#define M2M_ASSOC_FAIL									((uint8)4)
/*!< Indicates the WINC board has failed to associate with the AP.
*/

#define M2M_SCAN_ERR_WIFI								((sint8)-2)
/*!< Currently not used.
*/
#define M2M_SCAN_ERR_IP									((sint8)-3)
/*!< Currently not used.
*/
#define M2M_SCAN_ERR_AP									((sint8)-4)	
/*!< Currently not used.
*/
#define M2M_SCAN_ERR_P2P								((sint8)-5)
/*!< Currently not used.
*/
#define M2M_SCAN_ERR_WPS								((sint8)-6)	
/*!< Currently not used.
*/

/*======*======*======*======*
	MONTIORING MODE DEFINITIONS
 *======*======*======*======*/

#define M2M_WIFI_FRAME_TYPE_ANY							0xFF
/*!< Receive any frame type when configured as Monitor Mode.
*/
#define M2M_WIFI_FRAME_SUB_TYPE_ANY						0xFF
/*!< Receive frames with any sub type when configured as Monitor Mode.
*/

/*======*======*======*======*
	TLS DEFINITIONS
 *======*======*======*======*/
#define TLS_FILE_NAME_MAX								48
/*!<  Maximum length for each TLS certificate file name.
*/
#define TLS_SRV_SEC_MAX_FILES							8
/*!<  Maximum number of certificates allowed in TLS_SRV section.
*/
#define TLS_SRV_SEC_START_PATTERN_LEN					8
/*!<  Length of certificate struct start pattern.
*/

/**@}*/

/**@defgroup  OTADEFINE Defines
 * @ingroup OTAAPI
 */
 /* @{ */

/*======*======*======*======*
	OTA DEFINITIONS
 *======*======*======*======*/
 
#define OTA_STATUS_VALID					(0x12526285)
/*!< 
	Magic value in the control structure for a valid image after ROLLBACK.
*/
#define OTA_STATUS_INVALID					(0x23987718)
/*!< 
	Magic value in the control structure for a invalid image after ROLLBACK.
*/
#define OTA_MAGIC_VALUE						(0x1ABCDEF9)
/*!< 
	Magic value set at the beginning of the OTA image header.
*/
#define OTA_SHA256_DIGEST_SIZE				(32)
/*!< 
	SHA256 digest size in the OTA image.
	The SHA256 digest is set at the beginning of image before the OTA header.
*/

#define OTA_SUCCESS							(0)
/*!<
	OTA success status.
*/
#define OTA_ERR_WORKING_IMAGE_LOAD_FAIL		((sint8)-1)
/*!<
	Failure to load the firmware image.
*/
#define OTA_ERR_INVAILD_CONTROL_SEC			((sint8)-2)
/*!<
	Control structure is being corrupted.
*/
#define M2M_ERR_OTA_SWITCH_FAIL				((sint8)-3)
/*!<
	Failure to switch to the updated image, due to this being invalid.
*/
#define M2M_ERR_OTA_START_UPDATE_FAIL		((sint8)-4)
/*!<
	Failure to start the OTA update.
	This could be due to a connection failure or a problem with the integrity of the image.
*/
#define M2M_ERR_OTA_ROLLBACK_FAIL			((sint8)-5)
/*!<
	Invalid roll-back image.
*/
#define M2M_ERR_OTA_INVAILD_FLASH_SIZE		((sint8)-6)
/*!<
	The OTA Support requires 8MB flash size. This error indicates the current flash is less than 8M.
*/
#define M2M_ERR_OTA_INVAILD_ARG				((sint8)-7)
/*!<
	Invalid argument in any OTA Function
*/
/**@}*/

/**
* @addtogroup WlanEnums
*/
 /**@{*/ 

/*!
@enum	\
	tenuM2mReqGroup

@brief
*/
typedef enum{
	M2M_REQ_GROUP_MAIN = 0,
	M2M_REQ_GROUP_WIFI,
	M2M_REQ_GROUP_IP,
	M2M_REQ_GROUP_HIF,
	M2M_REQ_GROUP_OTA,
	M2M_REQ_GROUP_SSL,
	M2M_REQ_GROUP_SIGMA
}tenuM2mReqGroup;

/*!
@enum	\
	tenuM2mReqpkt

@brief
*/
typedef enum{
	M2M_REQ_CONFIG_PKT,
	M2M_REQ_DATA_PKT = 0x80 /*BIT7*/
}tenuM2mReqpkt;

/*!
@enum	\
	tenuM2mWepKeyIndex
	
@brief
	
*/
typedef enum {
	M2M_WIFI_WEP_KEY_INDEX_1 = ((uint8) 1),
	/*!< Index 1 for WEP key Authentication */
	M2M_WIFI_WEP_KEY_INDEX_2,
	/*!< Index 2 for WEP key Authentication */
	M2M_WIFI_WEP_KEY_INDEX_3,
	/*!< Index 3 for WEP key Authentication */
	M2M_WIFI_WEP_KEY_INDEX_4,
	/*!< Index 4 for WEP key Authentication */
}tenuM2mWepKeyIndex;

/*!
@enum	\
	tenuM2mDefaultConnErrcode

@brief

*/
typedef enum {
	M2M_DEFAULT_CONN_INPROGRESS = ((sint8)-23),
	/*!< Failure response due to another connection being already in progress */
	M2M_DEFAULT_CONN_FAIL,
	/*!< Failure to connect to the cached network */
	M2M_DEFAULT_CONN_SCAN_MISMATCH,
	/*!< Failure to find any of the cached networks in the scan results. */
	M2M_DEFAULT_CONN_EMPTY_LIST
	/*!< Failure due to empty network list. */
}tenuM2mDefaultConnErrcode;


/*!
@enum	\
	tenuM2mConnChangedErrcode

@brief

*/
typedef enum {
	 M2M_ERR_SCAN_FAIL = ((uint8)1),
	/*!< Failure to perform the scan operation.	*/
	 M2M_ERR_JOIN_FAIL,
	/*!< Failure to join the BSS. */
	 M2M_ERR_AUTH_FAIL,
	/*!< Failure to authenticate with the AP. */
	 M2M_ERR_ASSOC_FAIL,
	/*!< Failure to associate with the AP. */
	 M2M_ERR_CONN_INPROGRESS,
	 /*!< Failure due to another connection being in progress. */
}tenuM2mConnChangedErrcode;


/*!
@enum	\
	tenuM2mSetGainsErrcode

@brief

*/
typedef enum {
	M2M_ERR_GAIN_TABLE_INVALID = ((sint8)-10),
	/*!< Failure response due to trying to use an invalid table */
	M2M_ERR_READ_GAIN_TABLE,
	/*!< Failure to read gains from flash */
}tenuM2mSetGainsErrcode;

/*!
@enum	\
	tenuM2mPwrMode
	
@brief
	
*/
typedef enum {
	PWR_AUTO = ((uint8) 1),
	/*!< Automatic power mode. */
	PWR_LOW1,
	/*!< Low power mode #1. RX current 60mA.*/
	PWR_LOW2,
	/*!< Low power mode #2, RX current 55mA, sensitivity is less by 3dBm*/
	PWR_HIGH,
	/*!< High power mode: RX current 100mA.*/
}tenuM2mPwrMode;

/*!
@enum	\
	tenuM2mTxPwrLevel
	
@brief
	
*/
typedef enum {
	TX_PWR_HIGH = ((uint8) 1),
	/*!< PPA Gain 6dbm	PA Gain 18dbm */
	TX_PWR_MED,
	/*!< PPA Gain 6dbm	PA Gain 12dbm */
	TX_PWR_LOW,
	/*!< PPA Gain 6dbm	PA Gain 6dbm */
}tenuM2mTxPwrLevel;

/*!
@enum	\
	tenuM2mConfigCmd
	
@brief
	This enum contains all the host commands used to configure the WINC board.
	
*/
typedef enum {
	M2M_WIFI_REQ_RESTART = M2M_CONFIG_CMD_BASE,
	/*!< Restart the WINC MAC layer, it's doesn't restart the IP layer.	*/
	M2M_WIFI_REQ_SET_MAC_ADDRESS,
	/*!< Set the WINC mac address (not possible for production effused boards).	*/
	M2M_WIFI_REQ_CURRENT_RSSI,
	/*!< Request the current connected AP RSSI.	*/
	M2M_WIFI_RESP_CURRENT_RSSI,
	/*!< Response to M2M_WIFI_REQ_CURRENT_RSSI with the RSSI value. */
	M2M_WIFI_REQ_RESTRICT_BLE,
	/*!< Request restrict ble.	*/
	M2M_WIFI_REQ_UNRESTRICT_BLE,
	/*!< Request unrestrict ble.	*/
	M2M_WIFI_REQ_GET_CONN_INFO,
	/*!< Request connection information. */
	M2M_WIFI_RESP_CONN_INFO,
	/*!< Response to M2M_WIFI_REQ_GET_CONN_INFO with the connection information. */
	M2M_WIFI_REQ_SET_DEVICE_NAME,
	/*!< Request to set WINC device name property. */
	M2M_WIFI_REQ_START_PROVISION_MODE,
	/*!< Request to start provisioning mode. */
	M2M_WIFI_RESP_PROVISION_INFO,
	/*!< Response to the host with the provisioning information.*/
	M2M_WIFI_REQ_STOP_PROVISION_MODE,
	/*!< Request to stop provision mode. */
	M2M_WIFI_REQ_SET_SYS_TIME,
	/*!< Request to set system time. */
	M2M_WIFI_REQ_ENABLE_SNTP_CLIENT,
	/*!< Request to enable the simple network time protocol to get the
		time from the Internet. This is required for security purposes. */
	M2M_WIFI_REQ_DISABLE_SNTP_CLIENT,
	/*!< Request to disable the simple network time protocol for applications that
		do not need it. */
	M2M_WIFI_RESP_MEMORY_RECOVER,
	/*!< Reserved for debuging */
	M2M_WIFI_REQ_CUST_INFO_ELEMENT,
	/*!< Request to add custom information to the Beacons IE. */
	M2M_WIFI_REQ_SCAN,
	/*!< Request scan command. */
	M2M_WIFI_RESP_SCAN_DONE,
	/*!< Response to notify scan complete. */
	M2M_WIFI_REQ_SCAN_RESULT,
	/*!< Request for scan results. */
	M2M_WIFI_RESP_SCAN_RESULT,
	/*!< Response to provide the scan results.	*/
	M2M_WIFI_REQ_SET_SCAN_OPTION,
	/*!< Request to set scan options "slot time, slot number .. etc". 	*/
	M2M_WIFI_REQ_SET_SCAN_REGION,
	/*!< Request to set scan region. */
	M2M_WIFI_REQ_SET_POWER_PROFILE,
	/*!< Request to set the Power Profile. */
	M2M_WIFI_REQ_SET_TX_POWER,
	/*!< Request to set the TX Power. */
	M2M_WIFI_REQ_SET_BATTERY_VOLTAGE,
	/*!< Request to set the Battery Voltage. */
	M2M_WIFI_REQ_SET_ENABLE_LOGS,
	/*!< Request to enable logs. */
	M2M_WIFI_REQ_GET_SYS_TIME,
	/*!< Request to get system time. */
	M2M_WIFI_RESP_GET_SYS_TIME,
	/*!< Response to retrieve the system time. */
	M2M_WIFI_REQ_SEND_ETHERNET_PACKET,
	/*!< Request to send Ethernet packet in bypass mode. */
	M2M_WIFI_RESP_ETHERNET_RX_PACKET,
	/*!< Response to receive an Ethernet packet in bypass mode. */
	M2M_WIFI_REQ_SET_MAC_MCAST,
	/*!< Request to set multicast filters. */
	M2M_WIFI_REQ_BLE_API_SEND,
	/*!< Request to send an Encapsulated BLE API MSG */
	M2M_WIFI_RESP_BLE_API_RECV,
	/*!< Response to receive an Encapsulated BLE API MSG */
	M2M_WIFI_REQ_SET_GAIN_TABLE,
	/*!< Request to use Gain table from Flash */
	M2M_WIFI_RESP_SET_GAIN_TABLE,
	/*!< Response to fail to use Gain table from Flash */
	M2M_WIFI_REQ_PASSIVE_SCAN,
	/*!< Request a passive scan.
	*/
	M2M_WIFI_MAX_CONFIG_ALL
}tenuM2mConfigCmd;

/*!
@enum	\
	tenuM2mStaCmd
	
@brief
	This enum contains all the WINC commands while in Station mode.
*/
typedef enum {
	M2M_WIFI_REQ_CONNECT = M2M_STA_CMD_BASE,
	/*!< Request to connect with an specified AP. */
	M2M_WIFI_REQ_DEFAULT_CONNECT,
	/*!< Request to connect with a cached AP. */
	M2M_WIFI_RESP_DEFAULT_CONNECT,
	/*!< Response for the default connect.*/
	M2M_WIFI_REQ_DISCONNECT,
	/*!< Request to disconnect from the AP.	*/
	M2M_WIFI_RESP_CON_STATE_CHANGED,
	/*!< Response to indicate a change in the connection state.	*/
	M2M_WIFI_REQ_SLEEP,
	/*!< Request to sleep. */
	M2M_WIFI_REQ_WPS_SCAN,
	/*!< Request to WPS scan. */
	M2M_WIFI_REQ_WPS,
	/*!< Request to start WPS. */
	M2M_WIFI_REQ_START_WPS,
	/*!< This command is for internal use by the WINC and 
		should not be used by the host driver. */
	M2M_WIFI_REQ_DISABLE_WPS,
	/*!< Request to disable WPS. */
	M2M_WIFI_REQ_DHCP_CONF,
	/*!< Response to indicate the obtained IP address.*/
	M2M_WIFI_RESP_IP_CONFIGURED,
	/*!< This command is for internal use by the WINC and 
		should not be used by the host driver. */
	M2M_WIFI_RESP_IP_CONFLICT,
	/*!< Response to indicate a conflict in obtained IP address.
		The user should re attempt the DHCP request. */
	M2M_WIFI_REQ_ENABLE_MONITORING,
	/*!< Request to enable monitor mode. */
	M2M_WIFI_REQ_DISABLE_MONITORING,
	/*!< Request to disable monitor mode. */
	M2M_WIFI_RESP_WIFI_RX_PACKET,
	/*!< Response to indicate a packet was received in monitor mode. */
	M2M_WIFI_REQ_SEND_WIFI_PACKET,
	/*!< Request to send a packet in monitor mode. */
	M2M_WIFI_REQ_LSN_INT,
	/*!< Request to set the listen interval. */
	M2M_WIFI_REQ_DOZE,
	/*!< Request to doze */
	M2M_WIFI_REQ_GET_PRNG,
	/*!< Request PRNG. */
	M2M_WIFI_RESP_GET_PRNG,
	/*!< Response for PRNG. */
	M2M_WIFI_MAX_STA_ALL,
} tenuM2mStaCmd;

/*!
@enum	\
	tenuM2mApCmd

@brief
	This enum contains all the WINC commands while in AP mode.
*/
typedef enum {
	M2M_WIFI_REQ_ENABLE_AP = M2M_AP_CMD_BASE,
	/*!< Request to enable AP mode.	*/
	M2M_WIFI_REQ_DISABLE_AP,
	/*!< Request to disable AP mode. */
	M2M_WIFI_MAX_AP_ALL,
}tenuM2mApCmd;

/*!
@enum	\
	tenuM2mP2pCmd
	
@brief
	This enum contains all the WINC commands while in P2P mode.
*/
typedef enum {
	M2M_WIFI_REQ_P2P_INT_CONNECT = M2M_P2P_CMD_BASE,
	/*!< This command is for internal use by the WINC and 
		should not be used by the host driver. */
	M2M_WIFI_REQ_ENABLE_P2P,
	/*!< Request to enable P2P mode.*/
	M2M_WIFI_REQ_DISABLE_P2P,
	/*!< Request to disable P2P mode. */
	M2M_WIFI_REQ_P2P_REPOST,
	/*!< This command is for internal use by the WINC and 
		should not be used by the host driver.
	*/
	M2M_WIFI_MAX_P2P_ALL,
}tenuM2mP2pCmd;


/*!
@enum	\
	tenuM2mServerCmd

@brief
	These commands are currently not supported.
*/
typedef enum {
	M2M_WIFI_REQ_CLIENT_CTRL = M2M_SERVER_CMD_BASE,
	/*!< Currently not supported.*/
	M2M_WIFI_RESP_CLIENT_INFO,
	/*!< Currently not supported.*/
	M2M_WIFI_REQ_SERVER_INIT,
	/*!< Currently not supported.*/
	M2M_WIFI_MAX_SERVER_ALL
}tenuM2mServerCmd;




/*!
@enum	\
	tenuM2mIpCmd
	
@brief
	This enum contains all the WINC commands related to IP.
*/
typedef enum {
	M2M_IP_REQ_STATIC_IP_CONF = ((uint8) 10),
	/*!< Request to set static IP.*/
	M2M_IP_REQ_ENABLE_DHCP,
	/*!< Request to enable DHCP.*/
	M2M_IP_REQ_DISABLE_DHCP
	/*!< Request to disable DHCP.*/
} tenuM2mIpCmd;

/*!
@enum	\
	tenuM2mSigmaCmd
	
@brief
	This enum contains all the WINC commands related to Sigma.
*/
typedef enum {
	M2M_SIGMA_ENABLE = ((uint8) 3),
	/*!< Enable Sigma.*/
	M2M_SIGMA_TA_START,
	/*!< Start the traffic agent.*/
	M2M_SIGMA_TA_STATS,
	/*!< Get traffic stadistics.*/
	M2M_SIGMA_TA_RECEIVE_STOP,
	/*!< Stop receiving from the traffic agent.*/
	M2M_SIGMA_ICMP_ARP,
	/*!< Send ARP.*/
	M2M_SIGMA_ICMP_RX,
	/*!< Receive ICMP.*/
	M2M_SIGMA_ICMP_TX,
	/*!< Transmit ICMP.*/
	M2M_SIGMA_UDP_TX,
	/*!< Transmit UDP.*/
	M2M_SIGMA_UDP_TX_DEFER,
	/*!< Transmit UDP defer.*/
	M2M_SIGMA_SECURITY_POLICY,
	/*!< Set security policy.*/
	M2M_SIGMA_SET_SYSTIME
	/*!< Set system time.*/
} tenuM2mSigmaCmd;

/*!
@enum	\
	tenuM2mSslCmd
	
@brief
	This enum contains WINC commands related to TLS handshake.
*/
typedef enum{
	M2M_SSL_REQ_CERT_VERIF,
	M2M_SSL_REQ_ECC,
	M2M_SSL_RESP_ECC,
	M2M_SSL_RSV,
	M2M_SSL_REQ_WRITE_OWN_CERTS,
	M2M_SSL_REQ_SET_CS_LIST,
	M2M_SSL_RESP_SET_CS_LIST,
	M2M_SSL_RESP_WRITE_OWN_CERTS
}tenuM2mSslCmd;


/*!
@enum	\
	tenuM2mConnState
	
@brief
	This enum contains all the Wi-Fi connection states.
*/
typedef enum {
	M2M_WIFI_DISCONNECTED = 0,
	/*!< Wi-Fi state is disconnected. */
	M2M_WIFI_CONNECTED,
	/*!< Wi-Fi state is connected. */
	M2M_WIFI_UNDEF = 0xff
	/*!< Undefined Wi-Fi State. */
}tenuM2mConnState;

/*!
@enum	\
	tenuM2mSecType
	
@brief
	This enum contains all the supported Wi-Fi security types.
*/
typedef enum {
	M2M_WIFI_SEC_INVALID = 0,
	/*!< Invalid security type.	*/
	M2M_WIFI_SEC_OPEN,
	/*!< Wi-Fi network is not secured. */
	M2M_WIFI_SEC_WPA_PSK,
	/*!< Wi-Fi network is secured with WPA/WPA2 personal(PSK). */
	M2M_WIFI_SEC_WEP,
	/*!< Security type WEP (40 or 104) OPEN OR SHARED. */
	M2M_WIFI_SEC_802_1X
	/*!< Wi-Fi network is secured with WPA/WPA2 Enterprise.IEEE802.1x user-name/password authentication. */
}tenuM2mSecType;


/*!
@enum	\
	tenuM2mSecType
	
@brief
	This enum contains all the supported Wi-Fi SSID types.
*/
typedef enum {
	SSID_MODE_VISIBLE = 0,
	/*!< SSID is visible to others.	*/
	SSID_MODE_HIDDEN
	/*!< SSID is hidden. */
}tenuM2mSsidMode;

/*!
@enum	\
	tenuM2mScanCh
	
@brief
	This enum contains all the Wi-Fi RF channels.
*/
typedef enum {
	M2M_WIFI_CH_1 = ((uint8) 1),
	/*!< Channel 1. */
	M2M_WIFI_CH_2,
	/*!< Channel 2. */
	M2M_WIFI_CH_3,
	/*!< Channel 3. */
	M2M_WIFI_CH_4,
	/*!< Channel 4. */
	M2M_WIFI_CH_5,
	/*!< Channel 5. */
	M2M_WIFI_CH_6,
	/*!< Channel 6. */
	M2M_WIFI_CH_7,
	/*!< Channel 7. */
	M2M_WIFI_CH_8,
	/*!< Channel 8. */
	M2M_WIFI_CH_9,
	/*!< Channel 9. */
	M2M_WIFI_CH_10,
	/*!< Channel 10. */
	M2M_WIFI_CH_11,
	/*!< Channel 11. */
	M2M_WIFI_CH_12,
	/*!< Channel 12. */
	M2M_WIFI_CH_13,
	/*!< Channel 13. */
	M2M_WIFI_CH_14,
	/*!< Channel 14. */
	M2M_WIFI_CH_ALL = ((uint8) 255)
}tenuM2mScanCh;

/*!
@enum	\
	tenuM2mScanRegion
	
@brief
	This enum contains all the Wi-Fi channel regions.
*/
typedef enum {
	REG_CH_1 = ((uint16) 1 << 0),
	/*!< Region channel 1. */
	REG_CH_2 = ((uint16) 1 << 1),
	/*!< Region channel 2. */
	REG_CH_3 = ((uint16) 1 << 2),
	/*!< Region channel 3. */
	REG_CH_4 = ((uint16) 1 << 3),
	/*!< Region channel 4. */
	REG_CH_5 = ((uint16) 1 << 4),
	/*!< Region channel 5. */
	REG_CH_6 = ((uint16) 1 << 5),
	/*!< Region channel 6. */
	REG_CH_7 = ((uint16) 1 << 6),
	/*!< Region channel 7. */
	REG_CH_8 = ((uint16) 1 << 7),
	/*!< Region channel 8. */
	REG_CH_9 = ((uint16) 1 << 8),
	/*!< Region channel 9. */
	REG_CH_10 = ((uint16) 1 << 9),
	/*!< Region channel 10. */
	REG_CH_11 = ((uint16) 1 << 10),
	/*!< Region channel 11. */
	REG_CH_12 = ((uint16) 1 << 11),
	/*!< Region channel 12. */
	REG_CH_13 = ((uint16) 1 << 12),
	/*!< Region channel 13. */
	REG_CH_14 = ((uint16) 1 << 13),
	/*!< Region channel 14. */
	REG_CH_ALL = ((uint16) 0x3FFF),
	/*!< Region for all channels. */
	NORTH_AMERICA = ((uint16) 0x7FF),
	/*!< North America region with 11 channels*/
	EUROPE		=   ((uint16) 0x1FFF),
	/*!<Europe region with 13 channels */
	ASIA		=   ((uint16) 0x3FFF)
	/*!<Asia region with 14 channels */
}tenuM2mScanRegion;


/*!
@enum	\
	tenuPowerSaveModes
	
@brief
	This enum contains all the supported Wi-Fi Power Save modes.
*/
typedef enum {
	M2M_NO_PS,
	/*!< Power save is disabled. */
	M2M_PS_AUTOMATIC,
	/*!This powersave mode is not supported on WINC3400
	*/
	M2M_PS_H_AUTOMATIC,
	/*!This powersave mode is not supported on WINC3400
	*/
	M2M_PS_DEEP_AUTOMATIC,
	/*!< Power save is done automatically by the WINC.
		Achieves the highest possible power save.
	*/
	M2M_PS_MANUAL
	/*!This powersave mode is not supported on WINC3400
	*/
}tenuPowerSaveModes;

/*!
@enum	\
	tenuM2mWifiMode
	
@brief
	This enum contains all the supported Wi-Fi Operation Modes.
*/
typedef enum {
	M2M_WIFI_MODE_NORMAL = ((uint8) 1),
	/*!< Customer firmware.
	 */
	M2M_WIFI_MODE_CONFIG,
	/*!< Production test firmware.
	 */
}tenuM2mWifiMode;

/*!
@enum	\
	tenuWPSTrigger
	
@brief
	This enum contains the WPS triggering methods.
*/
typedef enum{
	WPS_PIN_TRIGGER = 0,
	/*!< WPS is triggered in PIN method.*/
	WPS_PBC_TRIGGER = 4
	/*!< WPS is triggered via push button.*/
}tenuWPSTrigger;

/*!
@enum	\
	tenuRootCertPubKeyType

@brief
	This enum contains the supported public key types for TLS root certificates.
*/
typedef enum{
	ROOT_CERT_PUBKEY_RSA		= 1,
	/*!< RSA public key.*/
	ROOT_CERT_PUBKEY_ECDSA		= 2
	/*!< ECDSA public key.*/
}tenuRootCertPubKeyType;
/**@}*/

/*!
@struct	\	
	tstrM2mPwrState

@brief
	This struct stores the Power Save modes.
*/
typedef struct {
	uint8	u8PwrMode; 
	/*!< Power Save Mode */
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mPwrMode;

/*!
@struct	\	
	tstrM2mTxPwrLevel

@brief
	This struct stores the Tx Power levels.
*/
typedef struct {
	uint8	u8TxPwrLevel; 
	/*!< Tx power level	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mTxPwrLevel;

/*!
@struct	\	
	tstrM2mEnableLogs

@brief
	This struct stores logging information.
*/
typedef struct {
	uint8	u8Enable; 
	/*!< Enable/Disable firmware logs*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mEnableLogs;

/*!
@struct	\	
	tstrM2mBatteryVoltage

@brief
	This struct stores the battery voltage.
*/
typedef struct {
	//Note: on SAMD D21 the size of double is 8 Bytes
	uint16	u16BattVolt; 
	/*!< Battery Voltage */
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mBatteryVoltage;

/*!
@struct	\
	tstrM2mWifiWepParams

@brief
	This struct stores the WEP security key parameters.
*/
typedef struct{
	uint8	u8KeyIndx;
	/*!< WEP key Index.	*/
	uint8	u8KeySz;
	/*!< WEP key Size.	*/
	uint8	au8WepKey[WEP_104_KEY_STRING_SIZE + 1];
	/*!< WEP Key represented as a NULL terminated ASCII string.	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes to keep the structure word alligned.	*/
}tstrM2mWifiWepParams;


/*!
@struct	\
	tstr1xAuthCredentials

@brief
	This struct stores the credentials for the user to authenticate with the AAA server (WPA-Enterprise Mode IEEE802.1x).
*/
typedef struct{
	uint8	au8UserName[M2M_1X_USR_NAME_MAX];
	/*!< User Name. It must be Null terminated string.*/
	uint8	au8Passwd[M2M_1X_PWD_MAX];
	/*!< Password corresponding to the user name. It must be Null terminated string.*/

	#define __PADDING_tstr1xAuthCredentials_ (4 - ((M2M_1X_USR_NAME_MAX + M2M_1X_PWD_MAX) % 4))
	uint8	__PAD__[__PADDING_tstr1xAuthCredentials_];
}tstr1xAuthCredentials;


/*!
@union	\
	tuniM2MWifiAuth

@brief
	This struct stores the Wi-Fi Security Parameters for all supported security modes.
*/
typedef union{
	uint8				au8PSK[M2M_MAX_PSK_LEN];
	/*!< Pre-Shared Key in case of WPA-Personal security. */
	tstr1xAuthCredentials	strCred1x;
	/*!< Credentials for RADIUS server authentication in case of WPA-Enterprise security. */
	tstrM2mWifiWepParams	strWepInfo;
	/*!< WEP key parameters in case of WEP security. */

	#define __PADDING_tuniM2MWifiAuth_ (4 - ((M2M_MAX_PSK_LEN + sizeof(tstr1xAuthCredentials) + sizeof(tstrM2mWifiWepParams)) % 4))
	uint8	__PAD__[__PADDING_tuniM2MWifiAuth_];
}tuniM2MWifiAuth;


/*!
@struct	\
	tstrM2MWifiSecInfo

@brief
	This struct stores the authentication credentials to connect to a Wi-Fi network.
*/
typedef struct{
	tuniM2MWifiAuth		uniAuth;
	/*!< Union holding all possible authentication parameters corresponding the current security types.	*/
	uint8				u8SecType;
	/*!< Wi-Fi network security type. See tenuM2mSecType for supported security types. */
#define __PADDING__		(4 - ((sizeof(tuniM2MWifiAuth) + 1) % 4))
	uint8				__PAD__[__PADDING__];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2MWifiSecInfo;


/*!
@struct	\
	tstrM2mWifiConnect

@brief	
	This struct stores the Wi-Fi connect request.
*/
typedef struct{
	tstrM2MWifiSecInfo	strSec;
	/*!< Security parameters for authenticating with the AP. */
	uint16				u16Ch;
	/*!< RF Channel for the target SSID. */
	uint8				au8SSID[M2M_MAX_SSID_LEN];
	/*!< SSID of the desired AP. It must be NULL terminated string.	*/
	uint8 				u8NoSaveCred;
#define __CONN_PAD_SIZE__		(4 - ((sizeof(tstrM2MWifiSecInfo) + M2M_MAX_SSID_LEN + 3) % 4))
	uint8				__PAD__[__CONN_PAD_SIZE__];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mWifiConnect;


/*!
@struct	\
	tstrM2MWPSConnect

@brief	
	This struct stores the WPS configuration parameters.

@sa 
	tenuWPSTrigger
*/
typedef struct {
	uint8 	u8TriggerType;
	/*!< WPS triggering method (Push button or PIN) */
	char         acPinNumber[8];
	/*!< WPS PIN No (for PIN method) */
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2MWPSConnect;


/*!
@struct	\
	tstrM2MWPSInfo

@brief	WPS Result

	This struct is passed to the application in response to a WPS request. If the WPS session is completed successfully, the
	structure will have Non-ZERO authentication type. If the WPS Session fails (due to error or timeout) the authentication type
	is set to ZERO.

@sa
	tenuM2mSecType
*/
typedef struct{
	uint8	u8AuthType;
	/*!< Network authentication type. */
	uint8   	u8Ch;
	/*!< RF Channel for the AP.	*/
	uint8	au8SSID[M2M_MAX_SSID_LEN];
	/*!< SSID obtained from WPS. */
	uint8	au8PSK[M2M_MAX_PSK_LEN];
	/*!< PSK for the network obtained from WPS.	*/

	#define __PADDING_tstrM2MWPSInfo_ (4 - ((2 + M2M_MAX_SSID_LEN + M2M_MAX_PSK_LEN) % 4))
	uint8	__PAD__[__PADDING_tstrM2MWPSInfo_];
}tstrM2MWPSInfo;

/*!
@struct	\
	tstrM2MBLEInfo

@brief	BLE Result

	This struct is passed to the application in response to a BLE request. If the BLE session is completed successfully, the
	structure will have Non-ZERO authentication type. If the BLE Session fails (due to error or timeout) the authentication type
	is set to ZERO.

@sa
	tenuM2mSecType
*/
typedef struct{
	uint8	u8AuthType;
	/*!< Network authentication type.*/
	uint8   	u8Ch;
	/*!< RF Channel for the AP. */
	uint8	au8SSID[M2M_MAX_SSID_LEN];
	/*!< SSID obtained from BLE provisioning. */
	uint8	au8PSK[M2M_MAX_PSK_LEN];
	/*!< PSK for the network obtained from BLE provisioning. */

	#define __PADDING_tstrM2MBLEInfo_ (4 - ((2 + M2M_MAX_SSID_LEN + M2M_MAX_PSK_LEN) % 4))
	uint8	__PAD__[__PADDING_tstrM2MBLEInfo_];
}tstrM2MBLEInfo;

/*!
@struct	\
	tstrM2MDefaultConnResp

@brief
	This struct contains the response error of m2m_default_connect.

@sa
	M2M_DEFAULT_CONN_SCAN_MISMATCH
	M2M_DEFAULT_CONN_EMPTY_LIST
*/
typedef struct{
	sint8		s8ErrorCode;
	/*!<
		Default connect error code. possible values are:
		- M2M_DEFAULT_CONN_EMPTY_LIST
		- M2M_DEFAULT_CONN_SCAN_MISMATCH
	*/
	uint8	__PAD24__[3];
}tstrM2MDefaultConnResp;

/*!
@struct	\
	tstrM2MScanOption
	
@brief	
	This struct contains the configuration options for Wi-Fi scan.

*/
typedef struct {
	uint8   u8NumOfSlot;
	/*!< The number of scan slots per channel. Refers to both active and passive scan.
		 Valid settings are in the range 0<Slots<=255.
		 Default setting is @ref M2M_SCAN_DEFAULT_NUM_SLOTS.
	*/
	uint8   u8SlotTime;
	/*!< The length of each scan slot in milliseconds. Refers to active scan only.
		 The device listens for probe responses and beacons during this time.
		 Valid settings are in the range 10<=SlotTime<=250.
		 Default setting is @ref M2M_SCAN_DEFAULT_SLOT_TIME.
	*/
	uint8   u8ProbesPerSlot;
	/*!< Number of probe requests to be sent each scan slot. Refers to active scan only.
		 Valid settings are in the range 0<Probes<=2.
		 Default setting is @ref M2M_SCAN_DEFAULT_NUM_PROBE.
	*/
	sint8   s8RssiThresh;
	/*!< The Received Signal Strength Indicator threshold required for (fast) reconnection to an AP without scanning all channels first.
		 Refers to active scan as part of reconnection to a previously connected AP.
		 The device connects to the target AP immediately if it receives a sufficiently strong probe response on the expected channel.
		 Low thresholds facilitate fast reconnection. High thresholds facilitate connection to the strongest signal.
		 Valid settings are in the range -128<=Thresh<0.
		 Default setting is @ref M2M_FASTCONNECT_DEFAULT_RSSI_THRESH.
	*/
	uint16  u16PassiveScanTime;
	/*!< The length of each scan slot in milliseconds. Refers to passive scan only.
		 The device listens for beacons during this time.
		 Valid settings are in the range 10<=PassiveScanTime<=1200.
		 Default setting is @ref M2M_SCAN_DEFAULT_PASSIVE_SLOT_TIME.
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment */
}tstrM2MScanOption;

/*!
@struct	\
	tstrM2MScanRegion
	
@brief	
	This struct contains the Wi-Fi information for the channel regions.

@sa 
	tenuM2mScanRegion
*/
typedef struct {
	uint16   u16ScanRegion;
	/*|< Specifies the number of channels allowed in the region (e.g. North America = 11 ... etc.).
	*/
	uint8 __PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment */
}tstrM2MScanRegion;

/*!
@struct	\
	tstrM2MScan
	
@brief	
	This struct contains the Wi-Fi scan request.

@sa 
	tenuM2mScanCh
*/
typedef struct {
	uint8 	u8ChNum;
	/*!< The Wi-Fi RF channel number */
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment */
}tstrM2MScan;


/*!
@struct	\	
	tstrM2mScanDone

@brief	
	This struct contains the Wi-Fi scan result.
*/
typedef struct{
	uint8 	u8NumofCh;
	/*!< Number of found APs */
	sint8 	s8ScanState;
	/*!< Scan status */
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mScanDone;


/*!
@struct	\
	tstrM2mReqScanResult

@brief
	The Wi-Fi Scan results list is stored in firmware. This struct contains the index by which the application can request a certain scan result.
*/
typedef struct {
	uint8 	u8Index;
	/*!< Index of the desired scan result */
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment */
}tstrM2mReqScanResult;


/*!
@struct	\
	tstrM2mWifiscanResult

@brief
	This struct contains the information corresponding to an AP in the scan result list identified by its order (index) in the list.
*/
typedef struct {
	uint8 	u8index; 
	/*!< AP index in the scan result list.	*/
	sint8 	s8rssi; 
	/*!< AP signal strength. */
	uint8 	u8AuthType; 
	/*!< AP authentication type. */
	uint8 	u8ch; 
	/*!< AP RF channel. */
	uint8	au8BSSID[6];
	/*!< BSSID of the AP. */
	uint8 	au8SSID[M2M_MAX_SSID_LEN];
	/*!< AP SSID. */
	uint8 	_PAD8_;
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mWifiscanResult;


/*!
@struct	\		
	tstrM2mWifiStateChanged

@brief		
	This struct contains the Wi-Fi connection state

@sa			
	M2M_WIFI_DISCONNECTED, M2M_WIFI_CONNECTED, M2M_WIFI_REQ_CON_STATE_CHANGED
*/
typedef struct {
	uint8	u8CurrState;
	/*!< Current Wi-Fi connection state */
	uint8  u8ErrCode;
	/*!< Error type */
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment */
}tstrM2mWifiStateChanged;


/*!
@struct	\
	tstrM2mPsType

@brief		
	This struct contains the Power Save configuration.

@sa
	tenuPowerSaveModes
*/
typedef struct{
	uint8 	u8PsType;
	/*!< Power save operating mode */
	uint8 	u8BcastEn;
	/*!< Broadcast Enable/Disable */
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment */
}tstrM2mPsType;

/*!
@struct	\
	tstrM2mSlpReqTime

@brief		
	This struct contains the sleep time for the Power Save request.

*/
typedef struct {
	uint32 u32SleepTime;
	/*!< Sleep time in ms */
} tstrM2mSlpReqTime;

/*!
@struct	\
	tstrM2mLsnInt

@brief
	This struct contains the Listen Interval. It is the value of the Wi-Fi StA Listen Interval when power save is enabled. It is given in units of Beacon period.
	It is the number of Beacon periods the WINC can sleep before it wakes up to receive data buffered for it in the AP.
*/
typedef struct {
	uint16 	u16LsnInt;
	/*!< Listen interval in Beacon period count. */
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mLsnInt;


/*!
@struct	\
	tstrM2MWifiMonitorModeCtrl

@brief
	This struct contains the Wi-Fi Monitor Mode Filter. It sets the filtering criteria for WLAN packets when monitoring mode is enabled.
	The received packets matching the filtering parameters, are passed directly to the application.
*/
typedef struct{
	uint8	u8ChannelID;
	/*!< RF Channel ID. It must use values from tenuM2mScanCh 	*/
	uint8	u8FrameType;
	/*!< It must use values from tenuWifiFrameType. */
	uint8	u8FrameSubtype;
	/*!< It must use values from tenuSubTypes. */
	uint8	au8SrcMacAddress[6];
	/*!< ZERO means DO NOT FILTER Source address. */
	uint8	au8DstMacAddress[6];
	/*!< ZERO means DO NOT FILTER Destination address. */
	uint8	au8BSSID[6];
	/*!< ZERO means DO NOT FILTER BSSID. */
	uint8 u8EnRecvHdr;
	/*!< Enable recv the full header before the payload */
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
}tstrM2MWifiMonitorModeCtrl;


/*!
@struct	\
	tstrM2MWifiRxPacketInfo

@brief
	This struct contains the Wi-Fi RX Frame Header. The M2M application has the ability to allow Wi-Fi monitoring mode for receiving all Wi-Fi Raw frames matching a well defined filtering criteria.
	When a target Wi-Fi packet is received, the header information are extracted and assigned in this structure. 
*/
typedef struct{
	uint8	u8FrameType;
	/*!< It must use values from tenuWifiFrameType. */
	uint8	u8FrameSubtype;
	/*!< It must use values from tenuSubTypes. */
	uint8	u8ServiceClass;
	/*!< Service class from Wi-Fi header. */
	uint8	u8Priority;
	/*!< Priority from Wi-Fi header. */
	uint8	u8HeaderLength;
	/*!< Frame Header length. */
	uint8	u8CipherType;
	/*!< Encryption type for the rx packet. */
	uint8	au8SrcMacAddress[6];
	/*!< ZERO means DO NOT FILTER Source address. */
	uint8	au8DstMacAddress[6];
	/*!< ZERO means DO NOT FILTER Destination address. */
	uint8	au8BSSID[6];
	/*!< ZERO means DO NOT FILTER BSSID. */
	uint16	u16DataLength;
	/*!< Data payload length (Header excluded).	*/
	uint16	u16FrameLength;
	/*!< Total frame length (Header + Data). */
	uint32	u32DataRateKbps;
	/*!< Data Rate in Kbps.	*/
	sint8		s8RSSI;
	/*!< RSSI.	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2MWifiRxPacketInfo;


/*!
@struct	\
	tstrM2MWifiTxPacketInfo

@brief
	This struct contains the Wi-Fi TX Packet Info. The M2M Application has the ability to compose raw Wi-Fi frames (under the application responsibility).
	When transmitting a Wi-Fi packet, the application must supply the firmware with this structure for sending the target frame.
*/
typedef struct{
	uint16	u16PacketSize;
	/*!< Wlan frame length.	*/
	uint16	u16HeaderLength;
	/*!< Wlan frame header length. */
}tstrM2MWifiTxPacketInfo;


/*!
 @struct	\	
 	tstrM2MP2PConnect

 @brief		
 	This struct contains the Listen Channel for P2P connect.
*/
typedef struct {
	uint8 	u8ListenChannel; 
	/*!< P2P Listen Channel (1, 6 or 11) */
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2MP2PConnect;

/*!
@struct	\
	tstrM2MAPConfig

@brief
	This structure holds the configuration parameters for the AP mode. It should be set by the application when
	it requests to enable the AP operation mode. The AP mode currently supports only OPEN and WEP security.
*/
typedef struct {
	uint8 	au8SSID[M2M_MAX_SSID_LEN]; 
	/*!< AP SSID */
	uint8 	u8ListenChannel; 
	/*!< Wi-Fi RF Channel which the AP will operate on */
	uint8	u8KeyIndx; 
	/*!< WEP key index */
	uint8	u8KeySz; 
	/*!< WEP key size */
	uint8	au8WepKey[WEP_104_KEY_STRING_SIZE + 1]; 
	/*!< WEP key */
	uint8 	u8SecType; 
	/*!< Security type: OPEN or WEP */
	uint8 	u8SsidHide;
	/*!< SSID Status "Hidden(1)/Visible(0)"	*/
	uint8	au8DHCPServerIP[4];
	/*!< AP DHCP server address	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing alignment */
}tstrM2MAPConfig;

/*!
@struct	\	
	tstrM2mServerInit

@brief	
	This struct contains the information for the PS Server initialization.
*/
typedef struct {
	uint8 	u8Channel;
	/*!< Server Listen channel */
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mServerInit;


/*!
@struct	\	
	tstrM2mClientState

@brief	
	This struct contains the information for the PS Client state.
*/
typedef struct {
	uint8 	u8State;
	/*!< PS Client State */
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mClientState;


/*!
@struct	\	
	tstrM2Mservercmd

@brief
	This struct contains the information for the PS Server command.
*/
typedef struct {
	uint8	u8cmd; 
	/*!< PS Server Cmd	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2Mservercmd;


/*!
@struct	\
	tstrM2mSetMacAddress
	
@brief		
	This struct contains the MAC address to be used. The WINC loads the mac address from the efuse by default to the WINC configuration memory,
	however, the application can overwrite the configuration memory with the mac address indicated from the Host.

@note		
	It's recommended to call this only once before calling connect request and after the m2m_wifi_init
*/
typedef struct {
	uint8 	au8Mac[6]; 
	/*!< MAC address */
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2mSetMacAddress;


/*!
@struct	\
 	tstrM2MDeviceNameConfig
 	
@brief
	This struct contains the Device Name of the WINC. It is used mainly for Wi-Fi Direct device discovery and WPS device information.
*/
typedef struct {
	uint8 	au8DeviceName[M2M_DEVICE_NAME_MAX];
	/*!< NULL terminated device name */
}tstrM2MDeviceNameConfig;


/*!
@struct	\	
 	tstrM2MIPConfig
 	
@brief		
 	This struct contains the static IP configuration.

@note
 	All member IP addresses are expressed in Network Byte Order (eg. "192.168.10.1" will be expressed as 0x010AA8C0).
 */
typedef struct {
	uint32 	u32StaticIP;
	/*!< The static IP assigned to the device. */
	uint32 	u32Gateway;
	/*!< IP of the default internet gateway. */
	uint32 	u32DNS;
	/*!< IP for the DNS server.	*/
	uint32 	u32SubnetMask;
	/*!< Subnet mask for the local area network. */
} tstrM2MIPConfig;

/*!
@struct	\	
 	tstrM2mIpRsvdPkt
 	
@brief		
 	This struct contains the size and data offset for the received packet.

 */
typedef struct{
	uint16	u16PktSz;
	/*<! Packet Size */
	uint16	u16PktOffset;
	/*<! Packet offset */
} tstrM2mIpRsvdPkt;


/*!
@struct	\	
 	tstrM2MProvisionModeConfig
 	
@brief		
 	This struct contains the provisioning mode configuration.
 */

typedef struct {
	tstrM2MAPConfig	strApConfig;
	/*!< Configuration parameters for the WiFi AP.	*/
	char			acHttpServerDomainName[64];
	/*!< The device domain name for HTTP provisioning.*/
	uint8			u8EnableRedirect;
	/*!<
		A flag to enable/disable HTTP redirect feature for the HTTP provisioning server. If the redirect is enabled,
		all HTTP traffic (http://URL) from the device associated with WINC AP will be redirected to the HTTP Provisioning web page.
		- 0 : Disable HTTP Redirect.
		- 1 : Enable HTTP Redirect. 
	*/
	uint8			__PAD24__[3];
}tstrM2MProvisionModeConfig;


/*!
@struct	\	
 	tstrM2MProvisionInfo
 	
@brief		
 	This struct contains the provisioning information obtained from the HTTP Provisioning server.
 */
typedef struct{
	uint8	au8SSID[M2M_MAX_SSID_LEN];
	/*!< Provisioned SSID. */
	uint8	au8Password[M2M_MAX_PSK_LEN];
	/*!< Provisioned Password. */
	uint8	u8SecType;
	/*!< Wifi Security type. */
	uint8	u8Status;
	/*!<
		Provisioning status. To be checked before reading the provisioning information. It may be
		- M2M_SUCCESS 	: Provision successful.
		- M2M_FAIL		: Provision Failed.
	*/
}tstrM2MProvisionInfo;


/*!
@struct	\	
 	tstrM2MConnInfo
 	
@brief		
 	This struct contains the connection information.
 */
typedef struct{
	char		acSSID[M2M_MAX_SSID_LEN];
	/*!< AP connection SSID name  */
	uint8	u8SecType;
	/*!< Security type */
	uint8	au8IPAddr[4];
	/*!< Connection IP address */
	uint8	au8MACAddress[6];
	/*!< MAC address of the peer Wi-Fi station */ 
	sint8	s8RSSI;
	/*!< Connection RSSI signal */
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment */
}tstrM2MConnInfo;

/*!
@struct	\
 	tstrM2mBleApiMsg

@brief
 	This struct contains a BLE message.
 */
typedef struct {
	uint16   u16Len;
	/*!< Length of the message */
	uint8    data[];		//lint !e43
	/*!< Payload of the message */
}tstrM2mBleApiMsg;

/*!
@struct	\
	tstrSystemTime

@brief
	This struct contains the system time.
*/
typedef struct{
	uint16	u16Year;
	/*!< Year */
	uint8	u8Month;
	/*!< Month */
	uint8	u8Day;
	/*!< Day */
	uint8	u8Hour;
	/*!< Hour */
	uint8	u8Minute;
	/*!< Minutes */
	uint8	u8Second;
	/*!< Seconds */
	
	#define __PADDING_tstrSystemTime_ (4 - (7 % 4))
	/*!< Padding for @ref tstrSystemTime structure. */
	uint8	__PAD__[__PADDING_tstrSystemTime_];
	/*!< Structure padding. */
}tstrSystemTime;

/*!
@struct	\
 	tstrM2MMulticastMac

@brief
 	This struct contains the information from the Multicast filter.
 */
 typedef struct {
	uint8 au8macaddress[M2M_MAC_ADDRES_LEN];
	/*!< Mac address needed to be added or removed from filter.	*/
	uint8 u8AddRemove;
	/*!< Set by 1 to add or 0 to remove from filter. */
	uint8	__PAD8__;
	/*!< Padding bytes for forcing 4-byte alignment	*/
}tstrM2MMulticastMac;

/*!
@struct	\
 	tstrM2MGainTable

@brief
 	This struct contains the information of the gain table index from flash to be used.
 */
typedef struct {
   uint8 u8GainTable;
   /*!< Gain Table offset.	*/
   uint8 __PAD24___[3];
   /*!< Padding bytes for forcing 4-byte alignment */
}tstrM2MGainTable;

/*!
@struct	\
 	tstrM2MGainTableRsp

@brief
 	This struct contains response when the firmware has failed to configure the gains from flash.
 */
typedef struct {
  sint8 s8ErrorCode;
  /*!< Error Code.	*/
  uint8 __PAD24__[3];
  /*!< Padding bytes for forcing 4-byte alignment */
}tstrM2MGainTableRsp;


/*!
@struct	\
	tstrRootCertRsaKeyInfo

@brief
	Info about a RSA public key.
*/
typedef struct{
	uint16	u16NSz;
	/*!< Modulus length. */
	uint16	u16ESz;
	/*!< Public exponent length. */
}tstrRootCertRsaKeyInfo;
/*!
@struct	\
	tstrRootCertEcdsaKeyInfo

@brief
	Info about a ECDSA public key.
*/
typedef struct{
	uint16	u16CurveID;
	/*!< Elliptic curve ID. */
	uint16	u16KeySz;
	/*!< Elliptic curve coordinate length. */
}tstrRootCertEcdsaKeyInfo;

/*!
@struct	\
	tstrRootCertPubKeyInfo

@brief
	Info about the public key contained in a root certificate.
*/
typedef struct{
	uint32	u32PubKeyType;
	/*!< Public key type. @see tenuRootCertPubKeyType. */
	union{
		tstrRootCertRsaKeyInfo		strRsaKeyInfo;
		/*!< Info about an RSA public key. */
		tstrRootCertEcdsaKeyInfo	strEcsdaKeyInfo;
		/*!< Info about an ECDSA public key. */
	};
	/*!< Union of RSA / ECDSA public key info structures. */
}tstrRootCertPubKeyInfo;

/*!
@struct	\
	tstrRootCertEntryHeader

@brief
	Header of a root certificate entry in flash.
*/
typedef struct{
	uint8					au8SHA1NameHash[20];
	/*!< SHA1 digest of root certificate issuer name. Used as entry identifier. */
	tstrSystemTime			strExpDate;
	/*!< Expiry date of root certificate. */
	tstrRootCertPubKeyInfo	strPubKey;
	/*!< Info about root certificate public key. */
}tstrRootCertEntryHeader;

/*!
@struct	\
	tstrRootCertFlashHeader
	
@brief
	Header of the root certificate flash storage area.
*/
typedef struct{
	uint8	au8StartPattern[16];
	/*!< Start pattern of root certificate flash store. */
	uint32	u32nCerts;
	/*!< Number of entries in root certificate flash store. */
}tstrRootCertFlashHeader;


/*!
@struct	\
 	tstrTlsSrvSecFileEntry

@brief
 	This struct contains a TLS certificate.
 */
typedef struct{
	char	acFileName[TLS_FILE_NAME_MAX];
	/*!< Name of the certificate.	*/
	uint32	u32FileSize;
	/*!< Size of the certificate.	*/
	uint32	u32FileAddr;
	/*!< Error Code.	*/
}tstrTlsSrvSecFileEntry;

/*!
@struct	\
 	tstrTlsSrvSecHdr

@brief
 	This struct contains a set of TLS certificates.
 */
typedef struct{
	uint8					au8SecStartPattern[TLS_SRV_SEC_START_PATTERN_LEN];
	/*!< Start pattern.	*/	
	uint32					u32nEntries;
	/*!< Number of certificates stored in the struct.	*/
	uint32					u32NextWriteAddr;
	/*	*/
	tstrTlsSrvSecFileEntry	astrEntries[TLS_SRV_SEC_MAX_FILES];
	/*!< TLS Certificate headers.	*/
	uint32					u32CRC;
	/*!< CRC32 of entire cert block, only the cert writer computes this, the FW just does a compare with replacement blocks.	*/
}tstrTlsSrvSecHdr;

typedef enum {
	TLS_FLASH_OK,
	/*!< Operation succeeded. Flash modified. */
	TLS_FLASH_OK_NO_CHANGE,
	/*!< Operation was unnecessary. Flash not modified. */
	TLS_FLASH_ERR_CORRUPT,
	/*!< Operation failed. Flash modified. */
	TLS_FLASH_ERR_NO_CHANGE,
	/*!< Operation failed. Flash not modified. */
	TLS_FLASH_ERR_UNKNOWN
	/*!< Operation failed. Flash status unknown. */
}tenuTlsFlashStatus;

#define TLS_CERTS_CHUNKED_SIG_VALUE 0x6ec8
typedef struct{
	uint16	u16Sig;
	uint16	u16TotalSize32;
	uint16	u16Offset32;
	uint16	u16Size32;
}tstrTlsSrvChunkHdr;

typedef struct{
	uint32	u32CsBMP;
}tstrSslSetActiveCsList;

/**@defgroup  OTATYPEDEF  Enumeration/Typedefs
 * @ingroup OTAAPI
 * @{ */

/*!
@enum	\
	tenuM2mOtaCmd
	
@brief
	This enum contains all the WINC commands used for OTA operation.
*/
typedef enum {
	M2M_OTA_REQ_NOTIF_SET_URL = M2M_OTA_CMD_BASE,
	/*!< Reserved. Do not use.*/
	M2M_OTA_REQ_NOTIF_CHECK_FOR_UPDATE,
	/*!< Reserved. Do not use.*/
	M2M_OTA_REQ_NOTIF_SCHED,
	/*!< Reserved. Do not use.*/
	M2M_OTA_REQ_START_UPDATE,
	/*!< Request to start an OTA update.*/
	M2M_OTA_REQ_SWITCH_FIRMWARE,
	/*!< Request to switch firmware.*/
	M2M_OTA_REQ_ROLLBACK,
	/*!< Request to perform an OTA rollback.*/
	M2M_OTA_REQ_ABORT,
	/*!< Request to abort OTA.*/
	M2M_OTA_RESP_NOTIF_UPDATE_INFO,
	/*!< Reserved. Do not use.*/
	M2M_OTA_RESP_UPDATE_STATUS,
	/*!< Response to indicate the OTA update status. */
	M2M_OTA_REQ_TEST,
	/*!< Reserved. Do not use.*/
	M2M_OTA_MAX_ALL,
}tenuM2mOtaCmd;

/*!
@enum	\
	tenuOtaUpdateStatus

@brief
	This struct contains the OTA return status.
*/
typedef enum {
	OTA_STATUS_SUCSESS            = 0,   
	/*!< OTA Success with no errors. */
	OTA_STATUS_FAIL               = 1,  
	/*!< OTA generic fail. */
	OTA_STATUS_INVAILD_ARG        = 2,
	/*!< Invalid or malformed download URL. */
	OTA_STATUS_INVAILD_RB_IMAGE   = 3,
	/*!< Invalid rollback image. */
	OTA_STATUS_INVAILD_FLASH_SIZE = 4,
	/*!< Flash size on device is not enough for OTA. */
	OTA_STATUS_AlREADY_ENABLED    = 5,
	/*!< An OTA operation is already enabled. */
	OTA_STATUS_UPDATE_INPROGRESS  = 6,
	/*!< An OTA operation update is in progress */
	OTA_STATUS_IMAGE_VERIF_FAILED = 7,
	/*!< OTA Verfication failed */
	OTA_STATUS_CONNECTION_ERROR   = 8,
	/*!< OTA connection error */
	OTA_STATUS_SERVER_ERROR       = 9,
	/*!< OTA server Error (file not found or else ...) */
	OTA_STATUS_ABORTED            = 10,
	/*!< OTA download has been aborted by the application. */
} tenuOtaUpdateStatus;
/*!
@enum	\
	tenuOtaUpdateStatusType

@brief
	This struct contains the OTA update status type.
*/
typedef enum { 

	DL_STATUS        = 1,   
	/*!< Download OTA file status */
	SW_STATUS        = 2,   
	/*!< Switching to the upgrade firmware status */
	RB_STATUS        = 3,   
	/*!< Roll-back status */
	AB_STATUS        = 4,
	/*!< Abort status */
}tenuOtaUpdateStatusType;
/**@}*/


/*!
@struct	\	
 	tstrOtaInitHdr
 	
@brief		
 	This struct contains the OTA image header.
 */

typedef struct{
	uint32 u32OtaMagicValue;
	/*!< Magic value kept in the OTA image after the 
	sha256 Digest buffer to define the Start of OTA Header. */
	uint32 u32OtaPayloadSzie;
	/*!< The Total OTA image payload size, include the sha256 key size. */
}tstrOtaInitHdr;

/*!
@struct	\	
 	tstrOtaControlSec
 	
@brief		
 	This struct contains the Control Section. The Control Section is used to define the working image and
	the validity of the roll-back image and its offset, also both firmware versions is kept in that structure.
 */

typedef struct {
	uint32 u32OtaMagicValue;
	/*!< Magic value used to ensure the structure is valid or not. */
	uint32 u32OtaFormatVersion;
	/*!< Control structure format version, the value will be incremented in case of structure changed or updated. */
	uint32 u32OtaSequenceNumber;
	/*!< Sequence number is used while update the control structure to keep track of how many times that section updated. */
	uint32 u32OtaLastCheckTime;
	/*!< Last time OTA check for update. */
	uint32 u32OtaCurrentworkingImagOffset;
	/*!< Current working offset in flash. */
	uint32 u32OtaCurrentworkingImagFirmwareVer;
	/*!< Current working image firmware version [Major/Product ID/Minor/Patch] */
	uint32 u32OtaCurrentworkingImagHifVer;
	/*!< Current working image HIF version */
	uint32 u32OtaRollbackImageOffset;
	/*!< Roll-back image offset in flash. */
	uint32 u32OtaRollbackImageValidStatus;
	/*!< Roll-back image valid status. */
	uint32 u32OtaRollbackImagFirmwareVer;
	/*!< Roll-back image firmware version [Major/Product ID/Minor/Patch]  */
	uint32 u32OtaRollbackImagHifVer;
	/*!< Roll-back working image HIF version */
	uint32 u32OtaCortusAppWorkingOffset;
	/*!< Cortus app working offset in flash. */
	uint32 u32OtaCortusAppWorkingValidSts;
	/*!< Working Cortus app valid status. */
	uint32 u32OtaCortusAppWorkingVer;
	/*!< Working cortus app version (ex 1.0.1)*/
	uint32 u32OtaCortusAppRollbackOffset;
	/*!< Ccortus app rollback offset in flash. */
	uint32 u32OtaCortusAppRollbackValidSts;
	/*!< Roll-back cortus app valid status */
	uint32 u32OtaCortusAppRollbackVer;
	/*!< Roll-back cortus app version (ex 18.0.1) */
	uint32 u32OtaControlSecCrc;
	/*!< CRC for the control structure to ensure validity */
} tstrOtaControlSec;

/*!
@struct	\
	tstrOtaUpdateStatusResp

@brief	
	This struct contains the OTA update status

@sa 
	tenuWPSTrigger
*/
typedef struct {
	uint8	u8OtaUpdateStatusType;
	/*!< 
		Status type tenuOtaUpdateStatusType
	*/
	uint8	u8OtaUpdateStatus;
	/*!< 
	OTA_SUCCESS 						
	OTA_ERR_WORKING_IMAGE_LOAD_FAIL		
	OTA_ERR_INVAILD_CONTROL_SEC			
	M2M_ERR_OTA_SWITCH_FAIL     		
	M2M_ERR_OTA_START_UPDATE_FAIL     	
	M2M_ERR_OTA_ROLLBACK_FAIL     		
	M2M_ERR_OTA_INVAILD_FLASH_SIZE     	
	M2M_ERR_OTA_INVAILD_ARG		     
	*/
	uint8 _PAD16_[2];
}tstrOtaUpdateStatusResp;

/*!
@struct	\
	tstrOtaUpdateInfo

@brief	
	This struct contains the OTA update information

@sa 
	tenuWPSTrigger
*/
typedef struct {
	uint32	u8NcfUpgradeVersion;
	/*!< NCF OTA Upgrade Version */
	uint32	u8NcfCurrentVersion;
	/*!< NCF OTA Current firmware version */
	uint32	u8NcdUpgradeVersion;
	/*!< NCD (host) upgraded version (if the u8NcdRequiredUpgrade == true)	*/
	uint8	u8NcdRequiredUpgrade;
	/*!< NCD Required upgrade to the above version */
	uint8 	u8DownloadUrlOffset;
	/*!< Download URL offset in the received packet */
	uint8 	u8DownloadUrlSize;
	/*!< Download URL size in the received packet */
	uint8	__PAD8__;
	/*!< Padding bytes for forcing 4-byte alignment */
} tstrOtaUpdateInfo;

/*!
@struct	\
 	tstrPrng

@brief
 	M2M Request PRNG
 */
typedef struct {
	 /*!< return buffer address	*/
	uint8 *pu8RngBuff;
	 /*!< PRNG size requested */
	uint16 	u16PrngSize;
	/*!< PRNG pads */
	uint8 __PAD16__[2];
}tstrPrng;
#endif
