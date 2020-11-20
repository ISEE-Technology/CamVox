/**
 *
 * \file
 *
 * \brief WINC3400 IoT Application Interface.
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

/**@defgroup m2m_wifi WLAN
*/

#ifndef __M2M_WIFI_H__
#define __M2M_WIFI_H__

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#include "common/include/nm_common.h"
#include "driver/include/m2m_types.h"
#include "driver/source/nmdrv.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/**@defgroup  WlanEnums Enumeration/Typedefs
 * @ingroup m2m_wifi
 * @{*/
 
/*!
@enum	\
    tenuWifiFrameType
@brief
    Enumeration for Wi-Fi MAC frame type codes (2-bit) 
    The following types are used to identify the type of frame sent or received.
    Each frame type constitutes a number of frame subtypes as defined in @ref tenuSubTypes to specify 
    the exact type of frame.
    
    Values are defined as per the IEEE 802.11 standard.
@remarks
    The following frame types are useful for advanced user usage when symbol CONF_MGMT is defined
    and the user application requires to monitor the frame transmission and reception.
@warning
    This functionality is not supported by current WINC firmware.
@see
    tenuSubTypes
*/
typedef enum {
    MANAGEMENT      = 0x00,    /*!< Wi-Fi Management frame (Probe Req/Resp, Beacon, Association Req/Resp ...etc). */
    CONTROL         = 0x04,    /*!< Wi-Fi Control frame (eg. ACK frame). */
    DATA_BASICTYPE  = 0x08,    /*!< Wi-Fi Data frame. */
    RESERVED        = 0x0C
}tenuWifiFrameType;


/*!
@enum	\
    tenuSubTypes

@brief
    Enumeration for Wi-Fi MAC Frame subtype code (6-bit).
    The frame subtypes fall into one of the three frame type groups as defined in @ref tenuWifiFrameType
    (MANAGEMENT, CONTROL & DATA).
    
    Values are defined as per the IEEE 802.11 standard.
@remarks
    The following sub-frame types are useful for advanced user usage when symbol CONF_MGMT is defined
    and the application developer requires to monitor the frame transmission and reception.
@warning
    This functionality is not supported by current WINC firmware.
@see
        tenuWifiFrameType
*/
typedef enum {
    /* Sub-Types related to Management */
    ASSOC_REQ             = 0x00,  /*!< Management: Association Request */
    ASSOC_RSP             = 0x10,  /*!< Management: Association Response */
    REASSOC_REQ           = 0x20,  /*!< Management: Re-Association Request */
    REASSOC_RSP           = 0x30,  /*!< Management: Re-Association Response */
    PROBE_REQ             = 0x40,  /*!< Management: Probe Request */
    PROBE_RSP             = 0x50,  /*!< Management: Probe Response */
    BEACON                = 0x80,  /*!< Management: Beacon */
    ATIM                  = 0x90,  /*!< Management: Announcement Traffic Information Map */
    DISASOC               = 0xA0,  /*!< Management: Disassociation */
    AUTH                  = 0xB0,  /*!< Management: Authentication */
    DEAUTH                = 0xC0,  /*!< Management: Deauthentication */
    ACTION                = 0xD0,  /*!< Management: Action */

    /* Sub-Types related to Control */
    PS_POLL               = 0xA4,  /*!< Control: Power Save Poll*/
    RTS                   = 0xB4,  /*!< Control: Request to Send */
    CTS                   = 0xC4,  /*!< Control: Clear to Send */
    ACK                   = 0xD4,  /*!< Control: Acknowledgement */
    CFEND                 = 0xE4,  /*!< Control: End of Contention Free Period */
    CFEND_ACK             = 0xF4,  /*!< Control: ACK of data from STA + CFEND */
    BLOCKACK_REQ          = 0x84,  /*!< Control: Block Acknoweledgement Request */
    BLOCKACK              = 0x94,  /*!< Control: Block Acknowledgement */

    /* Sub-Types related to Data */
    DATA                  = 0x08,  /*!< Data: A non-QoS data frame */
    DATA_ACK              = 0x18,  /*!< Data: Data + CF-Ack */
    DATA_POLL             = 0x28,  /*!< Data: Data + CF-Poll */
    DATA_POLL_ACK         = 0x38,  /*!< Data: Data + CF-Ack + CF-Poll*/
    NULL_FRAME            = 0x48,  /*!< Data: Null (no data) */
    CFACK                 = 0x58,  /*!< Data: CF-Ack (no data) */
    CFPOLL                = 0x68,  /*!< Data: CF-Poll (no data) */
    CFPOLL_ACK            = 0x78,  /*!< Data: CF-Poll + CF-Ack (no data) */
    QOS_DATA              = 0x88,  /*!< Data: A QoS data frame */
    QOS_DATA_ACK          = 0x98,  /*!< Data: QoS data + CF-Ack */
    QOS_DATA_POLL         = 0xA8,  /*!< Data: QoS data + CF-Poll */
    QOS_DATA_POLL_ACK     = 0xB8,  /*!< Data: QoS data + CF-Ack + CF-Poll */
    QOS_NULL_FRAME        = 0xC8,  /*!< Data: QoS Null (no data) */
    QOS_CFPOLL            = 0xE8,  /*!< Data: Qos CF-Poll (no data) */
    QOS_CFPOLL_ACK        = 0xF8   /*!< Data: QoS CF-Poll + CF-Ack (no data) */
}tenuSubTypes;


/*!
@enum	\
    tenuInfoElementId

@brief
    Enumeration for the Wi-Fi Information Element(IE) IDs, which indicates the specific type of IEs.
    IEs are management frame information included in management frames.

    Values are defined as per the IEEE 802.11 standard.

@warning
    This functionality is not supported by current WINC firmware.
*/
typedef enum {
    ISSID               = 0,       /*!< Service Set Identifier (SSID) */
    ISUPRATES           = 1,       /*!< Supported Rates */
    IFHPARMS            = 2,       /*!< FH parameter set */
    IDSPARMS            = 3,       /*!< DS parameter set */
    ICFPARMS            = 4,       /*!< CF parameter set */
    ITIM                = 5,       /*!< Traffic Information Map */
    IIBPARMS            = 6,       /*!< IBSS parameter set */
    ICOUNTRY            = 7,       /*!< Country element. */
    IEDCAPARAMS         = 12,      /*!< EDCA parameter set */
    ITSPEC              = 13,      /*!< Traffic Specification */
    ITCLAS              = 14,      /*!< Traffic Classification */
    ISCHED              = 15,      /*!< Schedule. */
    ICTEXT              = 16,      /*!< Challenge Text */
    IPOWERCONSTRAINT    = 32,      /*!< Power Constraint. */
    IPOWERCAPABILITY    = 33,      /*!< Power Capability */
    ITPCREQUEST         = 34,      /*!< TPC Request */
    ITPCREPORT          = 35,      /*!< TPC Report */
    ISUPCHANNEL         = 36,      /*!< Supported channel list */
    ICHSWANNOUNC        = 37,      /*!< Channel Switch Announcement */
    IMEASUREMENTREQUEST = 38,      /*!< Measurement request */
    IMEASUREMENTREPORT  = 39,      /*!< Measurement report */
    IQUIET              = 40,      /*!< Quiet element Info */
    IIBSSDFS            = 41,      /*!< IBSS DFS */
    IERPINFO            = 42,      /*!< ERP Information */
    ITSDELAY            = 43,      /*!< TS Delay */
    ITCLASPROCESS       = 44,      /*!< TCLAS Processing */
    IHTCAP              = 45,      /*!< HT Capabilities */
    IQOSCAP             = 46,      /*!< QoS Capability */
    IRSNELEMENT         = 48,      /*!< RSN Information Element */
    IEXSUPRATES         = 50,      /*!< Extended Supported Rates */
    IEXCHSWANNOUNC      = 60,      /*!< Extended Ch Switch Announcement */
    IHTOPERATION        = 61,      /*!< HT Information */
    ISECCHOFF           = 62,      /*!< Secondary Channel Offset */
    I2040COEX           = 72,      /*!< 20/40 Coexistence IE */
    I2040INTOLCHREPORT  = 73,      /*!< 20/40 Intolerant channel report */
    IOBSSSCAN           = 74,      /*!< OBSS Scan parameters */
    IEXTCAP             = 127,     /*!< Extended capability */
    IWMM                = 221,     /*!< WMM parameters */
    IWPAELEMENT         = 221      /*!< WPA Information Element */
}tenuInfoElementId;

/*!
@enum \
    tenuWifiCapability

@brief
    Enumeration for capability Information field bit. 
    The value of the capability information field from the 802.11 management frames received by the 
	wireless LAN interface. 
    Defining the capabilities of the Wi-Fi system. 
    
    Values are defined as per the IEEE 802.11 standard.
@warning
    This functionality is not supported by current WINC firmware.
*/
typedef enum{
    ESS            = 0x01,         /*!< ESS capability */
    IBSS           = 0x02,         /*!< IBSS mode */
    POLLABLE       = 0x04,         /*!< CF Pollable */
    POLLREQ        = 0x08,         /*!< Request to be polled */
    PRIVACY        = 0x10,         /*!< WEP encryption supported */
    SHORTPREAMBLE  = 0x20,         /*!< Short Preamble is supported */
    SHORTSLOT      = 0x400,        /*!< Short Slot is supported */
    PBCC           = 0x40,         /*!< PBCC */
    CHANNELAGILITY = 0x80,         /*!< Channel Agility */
    SPECTRUM_MGMT  = 0x100,        /*!< Spectrum Management */
    DSSS_OFDM      = 0x2000        /*!< DSSS-OFDM */                   
}tenuWifiCapability;


/*!
@typedef \
    tpfAppWifiCb

@brief	
    This is the main callback function for the Wi-Fi driver and is responsible for processing
    any M2M_WIFI events that are received on the Wi-Fi interface.
    These events (notifications) are usually received in response to earlier Wi-Fi requests such 
    as @ref m2m_wifi_request_scan, and @ref m2m_wifi_connect.

    Most Wi-Fi APIs are asynchronous and calling them generates information that is passed back
    via this callback - for instance @ref m2m_wifi_request_scan will return a set of detected
    networks.
    
    Applications must ensure a callback function is registered with the Wi-Fi driver by 
    calling @ref m2m_wifi_init.
@param [in]	u8MsgType
    Type of notification. Possible types are:
       - @ref M2M_WIFI_RESP_CON_STATE_CHANGED
       - @ref M2M_WIFI_RESP_CONN_INFO
       - @ref M2M_WIFI_REQ_DHCP_CONF
       - @ref M2M_WIFI_REQ_WPS
       - @ref M2M_WIFI_RESP_IP_CONFLICT 
       - @ref M2M_WIFI_RESP_SCAN_DONE
       - @ref M2M_WIFI_RESP_SCAN_RESULT
       - @ref M2M_WIFI_RESP_CURRENT_RSSI
       - @ref M2M_WIFI_RESP_CLIENT_INFO
       - @ref M2M_WIFI_RESP_PROVISION_INFO
       - @ref M2M_WIFI_RESP_DEFAULT_CONNECT
       - @ref M2M_WIFI_RESP_ETHERNET_RX_PACKET (If Bypass mode is active)
       - @ref M2M_WIFI_RESP_WIFI_RX_PACKET (If monitoring mode is active)
                
@param [in]	pvMsg
    A pointer to a buffer containing the notification parameters (if any). It should be
    Casted to the data type associated with type of notification.
    
@see
    tstrM2mWifiStateChanged
    tstrM2MWPSInfo
    tstrM2mScanDone
    tstrM2mWifiscanResult
    m2m_wifi_init
*/
typedef void (*tpfAppWifiCb) (uint8 u8MsgType, void * pvMsg);

/*!
@typedef \
    tpfAppEthCb

@brief	
    Ethernet (Bypass mode) notification callback function receiving Bypass mode events as 
    defined in the Wi-Fi responses enumeration @ref tenuM2mStaCmd. 

    If bypass mode is enabled, applications must ensure this callback function is registered 
    with the Wi-Fi driver by calling @ref m2m_wifi_init.

@param [in]	u8MsgType
    Type of notification. Possible types are:
        - @ref M2M_WIFI_RESP_ETHERNET_RX_PACKET

@param [in]	pvMsg
    A pointer to a buffer containing the notification parameters (if any). This must be
    casted to the data type uint8 *.
    
@param [in]	pvControlBuf
    A pointer to control buffer describing the accompanied message. This must be casted to
    the data type @ref tstrM2mIpCtrlBuf.

@see
    m2m_wifi_init

*/
typedef void (*tpfAppEthCb) (uint8 u8MsgType, void * pvMsg,void * pvCtrlBuf);

/*!
@typedef	\
    tpfAppMonCb

@brief	
    Wi-Fi monitoring mode callback function. This function delivers all received wi-Fi packets 
    to the application. Applications requiring to operate in the monitoring shall call the
    function m2m_wifi_enable_monitoring_mode, each frame received will invoke a single call to this
    callback function. Monitoring mode may be disabled by calling @ref m2m_wifi_disable_monitoring_mode.

@param [in]	pstrWifiRxPacket
    Pointer to a structure holding the Wi-Fi packet header parameters.

@param [in]	pu8Payload
    Pointer to the buffer holding the Wi-Fi packet payload information required by the application 
    starting from the defined OFFSET by the application (when calling 
    m2m_wifi_enable_monitoring_mode). Could hold a value of NULL, if the application does not need 
    any data from the payload.

@param [in]	u16PayloadSize
    The size of the payload in bytes.
                
@see
    m2m_wifi_enable_monitoring_mode		
    
@warning
    u16PayloadSize may not exceed the buffer size given through m2m_wifi_enable_monitoring_mode.
*/
typedef void (*tpfAppMonCb) (tstrM2MWifiRxPacketInfo *pstrWifiRxPacket, uint8 * pu8Payload, uint16 u16PayloadSize);

/**@}*/

/*!
@struct 	\
    tstrEthInitParam
    
@brief		
    Structure to hold Ethernet interface parameters. 
    Structure is to be defined and have its attributes set, based on the application's functionality
    before a call is made to initialize the wi-fi operations by calling the
    @ref m2m_wifi_init function. Part of the wi-fi configuration structure @ref tstrWifiInitParam.
    Applications shouldn't need to define this structure, if the bypass mode is not defined.
    
@see
    tpfAppEthCb
    tpfAppWifiCb
    m2m_wifi_init
*/
typedef struct {
    tpfAppWifiCb pfAppWifiCb;      /*!< Callback for wifi notifications. */
    tpfAppEthCb  pfAppEthCb;       /*!< Callback for Ethernet interface. */
    uint8 * au8ethRcvBuf;          /*!< Pointer to Receive Buffer of Ethernet Packet */
    uint16	u16ethRcvBufSize;      /*!< Size of Receive Buffer for Ethernet Packet */
} tstrEthInitParam;

/*!
@struct	\	
 	tstrM2mIpCtrlBuf

@brief		
 	Structure holding the incoming buffer's data size information, indicating the data size of the 
    buffer and the remaining buffer's data size. The data of the buffer which holds the packet sent 
    to the host when in the bypass mode, is placed in the @ref tstrEthInitParam::au8ethRcvBuf attribute.
    This following information is retrieved in the host when an event 
    @ref M2M_WIFI_RESP_ETHERNET_RX_PACKET is received in the Wi-Fi callback function 
    @ref tpfAppWifiCb. 

    The application is expected to use this structure's information to determine if there is still incoming data to be received from the firmware.

 @see
     tpfAppEthCb
     tstrEthInitParam
 
 @warning
     Make sure that bypass mode is defined before using @ref tstrM2mIpCtrlBuf

 */
typedef struct{
    uint16	u16DataSize;          /*!< Size of the received data in bytes. */
    uint16	u16RemainigDataSize;  /*!< Size of the remaining data bytes to be delivered to host. */
} tstrM2mIpCtrlBuf;


/**
@struct		\
    tstrWifiInitParam

@brief		
    Structure, holding the Wi-fi configuration attributes such as the wi-fi callback , monitoring mode callback and Ethernet parameter initialization structure.
    Such configuration parameters are required to be set before calling the wi-fi initialization function @ref m2m_wifi_init.
    @ref pfAppWifiCb attribute must be set to handle the wi-fi callback operations.
    @ref pfAppMonCb attribute, is optional based on whether the application requires the monitoring mode configuration, and can there not
    be set before the initialization.
    @ref strEthInitParam structure, is another optional configuration based on whether the bypass mode is set.
    
*/
typedef struct {
    tpfAppWifiCb pfAppWifiCb;     /*!< Callback for Wi-Fi notifications. */
    tpfAppMonCb  pfAppMonCb;      /*!< Callback for monitoring interface. */
    tstrEthInitParam strEthInitParam ; /*!< Structure to hold Ethernet interface parameters. */
	uint8        GainTableIndex;       /*!< Gain Table index to be used to configure the WiFi and BLE gains. */
	uint8	     __PAD24__[3]; 		   /*!< Padding bytes for forcing 4-byte alignment */
} tstrWifiInitParam;


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/** \defgroup WLANAPI Functions
 *   @ingroup m2m_wifi
 */
#ifdef __cplusplus
     extern "C" {
#endif
 /**@{*/
/*!
@fn	\
    NMI_API void  m2m_wifi_download_mode(void);

@brief
    Synchronous API that prepares the WINC IC to enter firmware or certificate download mode.

@details
    The WINC board is prepared for download, through initializations for the WINC driver including 
	bus initializations and interrupt enabling, it also halts the chip, to allow for the firmware 
	downloads. Firmware can be downloaded through a number of interfaces, UART, I2C and SPI.
    
@return		
    The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
*/
NMI_API sint8  m2m_wifi_download_mode(void);

/*!
@fn	\
    NMI_API sint8  m2m_wifi_init(tstrWifiInitParam * pWifiInitParam);

@brief
    Synchronous API that initialises the WINC driver.
	
@details
    This function initializes the driver by registering the call back function for M2M_WIFI layer 
	(also the call back function for bypass mode/monitoring mode if defined), initializing the host 
	interface layer and the bus interfaces.	Wi-Fi callback registering is essential to allow the 
	handling of the events received, in response to the asynchronous Wi-Fi operations.

    Following are the possible Wi-Fi events that are expected to be received through the callback 
	function (provided by the application):

     - @ref M2M_WIFI_RESP_CON_STATE_CHANGED
     - @ref M2M_WIFI_RESP_CONN_INFO
     - @ref M2M_WIFI_REQ_DHCP_CONF
     - @ref M2M_WIFI_REQ_WPS
     - @ref M2M_WIFI_RESP_IP_CONFLICT
     - @ref M2M_WIFI_RESP_SCAN_DONE
     - @ref M2M_WIFI_RESP_SCAN_RESULT
     - @ref M2M_WIFI_RESP_CURRENT_RSSI
     - @ref M2M_WIFI_RESP_CLIENT_INFO
     - @ref M2M_WIFI_RESP_PROVISION_INFO
     - @ref M2M_WIFI_RESP_DEFAULT_CONNECT
     - @ref M2M_WIFI_RESP_ETHERNET_RX_PACKET (if bypass mode is enabled)
     - @ref M2M_WIFI_RESP_WIFI_RX_PACKET (if monitoring mode is enabled)
 
    Any application using the WINC driver must call this function at the start of its main function.

@param [in]	pWifiInitParam
    This is a pointer to the @ref tstrWifiInitParam structure which contains pointers to the 
	application WIFI layer callback, monitoring mode callback and @ref tstrEthInitParam 
	structure (which contains initialisation settings for bypass mode).
    
@return		
    The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
@pre 
    Prior to this function call, application users must provide a callback function responsible for 
	receiving all the wi-fi events that are received on the M2M_WIFI layer.
    
@warning
    Failure to successfully complete function indicates that the driver couldn't be initialized and 
	a fatal error will prevent the application from proceeding. 
    
@see
    m2m_wifi_deinit
    tenuM2mStaCmd
*/
NMI_API sint8  m2m_wifi_init(tstrWifiInitParam * pWifiInitParam);

/*!
@fn	\
    NMI_API sint8  m2m_wifi_deinit(void * arg);

@brief 
	Synchronous API that de-initialises the WINC driver.
	
@details
    This disables the host interface and frees any resources used by the M2M_WIFI layer. 
	The function must be called in the during the final phases of closing the application to ensure 
	that all resources have been correctly released.
	
@param [in]	arg
        Opaque argument, not used in current implementation. Application should use null.

@return		
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.
*/
NMI_API sint8  m2m_wifi_deinit(void * arg);

/*!
@fn	\
    sint8  m2m_wifi_init_hold(void);

@brief
    First part of m2m_wifi_init, up to the point of initializing spi for flash access.
@see
    m2m_wifi_init
*/
sint8 m2m_wifi_init_hold(void);
/*!
@fn	\
    sint8  m2m_wifi_init_start(tstrWifiInitParam * param);

@brief
    Second part of m2m_wifi_init, continuing from where m2m_wifi_init_hold left off.

@param [in]	param 
	Structure containing configration details

@see
    m2m_wifi_init
*/
sint8 m2m_wifi_init_start(tstrWifiInitParam * param);

/*!
@fn	\
    sint8  m2m_wifi_reinit(tstrWifiInitParam * pWifiInitParam);

@brief
    Deinitialize and reinitialize wifi. Resets the WINC.
    Parameter may be set to NULL, in which case there is no change to the parameters given at
    initialization.
*/
sint8 m2m_wifi_reinit(tstrWifiInitParam * param);
/*!
@fn	\
    sint8  m2m_wifi_reinit_hold(void);

@brief
    First part of m2m_wifi_reinit, up to the point of initializing spi for flash access.
@see
    m2m_wifi_reinit
*/
sint8 m2m_wifi_reinit_hold(void);

/*!
@fn	\
    sint8  m2m_wifi_reinit_start(tstrWifiInitParam * param);

@brief
    Second part of m2m_wifi_reinit, continuing from where m2m_wifi_reinit_hold left off.
@see
    m2m_wifi_reinit
*/
sint8 m2m_wifi_reinit_start(tstrWifiInitParam * param);

/*!
@fn	\
    NMI_API void m2m_wifi_yield(void);

@brief
    Yield from processing more synchronous M2M events

@details 
    This function causes the synchronous M2M event handler function to yield from processing further
    events and return control to the caller.

@return		
    The function returns @ref M2M_SUCCESS for successful interrupt handling and a negative value 
	otherwise.

@pre
    Prior to receiving  Wi-Fi interrupts, the WINC driver should have been successfully initialized 
	by calling the @ref m2m_wifi_init function.
     
@warning
    Failure to successfully complete this function indicates bus errors and hence a fatal error that will prevent the application from proceeding.
*/

NMI_API void m2m_wifi_yield(void);
/*!
@fn	\
    NMI_API sint8 m2m_wifi_handle_events(void * arg);

@brief
    Synchronous M2M event handler function

@details 
    This function is responsible for handling interrupts received from the WINC firmware. 
	Applications should call this function periodically in-order to receive the events that are to 
	be handled by the callback functions implemented by the application.
*/

NMI_API sint8 m2m_wifi_handle_events(void * arg);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_default_connect(void);

@brief
	Asynchronous API that attempts to reconnect to the last-associated access point.
	
@details
    An application calling this function will cause the firmware to attempt to reconnect to the 
	access point with which it had last successfully connected. A failure to connect will result in 
	a response of @ref M2M_WIFI_RESP_DEFAULT_CONNECT indicating a connection error as defined in the 
	structure @ref tstrM2MDefaultConnResp.
	
    Possible errors are:
     - @ref M2M_DEFAULT_CONN_EMPTY_LIST indicating that the connection list is empty, or
     - @ref M2M_DEFAULT_CONN_SCAN_MISMATCH indicating a mismatch for the saved AP name.

@return		
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@pre 
    Prior to connecting, the WINC driver should have been successfully initialized by calling the 
	@ref m2m_wifi_init function.
  
@warning
    - This function maybe called in station mode only.
    - It is important to note that successful completion of a call to m2m_wifi_default_connect() 
	  does not guarantee success of the WIFI connection; a negative return value indicates only 
      locally-detected errors.
    
@see
    m2m_wifi_connect
*/
NMI_API sint8 m2m_wifi_default_connect(void);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_connect(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch);

@brief
    Asynchronous API to request connection to a specified access point
	
@details
    Prior to a successful connection, the application developers must know the SSID, the security 
	type, and the authentication parameters of the target access point; knowledge of the channel number 
	is optional.
	
    The connection status will be indicated to the application via a 
	@ref M2M_WIFI_RESP_CON_STATE_CHANGED event. The status will be one of those defined in
	@ref tenuM2mConnState withe @ref M2M_WIFI_CONNECTED indicating a successful connection.

@param [in]	pcSsid
    A buffer holding the SSID corresponding to the requested AP.
                
@param [in]	u8SsidLen
    Length of the given SSID (not including the NULL termination). A length less than ZERO or 
	greater than the maximum defined SSID @ref M2M_MAX_SSID_LEN will result in a negative error 
    @ref M2M_ERR_FAIL.
                
@param [in]	u8SecType
    Wi-Fi security type security for the network. It can be one of the following types:
        -@ref M2M_WIFI_SEC_OPEN
        -@ref M2M_WIFI_SEC_WEP
        -@ref M2M_WIFI_SEC_WPA_PSK
        -@ref M2M_WIFI_SEC_802_1X
    A value outside these possible values will result in a negative return error @ref M2M_ERR_FAIL.

@param [in]	pvAuthInfo
    Authentication parameters required for completing the connection. It is type is based on the 
	security type. If the authentication parameters are NULL or are greater than the maximum length 
	of the authentication parameters length as defined by @ref M2M_MAX_PSK_LEN a negative error will 
	return @ref M2M_ERR_FAIL(-12) indicating connection failure.

@param [in]	u16Ch
    Wi-Fi channel number as defined in @ref tenuM2mScanCh enumeration. Specification of a channel 
	number greater than @ref M2M_WIFI_CH_14 returns a negative error @ref M2M_ERR_FAIL(-12) unless
    the value is M2M_WIFI_CH_ALL (255). A channel number of M2M_WIFI_CH_ALL indicates that the 
	firmware should scan all channels to find the SSID specified in parameter pcSsid.
	
    Failure to find the connection match will return a negative error 
	@ref M2M_DEFAULT_CONN_SCAN_MISMATCH.

@return	
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.	

@pre
  	Prior to a successful connection request, the wi-fi driver must have been successfully initialized through the call of the @ref m2m_wifi_init function

@see
    tuniM2MWifiAuth
    tstr1xAuthCredentials
    tstrM2mWifiWepParams
    
@warning
    - This function must be called in station mode only.
    - Successful completion of this function does not guarantee success of the WIFI connection, and
      a negative return value indicates only locally-detected errors.
*/
NMI_API sint8 m2m_wifi_connect(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_connect_sc(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch,uint8 u8SaveCred);

@brief
    Asynchronous API to request connection to a specific AP with option to store credentials in
	flash.

@details
    Prior to a successful connection, the application developers must know the SSID, the security 
	type, and the authentication parameters of the target access point; knowledge of the channel number 
	is optional.

    The connection status will be indicated to the application via a 
	@ref M2M_WIFI_RESP_CON_STATE_CHANGED event. The status will be one of those defined in
	@ref tenuM2mConnState withe @ref M2M_WIFI_CONNECTED indicating a successful connection.

    The only difference between this function and @ref m2m_wifi_connect, is the option to save the 
	acess point info ( SSID, password...etc) into flash.

@param [in]	pcSsid
    A buffer holding the SSID corresponding to the requested AP.
                
@param [in]	u8SsidLen
    Length of the given SSID (not including the NULL termination). A length less than ZERO or 
	greater than the maximum defined SSID @ref M2M_MAX_SSID_LEN will result in a negative error 
    @ref M2M_ERR_FAIL.
                
@param [in]	u8SecType
    Wi-Fi security type security for the network. It can be one of the following types:
        -@ref M2M_WIFI_SEC_OPEN
        -@ref M2M_WIFI_SEC_WEP
        -@ref M2M_WIFI_SEC_WPA_PSK
        -@ref M2M_WIFI_SEC_802_1X
    A value outside these possible values will result in a negative return error @ref M2M_ERR_FAIL.

@param [in]	pvAuthInfo
    Authentication parameters required for completing the connection. It is type is based on the 
	security type. If the authentication parameters are NULL or are greater than the maximum length 
	of the authentication parameters length as defined by @ref M2M_MAX_PSK_LEN a negative error will 
	return @ref M2M_ERR_FAIL(-12) indicating connection failure.

@param [in]	u16Ch
    Wi-Fi channel number as defined in @ref tenuM2mScanCh enumeration. Specification of a channel 
	number greater than @ref M2M_WIFI_CH_14 returns a negative error @ref M2M_ERR_FAIL(-12) unless
    the value is M2M_WIFI_CH_ALL (255). A channel number of M2M_WIFI_CH_ALL indicates that the 
	firmware should scan all channels to find the SSID specified in parameter pcSsid.
	
    Failure to find the connection match will return a negative error 
	@ref M2M_DEFAULT_CONN_SCAN_MISMATCH.

@param [in] u8SaveCred
    Option to store the acess point SSID and password into the WINC flash memory or not.

@pre
    Prior to a successful connection request, the wi-fi driver must have been successfully initialized through the call of the @ref m2m_wifi_init function

@see
    tuniM2MWifiAuth
    tstr1xAuthCredentials
    tstrM2mWifiWepParams
    
@warning
    - This function must be called in station mode only.
    - Successful completion of this function does not guarantee success of the WIFI connection, and
      a negative return value indicates only locally-detected errors.
    
@return	The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
and a negative value otherwise.
    
*/
NMI_API sint8 m2m_wifi_connect_sc(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch, uint8 u8SaveCred);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_disconnect(void);

@brief
    Synchronous API to request disconnection from a network.

@details
    This function will cause the WINC IC to disconnect from any currently connected access point.

@return		
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@pre 
    Disconnection must be made to a successfully connected AP. If the WINC is not in the connected state, a call to this function will hold insignificant.

@warning
    This function must be called in station mode only.
    
@see
    m2m_wifi_connect
    m2m_wifi_default_connect
    
*/
NMI_API sint8 m2m_wifi_disconnect(void);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_start_provision_mode(tstrM2MAPConfig *pstrAPConfig, char *pcHttpServerDomainName, uint8 bEnableHttpRedirect);

@brief
    Asynchronous API for control of Wi-Fi provisioning functionality.

@details	
    This function allows the application to start the WINC IC in 'provisioning mode', a special mode
    that triggers the WINC to create a Wi-Fi access point, DHCP server, and HTTP server.

    The HTTP server presents a provisioning page to a connected client which lists the access points
	detected in the vicinity of the WINC, and allows one of these to be selected and any appropriate
	credentials to be entered. This allows a headless system to be provisioned (configured to 
    connect with an access point). 
	
    Provisioning status is returned in a @ref M2M_WIFI_RESP_PROVISION_INFO event.
	
@param [in]	pstrAPConfig
    AP configuration parameters as defined in @ref tstrM2MAPConfig configuration structure.
    A NULL value passed in, will result in a negative error @ref M2M_ERR_FAIL.
                
@param [in]	pcHttpServerDomainName
    Domain name of the HTTP Provision WEB server which others will use to load the provisioning 
	Home page. For example "wincconf.net".

@param [in]	bEnableHttpRedirect
    A flag to enable/disable the HTTP redirect feature. Possible values are:
        - Zero:     DO NOT Use HTTP Redirect. In this case the associated device could open the 
		            provisioning page ONLY when the HTTP Provision URL of the WINC HTTP Server is 
				    correctly written on the browser.
        - Non-Zero: Use HTTP Redirect. In this case, all http traffic (http://URL) from the 
		            associated device (Phone, PC, ...etc) will be redirected to the WINC HTTP 
					Provisioning Home page.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@pre	
    - A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at 
	  startup. Registering the callback is done through passing it to the initialization 
	  @ref m2m_wifi_init function.
    - The event @ref M2M_WIFI_RESP_CONN_INFO must be handled in the callback to receive the 
	  requested connection info.
    
@see
    tpfAppWifiCb
    m2m_wifi_init
    M2M_WIFI_RESP_PROVISION_INFO
    m2m_wifi_stop_provision_mode
    tstrM2MAPConfig

@warning
        DO Not use ".local" in the pcHttpServerDomainName.
        

\section WIFIExample1 Example
  The example demonstrates a code snippet for how provisioning is triggered and the response event 
  received accordingly. 

@code
    #include "m2m_wifi.h"
    #include "m2m_types.h"

    void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        switch(u8WiFiEvent)
        {
        case M2M_WIFI_RESP_PROVISION_INFO:
            {
                tstrM2MProvisionInfo	*pstrProvInfo = (tstrM2MProvisionInfo*)pvMsg;
                if(pstrProvInfo->u8Status == M2M_SUCCESS)
                {
                    m2m_wifi_connect((char*)pstrProvInfo->au8SSID, (uint8)strlen(pstrProvInfo->au8SSID), pstrProvInfo->u8SecType, 
                            pstrProvInfo->au8Password, M2M_WIFI_CH_ALL);

                    printf("PROV SSID : %s\n",pstrProvInfo->au8SSID);
                    printf("PROV PSK  : %s\n",pstrProvInfo->au8Password);
                }
                else
                {
                    printf("(ERR) Provisioning Failed\n");
                }
            }
            break;

            default:
            break;
        }
    }

    int main()
    {
        tstrWifiInitParam 	param;
        
        param.pfAppWifiCb	= wifi_event_cb;
        if(!m2m_wifi_init(&param))
        {
            tstrM2MAPConfig		apConfig;
            uint8				bEnableRedirect = 1;
            
            strcpy(apConfig.au8SSID, "WINC_SSID");
            apConfig.u8ListenChannel 	= 1;
            apConfig.u8SecType			= M2M_WIFI_SEC_OPEN;
            apConfig.u8SsidHide			= 0;
            
            // IP Address
            apConfig.au8DHCPServerIP[0]	= 192;
            apConfig.au8DHCPServerIP[1]	= 168;
            apConfig.au8DHCPServerIP[2]	= 1;
            apConfig.au8DHCPServerIP[0]	= 1;

            m2m_wifi_start_provision_mode(&apConfig, "atmelwincconf.com", bEnableRedirect);
                        
            while(1)
            {
                m2m_wifi_handle_events(NULL);
            }
        }
    }
@endcode
*/
NMI_API sint8 m2m_wifi_start_provision_mode(tstrM2MAPConfig *pstrAPConfig, char *pcHttpServerDomainName, uint8 bEnableHttpRedirect);

/*!
@fn	\
    sint8 m2m_wifi_stop_provision_mode(void);

@brief
    Synchronous API for terminating provisioning mode on the WINC IC.

@details	
    This function will terminate any currently active provisioning mode on the WINC IC, returning the 
	IC to idle.
	
@return
    The function returns ZERO for success and a negative value otherwise.

@pre
    An active provisioning session must be active before it is terminated through this function.
@see
    m2m_wifi_start_provision_mode
*/
NMI_API sint8 m2m_wifi_stop_provision_mode(void);

/*!
@fn	\
    sint8 m2m_wifi_get_connection_info(void);

@brief
    Asynchronous API for retrieving the WINC IC's connection status.

@details
    Requests the connection status from the WINC IC including information regarding any access 
	point to which it is currently connected, or any non-AP station that is connected to the WINC. 
	All information will be returned to the application via the Wi-Fi notifiction callback and the 
	event @ref M2M_WIFI_RESP_CONN_INFO.
	
	The connection info can be retrieved using the structure @ref tstrM2MConnInfo which contains:
    - Connection Security
    - Connection RSSI
    - Remote MAC address
    - Remote IP address
    - SSID of the network (in cases where the WINC is in non-AP mode)

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.
	
@pre	
    - A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at 
	  startup. Registering the callback is done through passing it to the initialization 
	  @ref m2m_wifi_init function.
    - The event @ref M2M_WIFI_RESP_CONN_INFO must be handled in the callback to receive the 
	  requested connection info.
    
@warning
    - In case of WINC AP mode or P2P mode, the SSID field shall be a NULL string.

@see
    M2M_WIFI_RESP_CONN_INFO,
    tstrM2MConnInfo
\section WIFIExample2 Example
  The code snippet shows an example of how wi-fi connection information is retrieved .
@code
    #include "m2m_wifi.h"
    #include "m2m_types.h"

    void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        switch(u8WiFiEvent)
        {
        case M2M_WIFI_RESP_CONN_INFO:
            {
                tstrM2MConnInfo		*pstrConnInfo = (tstrM2MConnInfo*)pvMsg;
                
                printf("CONNECTED AP INFO\n");
                printf("SSID     			: %s\n",pstrConnInfo->acSSID);
                printf("SEC TYPE 			: %d\n",pstrConnInfo->u8SecType);
                printf("Signal Strength		: %d\n", pstrConnInfo->s8RSSI); 
                printf("Local IP Address	: %d.%d.%d.%d\n", 
                    pstrConnInfo->au8IPAddr[0] , pstrConnInfo->au8IPAddr[1], pstrConnInfo->au8IPAddr[2], pstrConnInfo->au8IPAddr[3]);
            }
            break;

        case M2M_WIFI_REQ_DHCP_CONF:
            {
                // Get the current AP information.
                m2m_wifi_get_connection_info();
            }
            break;
        default:
            break;
        }
    }

    int main()
    {
        tstrWifiInitParam 	param;
        
        param.pfAppWifiCb	= wifi_event_cb;
        if(!m2m_wifi_init(&param))
        {
            // connect to the default AP
            m2m_wifi_default_connect();
                        
            while(1)
            {
                m2m_wifi_handle_events(NULL);
            }
        }
    }
@endcode
*/
NMI_API sint8 m2m_wifi_get_connection_info(void);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_set_mac_address(uint8 au8MacAddress[6]);

@brief
    Synchronous API for assigning a MAC address to the WINC IC.
	
@details
    This function is intended to allow non-production software to assign a MAC address to the WINC IC.
	
@param [in]	au8MacAddress
                MAC Address to be provisioned to the WINC.

@return		
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@warning
    This function is intended for development use only and not for use in production software.
*/
NMI_API sint8 m2m_wifi_set_mac_address(uint8 au8MacAddress[6]);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_wps(uint8 u8TriggerType,const char * pcPinNumber);

@brief
    Asynchronous API to engage the WINC IC's Wi-Fi Protected Setup (enrolee) function.

@details	
    This function can be called to make the WINC enter WPS (Wi-Fi Protected Setup) mode. The result 
	is passed to the Wi-Fi notification callback with the event @ref M2M_WIFI_REQ_WPS.
	
@param [in]	u8TriggerType
    WPS Trigger method. This may be:
        - [WPS_PIN_TRIGGER](@ref WPS_PIN_TRIGGER)   Push button method
        - [WPS_PBC_TRIGGER](@ref WPS_PBC_TRIGGER)	Pin method
                
@param [in]	pcPinNumber
    Valid only if the u8TriggerType is WPS_PIN_TRIGGER, this parameter contains the PIN number.
    The number must follow the format as given in the WSC1.0 specification.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@warning
    This function is not supported in AP or P2P modes.
    
@pre	
    - A Wi-Fi notification callback of type (@ref tpfAppWifiCb MUST be implemented and registered at 
	  startup. Registering the callback is done through passing it to the 
	  [m2m_wifi_init](@ref m2m_wifi_init).
    - The event [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS) must be handled in the callback to receive 
	  the WPS status.
    - The WINC device MUST be in IDLE or STA mode. If AP or P2P mode is active, the WPS will not be 
	  performed. 
    - The [m2m_wifi_handle_events](@ref m2m_wifi_handle_events) MUST be called to receive the responses 
	  in the callback.

@see
    tpfAppWifiCb
    m2m_wifi_init
    M2M_WIFI_REQ_WPS
    tenuWPSTrigger
    tstrM2MWPSInfo

\section WIFIExample3 Example
  The code snippet shows an example of how wifi WPS is triggered .
@code
    #include "m2m_wifi.h"
    #include "m2m_types.h"

    void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        switch(u8WiFiEvent)
        {
        case M2M_WIFI_REQ_WPS:
            {
                tstrM2MWPSInfo	*pstrWPS = (tstrM2MWPSInfo*)pvMsg;
                if(pstrWPS->u8AuthType != 0)
                {
                    printf("WPS SSID           : %s\n",pstrWPS->au8SSID);
                    printf("WPS PSK            : %s\n",pstrWPS->au8PSK);
                    printf("WPS SSID Auth Type : %s\n",pstrWPS->u8AuthType == M2M_WIFI_SEC_OPEN ? "OPEN" : "WPA/WPA2");
                    printf("WPS Channel        : %d\n",pstrWPS->u8Ch + 1);
                    
                    // establish Wi-Fi connection
                    m2m_wifi_connect((char*)pstrWPS->au8SSID, (uint8)m2m_strlen(pstrWPS->au8SSID),
                        pstrWPS->u8AuthType, pstrWPS->au8PSK, pstrWPS->u8Ch);
                }
                else
                {
                    printf("(ERR) WPS Is not enabled OR Timed out\n");
                }
            }
            break;
            
        default:
            break;
        }
    }

    int main()
    {
        tstrWifiInitParam 	param;
        
        param.pfAppWifiCb	= wifi_event_cb;
        if(!m2m_wifi_init(&param))
        {
            // Trigger WPS in Push button mode.
            m2m_wifi_wps(WPS_PBC_TRIGGER, NULL);
            
            while(1)
            {
                m2m_wifi_handle_events(NULL);
            }
        }
    }
@endcode
*/


NMI_API sint8 m2m_wifi_ble_set_gain_table(uint8 table_idx);
/*!
@fn	\
    NMI_API sint8 m2m_wifi_ble_set_gain_table(uint8 table_idx);

@brief
    Asynchronous API that notifies the WINC with the Gain Table index from Flash that should use to configure the WiFi and BLE gains.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.
*/

NMI_API sint8 m2m_wifi_wps(uint8 u8TriggerType,const char  *pcPinNumber);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_wps_disable(void);

@brief
    Synchronous API that disables Wi-Fi Protected Setup mode in the WINC IC.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_wps_disable(void);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_p2p(uint8 u8Channel);

@brief
    Asynchronous API for enabling Wi-Fi Direct (P2P) mode in the WINC IC.

@param [in]	u8Channel
    P2P Listen RF channel.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@pre
    - A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at 
      initialization. Registering the callback
      is done through passing it to the @ref m2m_wifi_init.
    - The events @ref M2M_WIFI_RESP_CON_STATE_CHANGED and @ref M2M_WIFI_REQ_DHCP_CONF 
      must be handled in the callback.
    - The @ref m2m_wifi_handle_events MUST be called to receive the responses in the callback.

@warning
    - This function is not available in the WINC 3400
    - This function is not allowed in AP or STA modes.

@see
    tpfAppWifiCb
    m2m_wifi_init
    M2M_WIFI_RESP_CON_STATE_CHANGED
    M2M_WIFI_REQ_DHCP_CONF
    tstrM2mWifiStateChanged

\section WIFIExample4 Example
  The code snippet shows an example of how the p2p mode operates.
@code
    #include "m2m_wifi.h"
    #include "m2m_types.h"
    
    void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        switch(u8WiFiEvent)
        {
        case M2M_WIFI_RESP_CON_STATE_CHANGED:
            {
                tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged*)pvMsg;
                M2M_INFO("Wifi State :: %s :: ErrCode %d\n", pstrWifiState->u8CurrState? "CONNECTED":"DISCONNECTED",pstrWifiState->u8ErrCode);
                
                // Do something
            }
            break;
            
        case M2M_WIFI_REQ_DHCP_CONF:
            {
                uint8	*pu8IPAddress = (uint8*)pvMsg;

                printf("P2P IP Address \"%u.%u.%u.%u\"\n",pu8IPAddress[0],pu8IPAddress[1],pu8IPAddress[2],pu8IPAddress[3]);
            }
            break;
            
        default:
            break;
        }
    }
    
    int main()
    {
        tstrWifiInitParam 	param;
        
        param.pfAppWifiCb	= wifi_event_cb;
        if(!m2m_wifi_init(&param))
        {
            // Trigger P2P
            m2m_wifi_p2p(M2M_WIFI_CH_1);
            
            while(1)
            {
                m2m_wifi_handle_events(NULL);
            }
        }
    }
    
@endcode

*/
NMI_API sint8 m2m_wifi_p2p(uint8 u8Channel);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_p2p_disconnect(void);

@brief
    Synchronous API to disable Wi-Fi Direct (P2P) Mode on the WINC IC. 

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@see
    m2m_wifi_p2p
*/
NMI_API sint8 m2m_wifi_p2p_disconnect(void);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_enable_ap(CONST tstrM2MAPConfig* pstrM2MAPConfig);

@brief
    Asynchronous API to enable access point (AKA "hot-spot") mode on the WINC IC

@details
    The WINC IC supports the ability to operate as an access point with the limitation that:
	  - only 1 station may be associated at any one time
	  - open system and WEP are the only security suites supported.

@param [in]	pstrM2MAPConfig
    A structure holding the AP configurations.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@warning
    This function is not allowed in P2P or STA modes.
    
@pre
    - A Wi-Fi notification callback of type @ref tpfAppWifiCb  MUST be implemented and registered at initialization. Registering the callback
      is done through passing it to the [m2m_wifi_init](@ref m2m_wifi_init).
    - The event @ref M2M_WIFI_REQ_DHCP_CONF must be handled in the callback.
    - The @ref m2m_wifi_handle_events MUST be called to receive the responses in the callback.

@see
    tpfAppWifiCb
    tenuM2mSecType
    m2m_wifi_init
    M2M_WIFI_REQ_DHCP_CONF
    tstrM2mWifiStateChanged
    tstrM2MAPConfig

\section WIFIExample5 Example
  The code snippet demonstrates how the AP mode is enabled after the driver is initialized in the application's main function and the handling
  of the event @ref M2M_WIFI_REQ_DHCP_CONF, to indicate successful connection.
@code
    #include "m2m_wifi.h"
    #include "m2m_types.h"
    
    void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        switch(u8WiFiEvent)
        {
        case M2M_WIFI_REQ_DHCP_CONF:
            {
                uint8	*pu8IPAddress = (uint8*)pvMsg;

                printf("Associated STA has IP Address \"%u.%u.%u.%u\"\n",pu8IPAddress[0],pu8IPAddress[1],pu8IPAddress[2],pu8IPAddress[3]);
            }
            break;
            
        default:
            break;
        }
    }
    
    int main()
    {
        tstrWifiInitParam 	param;
        
        param.pfAppWifiCb	= wifi_event_cb;
        if(!m2m_wifi_init(&param))
        {
            tstrM2MAPConfig		apConfig;
            
            strcpy(apConfig.au8SSID, "WINC_SSID");
            apConfig.u8ListenChannel 	= 1;
            apConfig.u8SecType			= M2M_WIFI_SEC_OPEN;
            apConfig.u8SsidHide			= 0;
            
            // IP Address
            apConfig.au8DHCPServerIP[0]	= 192;
            apConfig.au8DHCPServerIP[1]	= 168;
            apConfig.au8DHCPServerIP[2]	= 1;
            apConfig.au8DHCPServerIP[0]	= 1;
            
            // Trigger AP
            m2m_wifi_enable_ap(&apConfig);
            
            while(1)
            {
                m2m_wifi_handle_events(NULL);
            }
        }
    }

@endcode

*/
NMI_API sint8 m2m_wifi_enable_ap(CONST tstrM2MAPConfig* pstrM2MAPConfig);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_disable_ap(void);

@brief
    Synchronous API to disable access point mode on the WINC IC.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@see
    m2m_wifi_enable_ap
*/
NMI_API sint8 m2m_wifi_disable_ap(void);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_set_static_ip(tstrM2MIPConfig * pstrStaticIPConf);

@brief
    Synchronous API to manually assign a (static) IP address to the WINC IC.

@details
    Typically an infrastructure access point will be able to provide an IP address to all clients 
    after they associate. The WINC will request configuration via DHCP automatically after
    successfully connecting to an access point. This function should only be called in the event 
    that the network has no DHCP server and the application knows the specifics of the network.

@param [in]	pstrStaticIPConf
    Pointer to a structure holding the static IP Configurations (IP, Gateway, subnet mask and 
    DNS address).

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@pre
    The application must first call @ref m2m_wifi_enable_dhcp to request that DHCP functionality is
    disabled prior to calling this function.

@warning
    Exercise caution using this function. DHCP is the preferred method for configuring IP addresses.
	
@see
    tstrM2MIPConfig
*/
NMI_API sint8 m2m_wifi_set_static_ip(tstrM2MIPConfig * pstrStaticIPConf);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_request_dhcp_client(void);

@brief
    Legacy (deprecated) Asynchronous API for starting a DHCP client on the WINC IC.
	
@details
    This is a legacy API and is no longer supported. Calls to this API will not result in any 
	changes being made to the state of the WINC IC. 

@return
    This function always returns @ref M2M_SUCCESS.

@warning
    - This function has been deprecated. DHCP is used automatically when the WINC IC connects.
*/
NMI_API sint8 m2m_wifi_request_dhcp_client(void);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_request_dhcp_server(uint8* addr);

@brief
    Legacy (deprecated) Asynchronous function to start a DHCP client on the WINC IC.

@details
    This is a legacy API and is no longer supported. Calls to this API will not result in any 
	changes being made to the state of the WINC IC. 

@param [in] addr The address to issue to a connected client (only one client is supported)

@return 
    This function always returns @ref M2M_SUCCESS.
	
@warning
    - This function has been deprecated. DHCP is used automatically when the WINC IC connects.
*/
NMI_API sint8 m2m_wifi_request_dhcp_server(uint8* addr);

/*!
@fn	\
    NMI_API  sint8 m2m_wifi_enable_dhcp(uint8  u8DhcpEn );
    
@brief
    Asynchronous function to control the DHCP client functionality within the WINC IC.

@details
    This function allows the application to control the behaviour of the DHCP client function within 
	the WINC IC	once it has associated with an access point. DHCP client functionality is enabled by 
	default.
	
@param [in]	 u8DhcpEn 
    The state of the DHCP client feature after successful association with an access point:
      - 0: DHCP client will be disabled after connection.
      - 1: DHCP client will be enabled after connection.

@return
    The function SHALL return 0 for success and a negative value otherwise.
 
@warning
    - DHCP client is enabled by default
    - This Function should be called to disable DHCP client operation before using @ref m2m_wifi_set_static_ip

@see
    m2m_wifi_set_static_ip()
*/
NMI_API sint8 m2m_wifi_enable_dhcp(uint8  u8DhcpEn );


/*!
@fn	\
    sint8 m2m_wifi_set_scan_options(tstrM2MScanOption* ptstrM2MScanOption)

@brief
    Synchronous API for configuring the behaviour of the WINC IC's network scanning functions.

@details
    This function allows the application to tune the scanning behaviour of the WINC IC using the
	parameters described in @ref tstrM2MScanOption.
	
@param [in]	ptstrM2MScanOption;
    Pointer to the structure holding the Scan Parameters.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@see
    tenuM2mScanCh
    m2m_wifi_request_scan
    tstrM2MScanOption
*/
NMI_API sint8 m2m_wifi_set_scan_options(tstrM2MScanOption* ptstrM2MScanOption);

/*!
@fn	\
    sint8 m2m_wifi_set_scan_region(uint16 ScanRegion)

@brief
    Synchronous API for configuring the regulatory restrictions that may affect the WINC ICs 
	scanning behaviour.
	
@details
    This function sets a property called the scan region, a parameter that affects the range of 
	channels that the WINC IC may legally scan given a geographic region.
 
@param [in]	ScanRegion

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@see
    tenuM2mScanRegion
    m2m_wifi_request_scan
    
*/
NMI_API sint8 m2m_wifi_set_scan_region(uint16  ScanRegion);

/*!
@fn	\
    NMI_API sint8 m2m_wifi_request_scan(uint8 ch);

@brief
	Asynchronous API to request the WINC IC perform a scan for networks.

@details
	Scan statuses are delivered to the application via the Wi-Fi event callback (@ref tpfAppWifiCb) in 
	three stages. The first step involves the event @ref M2M_WIFI_RESP_SCAN_DONE which, if successful, 
	provides the number of detected networks (access points). The application must then read the list 
	of access points via multiple calls to the asynchronous @ref m2m_wifi_req_scan_result API. For 
	each call to this function, the application will receive (step three) the event 
	@ref M2M_WIFI_RESP_SCAN_RESULT.

@param [in]	ch
				RF Channel ID for SCAN operation. It should be set according to tenuM2mScanCh, with a 
				value of @ref M2M_WIFI_CH_ALL to scan all channels.

@return
	The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@warning
	This API is valid only for STA mode, it may be called regardless of connection state (connected
	or disconnected).

@pre
	- A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at 
	  initialization. Registration of the callback is done via @ref m2m_wifi_init.
	- The events @ref M2M_WIFI_RESP_SCAN_DONE and @ref M2M_WIFI_RESP_SCAN_RESULT must be handled in 
	  the (tpfAppWifiCb) callback.
	- The @ref m2m_wifi_handle_events function must be called to receive the responses in the 
	  callback.

@see	M2M_WIFI_RESP_SCAN_DONE
@see	M2M_WIFI_RESP_SCAN_RESULT
@see	tpfAppWifiCb
@see	tstrM2mWifiscanResult
@see	tenuM2mScanCh
@see	m2m_wifi_init
@see	m2m_wifi_handle_events
@see	m2m_wifi_req_scan_result

\section WIFIExample6 Example
  The code snippet demonstrates an example of how the scan request is called from the application's main function and the handling of
  the events received in response.
@code

    #include "m2m_wifi.h"
    #include "m2m_types.h"
    
    void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        static uint8	u8ScanResultIdx = 0;
        
        switch(u8WiFiEvent)
        {
        case M2M_WIFI_RESP_SCAN_DONE:
            {
                tstrM2mScanDone	*pstrInfo = (tstrM2mScanDone*)pvMsg;
                
                printf("Num of AP found %d\n",pstrInfo->u8NumofCh);
                if(pstrInfo->s8ScanState == M2M_SUCCESS)
                {
                    u8ScanResultIdx = 0;
                    if(pstrInfo->u8NumofCh >= 1)
                    {
                        m2m_wifi_req_scan_result(u8ScanResultIdx);
                        u8ScanResultIdx ++;
                    }
                    else
                    {
                        printf("No AP Found Rescan\n");
                        m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
                    }
                }
                else
                {
                    printf("(ERR) Scan fail with error <%d>\n",pstrInfo->s8ScanState);
                }
            }
            break;

        case M2M_WIFI_RESP_SCAN_RESULT:
            {
                tstrM2mWifiscanResult		*pstrScanResult =(tstrM2mWifiscanResult*)pvMsg;
                uint8						u8NumFoundAPs = m2m_wifi_get_num_ap_found();
                
                printf(">>%02d RI %d SEC %s CH %02d BSSID %02X:%02X:%02X:%02X:%02X:%02X SSID %s\n",
                    pstrScanResult->u8index,pstrScanResult->s8rssi,
                    pstrScanResult->u8AuthType,
                    pstrScanResult->u8ch,
                    pstrScanResult->au8BSSID[0], pstrScanResult->au8BSSID[1], pstrScanResult->au8BSSID[2],
                    pstrScanResult->au8BSSID[3], pstrScanResult->au8BSSID[4], pstrScanResult->au8BSSID[5],
                    pstrScanResult->au8SSID);
                
                if(u8ScanResultIdx < u8NumFoundAPs)
                {
                    // Read the next scan result
                    m2m_wifi_req_scan_result(index);
                    u8ScanResultIdx ++;
                }
            }
            break;
        default:
            break;
        }
    }
    
    int main()
    {
        tstrWifiInitParam 	param;
        
        param.pfAppWifiCb	= wifi_event_cb;
        if(!m2m_wifi_init(&param))
        {
            // Scan all channels
            m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
            
            while(1)
            {
                m2m_wifi_handle_events(NULL);
            }
        }
    }
    
@endcode
*/
NMI_API sint8 m2m_wifi_request_scan(uint8 ch);

/*!
@fn	\
	sint8 m2m_wifi_request_scan_passive(uint8 ch);

@param [in]	ch
				RF Channel ID for SCAN operation. It should be set according to tenuM2mScanCh, with a 
				value of @ref M2M_WIFI_CH_ALL to scan all channels.

@warning
	This function is not allowed in P2P or AP modes. It works only for STA mode (both connected or disconnected states).

@pre
	- A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at initialization. Registering the callback
	  is done through passing it to the @ref m2m_wifi_init.
	- The events @ref M2M_WIFI_RESP_SCAN_DONE and @ref M2M_WIFI_RESP_SCAN_RESULT.
	  must be handled in the callback.
	- The @ref m2m_wifi_handle_events function MUST be called to receive the responses in the callback.

@see	m2m_wifi_request_scan
@see	M2M_WIFI_RESP_SCAN_DONE
@see	M2M_WIFI_RESP_SCAN_RESULT
@see	tpfAppWifiCb
@see	tstrM2MScanOption
@see	tstrM2mWifiscanResult
@see	tenuM2mScanCh
@see	m2m_wifi_init
@see	m2m_wifi_handle_events
@see	m2m_wifi_req_scan_result

@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
sint8 m2m_wifi_request_scan_passive(uint8 ch);

/*!
@fn        NMI_API uint8 m2m_wifi_get_num_ap_found(void);

@brief
    Synchronous API to retrieve the number of AP's found during the last scan operation.

@details
    This function allows the application to recover the number of access points discovered during
	the most recent scan activity. This is achieved via a global variable in the WINC driver that
	is populated when receiving the M2M_WIFI_RESP_SCAN_DONE event. 

@return
    Return the number of AP's found in the last Scan Request.

@see       
    m2m_wifi_request_scan 
    M2M_WIFI_RESP_SCAN_DONE
    M2M_WIFI_RESP_SCAN_RESULT         

@pre
    - m2m_wifi_request_scan must be called first to ensure up to date results are available.

@warning   
    - This function must be called only in the wi-fi callback function when the events 
	  @ref M2M_WIFI_RESP_SCAN_DONE or @ref M2M_WIFI_RESP_SCAN_RESULT are received. Calling this 
	  function in any other place may result in undefined/outdated numbers.
	  
\section WIFIExample7 Example
  The code snippet demonstrates an example of how the scan request is called from the application's main function and the handling of
  the events received in response.
@code
    #include "m2m_wifi.h"
    #include "m2m_types.h"
    
    void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        static uint8	u8ScanResultIdx = 0;
        
        switch(u8WiFiEvent)
        {
        case M2M_WIFI_RESP_SCAN_DONE:
            {
                tstrM2mScanDone	*pstrInfo = (tstrM2mScanDone*)pvMsg;
                
                printf("Num of AP found %d\n",pstrInfo->u8NumofCh);
                if(pstrInfo->s8ScanState == M2M_SUCCESS)
                {
                    u8ScanResultIdx = 0;
                    if(pstrInfo->u8NumofCh >= 1)
                    {
                        m2m_wifi_req_scan_result(u8ScanResultIdx);
                        u8ScanResultIdx ++;
                    }
                    else
                    {
                        printf("No AP Found Rescan\n");
                        m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
                    }
                }
                else
                {
                    printf("(ERR) Scan fail with error <%d>\n",pstrInfo->s8ScanState);
                }
            }
            break;
        
        case M2M_WIFI_RESP_SCAN_RESULT:
            {
                tstrM2mWifiscanResult		*pstrScanResult =(tstrM2mWifiscanResult*)pvMsg;
                uint8						u8NumFoundAPs = m2m_wifi_get_num_ap_found();
                
                printf(">>%02d RI %d SEC %s CH %02d BSSID %02X:%02X:%02X:%02X:%02X:%02X SSID %s\n",
                    pstrScanResult->u8index,pstrScanResult->s8rssi,
                    pstrScanResult->u8AuthType,
                    pstrScanResult->u8ch,
                    pstrScanResult->au8BSSID[0], pstrScanResult->au8BSSID[1], pstrScanResult->au8BSSID[2],
                    pstrScanResult->au8BSSID[3], pstrScanResult->au8BSSID[4], pstrScanResult->au8BSSID[5],
                    pstrScanResult->au8SSID);
                
                if(u8ScanResultIdx < u8NumFoundAPs)
                {
                    // Read the next scan result
                    m2m_wifi_req_scan_result(index);
                    u8ScanResultIdx ++;
                }
            }
            break;
        default:
            break;
        }
    }
    
    int main()
    {
        tstrWifiInitParam 	param;
        
        param.pfAppWifiCb	= wifi_event_cb;
        if(!m2m_wifi_init(&param))
        {
            // Scan all channels
            m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
            
            while(1)
            {
                m2m_wifi_handle_events(NULL);
            }
        }
    }
@endcode			 
*/
NMI_API uint8 m2m_wifi_get_num_ap_found(void);

/*!
@fn          NMI_API sint8 m2m_wifi_req_scan_result(uint8 index);

@brief
    Asynchronous API to request the information of an access point discovered via scanning.

@details
    This function allows the information of any discovered access point to be retrieved. When a 
    scan is completed, the application is informed of the number of networks (access points)
    discovered. Calling this function with an index, N, will return the information for the Nth
	access point. The information will be returned to the application via a 
	@ref M2M_WIFI_RESP_SCAN_RESULT event, and the response data may be obtained through casting 
	the pointer (pvMsg) to @ref tstrM2mWifiscanResult.

@param [in]  index 
    Index for the requested result, the index range start from 0 till number of AP's found

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.
	
@see
    tstrM2mWifiscanResult
    m2m_wifi_get_num_ap_found
    m2m_wifi_request_scan             

@pre
    - @ref m2m_wifi_request_scan must be called first to ensure up to date results are available.
    - A Wi-Fi notification callback of type @ref tpfAppWifiCb must be implemented and registered 
	  in order to receive scan data after calling this function. Registration of the callback
      is done via the @ref m2m_wifi_init function.
    - The event @ref M2M_WIFI_RESP_SCAN_RESULT must be handled in the callback to receive the 
	  requested connection information.

@warning
    - This API is valid only for STA mode, it may be called regardless of connection state (connected
	  or disconnected).
    - Calling this function without first issuing a scan request may lead to stale data being 
	  recovered.
    - Application code should refrain from introducing significant delays between issuing the scan 
	  request and scan result requests.

\section WIFIExample8 Example
  The code snippet demonstrates an example of how the scan request is called from the application's main function and the handling of
  the events received in response.
@code
    #include "m2m_wifi.h"
    #include "m2m_types.h"
    
    void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        static uint8	u8ScanResultIdx = 0;
        
        switch(u8WiFiEvent)
        {
        case M2M_WIFI_RESP_SCAN_DONE:
            {
                tstrM2mScanDone	*pstrInfo = (tstrM2mScanDone*)pvMsg;
                
                printf("Num of AP found %d\n",pstrInfo->u8NumofCh);
                if(pstrInfo->s8ScanState == M2M_SUCCESS)
                {
                    u8ScanResultIdx = 0;
                    if(pstrInfo->u8NumofCh >= 1)
                    {
                        m2m_wifi_req_scan_result(u8ScanResultIdx);
                        u8ScanResultIdx ++;
                    }
                    else
                    {
                        printf("No AP Found Rescan\n");
                        m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
                    }
                }
                else
                {
                    printf("(ERR) Scan fail with error <%d>\n",pstrInfo->s8ScanState);
                }
            }
            break;
        
        case M2M_WIFI_RESP_SCAN_RESULT:
            {
                tstrM2mWifiscanResult		*pstrScanResult =(tstrM2mWifiscanResult*)pvMsg;
                uint8						u8NumFoundAPs = m2m_wifi_get_num_ap_found();
                
                printf(">>%02d RI %d SEC %s CH %02d BSSID %02X:%02X:%02X:%02X:%02X:%02X SSID %s\n",
                    pstrScanResult->u8index,pstrScanResult->s8rssi,
                    pstrScanResult->u8AuthType,
                    pstrScanResult->u8ch,
                    pstrScanResult->au8BSSID[0], pstrScanResult->au8BSSID[1], pstrScanResult->au8BSSID[2],
                    pstrScanResult->au8BSSID[3], pstrScanResult->au8BSSID[4], pstrScanResult->au8BSSID[5],
                    pstrScanResult->au8SSID);
                
                if(u8ScanResultIdx < u8NumFoundAPs)
                {
                    // Read the next scan result
                    m2m_wifi_req_scan_result(index);
                    u8ScanResultIdx ++;
                }
            }
            break;
        default:
            break;
        }
    }
    
    int main()
    {
        tstrWifiInitParam 	param;
        
        param.pfAppWifiCb	= wifi_event_cb;
        if(!m2m_wifi_init(&param))
        {
            // Scan all channels
            m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
            
            while(1)
            {
                m2m_wifi_handle_events(NULL);
            }
        }
    }
    
@endcode 
*/
NMI_API sint8 m2m_wifi_req_scan_result(uint8 index);

/*!
@fn          NMI_API sint8 m2m_wifi_req_curr_rssi(void);

@brief
    Asynchronous API to request the current Receive Signal Strength (RSSI) of the current connection.

@details
    This function will result in the application receiving the RSSI via a 
	@ref M2M_WIFI_RESP_CURRENT_RSSI event.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.	

@pre
    - A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered 
	during initialization. Registration of the callback is done via the 
	[m2m_wifi_init](@ref m2m_wifi_init) API.
    - The event @ref M2M_WIFI_RESP_CURRENT_RSSI must be handled in the callback.       

\section WIFIExample9 Example
  The code snippet demonstrates how the RSSI request is called in the application's main function and the handling of event received in the callback. 
@code
    #include "m2m_wifi.h"
    #include "m2m_types.h"
    
    void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        static uint8	u8ScanResultIdx = 0;
        
        switch(u8WiFiEvent)
        {
        case M2M_WIFI_RESP_CURRENT_RSSI:
            {
                sint8	*rssi = (sint8*)pvMsg;
                M2M_INFO("ch rssi %d\n",*rssi);
            }
            break;
        default:
            break;
        }
    }
    
    int main()
    {
        tstrWifiInitParam 	param;
        
        param.pfAppWifiCb	= wifi_event_cb;
        if(!m2m_wifi_init(&param))
        {
            // Scan all channels
            m2m_wifi_req_curr_rssi();
            
            while(1)
            {
                m2m_wifi_handle_events(NULL);
            }
        }
    }

@endcode	
*/
NMI_API sint8 m2m_wifi_req_curr_rssi(void);

/*!
@fn          NMI_API sint8 m2m_wifi_req_restrict_ble(void);

@brief
    Asynchronous API to request restricting of BLE functionality by placing the BLE processor in a low power state.
    It is recommended to do this if it is know that BLE functionality will not be used for any significant length of time.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.	

@pre
    - A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered 
	during initialization. Registration of the callback is done via the 
	[m2m_wifi_init](@ref m2m_wifi_init) API.

*/
NMI_API sint8 m2m_wifi_req_restrict_ble(void);

/*!
@fn          NMI_API sint8 m2m_wifi_req_unrestrict_ble(void);

@brief
    Asynchronous API to request un-restricting of BLE functionality by reverting the BLE processor to full power mode. 

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.	

@pre
    - A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered 
	during initialization. Registration of the callback is done via the 
	[m2m_wifi_init](@ref m2m_wifi_init) API.

*/
NMI_API sint8 m2m_wifi_req_unrestrict_ble(void);

/*!
@fn          NMI_API sint8 m2m_wifi_get_otp_mac_address(uint8 *pu8MacAddr, uint8 * pu8IsValid);

@brief
    Synchronous API to query the MAC address programmed into the WINC ICs OTP memory

@details
    This function attempts to read the device's MAC address from the One Time Programmable (OTP)
	memory on the IC. The presence (yes or no) of a MAC address in the OTP memory and, in the case
	of it being present, its value are returned via RAM pointed to by the input arguments.
	
@param [out] pu8MacAddr
    Output MAC address buffer 6 bytes in size. This is valid only if *pu8Valid equals 1.
			 
@param [out] pu8IsValid
    A boolean value set by the callee to indicate the validity of pu8MacAddr in OTP. If no MAC has
	been programmed in the OTP the value of this parameter will be zero; otherwise it will be 
	non-zero.

@return
    The function returns @ref M2M_SUCCESS for success and a negative value otherwise.

@see         
    m2m_wifi_get_mac_address             

*/
NMI_API sint8 m2m_wifi_get_otp_mac_address(uint8 *pu8MacAddr, uint8 * pu8IsValid);

/*!
@fn          NMI_API sint8 m2m_wifi_get_mac_address(uint8 *pu8MacAddr)	

@brief
    Synchronous API to retrieve the MAC address currently in use on the device.

@details
    This function obtains the MAC address that is currently in use by the device. If the function
	returns with @ref M2M_SUCCESS then the content of the memory referenced by pu8MacAddr will be 
	populated with the 6 byte MAC address; otherwise, that memory will be left unchanged.
	
@param [out] pu8MacAddr
    Pointer to a buffer in memory containing a 6-byte MAC address (provided function returns 
	MEM_SUCCESS)

@return
    The function returns @ref M2M_SUCCESS for successful operation and a negative value otherwise.

@see         
    m2m_wifi_get_otp_mac_address             
*/
NMI_API sint8 m2m_wifi_get_mac_address(uint8 *pu8MacAddr);

/*!
@fn			NMI_API sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn);

@brief
    Synchronous API to set the power-save mode of the WINC IC.
	
@param [in]	PsTyp
    Desired power saving mode. Supported types are defined in @ref tenuPowerSaveModes.

@param [in]	BcastEn
    Broadcast reception enable flag. 
      - If set to 1, the WINC IC will wake for each DTIM beacon to ensure broadcast traffic can be
	    received.
      - If set to 0, the WINC IC will not wakeup at the DTIM beacon, instead it will wake every N
	    beacon periods as per the negotiated Listen Interval. 

@return
    The function returns @ref M2M_SUCCESS for successful operation and a negative value otherwise.
	
@see
   tenuPowerSaveModes
   m2m_wifi_get_sleep_mode
*/
NMI_API sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn);

/*!
@fn	        NMI_API sint8 m2m_wifi_request_sleep(uint32 u32SlpReqTime);

@brief
    Asynchronous API to place the WINC IC into sleep mode for a specified period of time.

@details
    This function allows a WINC IC that is running in M2M_PS_MANUAL mode to be placed into sleep
	state for a specified period of time (measured in milliseconds).
 
@param [in]	u32SlpReqTime
    The time in ms that the WINC IC should sleep for 

@return
    The function returns @ref M2M_SUCCESS for successful operation and a negative value otherwise.

@warning
    - This API is currently unsupported on the WINC3400

@see
    tenuPowerSaveModes 
    m2m_wifi_set_sleep_mode
*/
NMI_API sint8 m2m_wifi_request_sleep(uint32 u32SlpReqTime);

/*!
@fn		    NMI_API uint8 m2m_wifi_get_sleep_mode(void);

@brief
    Synchronous API to retrieve the current power save mode of the WINC IC.
	
@return
	The current operating power saving mode. The value will be one of those from the enumerated type 
    @ref tenuPowerSaveModes.

@see
    tenuPowerSaveModes 
    m2m_wifi_set_sleep_mode
*/
NMI_API uint8 m2m_wifi_get_sleep_mode(void);

#if 0
/*
 * These two functions are for a mode in which two WINC ICs communicate with each other
 * via probe request and probe response frames. This mode is not supported in WINC fw.
 */
NMI_API sint8 m2m_wifi_req_client_ctrl(uint8 cmd);
NMI_API sint8 m2m_wifi_req_server_init(uint8 ch);
#endif

/*!
@fn			NMI_API sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength);		

@brief
    Asynchronous API to set the Wi-Fi Direct "Device Name" of the WINC IC

@param [in]	pu8DeviceName
    Buffer holding the device name.
	
@param [in]	u8DeviceNameLength
    Length of the device name. Should not exceed the maximum device name's length M2M_DEVICE_NAME_MAX.
	
@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength);

/*!
@fn			NMI_API sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt * pstrM2mLsnInt);

@brief
    Asynchronous API to set Wi-Fi listen interval for power save operation. 

@param [in]	pstrM2mLsnInt
            Structure holding the listen interval configurations.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@pre
    The function @ref m2m_wifi_set_sleep_mode shall be called first
	
@see
    - tstrM2mLsnInt
    - m2m_wifi_set_sleep_mode
*/
NMI_API sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt *pstrM2mLsnInt);

/*!
@fn             NMI_API sint8 m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl *, uint8 *, uint16 , uint16);

@brief
    Asynchronous call to enable Wi-Fi monitoring (promiscuous receive) mode.

@details
    This function places the reciver into a mode where all frames (or those matching a set of filter
	criteria) received on air are passed to the application.
	
	A dedicated callback function, @ref tpfAppMonCb, must be registered to handle frames received in 
	promiscuous mode. This is done via an instance of a @ref tstrWifiInitParam structure and a call to
	the @ref m2m_wifi_init function.
  
@param [in]     pstrMtrCtrl
    Pointer to @ref tstrM2MWifiMonitorModeCtrl structure holding the Filtering parameters.

@param [in]     pu8PayloadBuffer
    Pointer to a Buffer allocated by the application. The buffer SHALL hold the Data field of 
    the WIFI RX Packet (Or a part from it). If it is set to NULL, the WIFI data payload will 
    be discarded by the monitoring driver.

@param [in]     u16BufferSize
    The total size of the pu8PayloadBuffer in bytes.
	
@param [in]     u16DataOffset
    Starting offset in the DATA FIELD of the received WIFI packet. The application may be interested
    in reading specific information from the received packet. It must assign the offset to the starting
    position of it relative to the DATA payload start.\n
    \e Example, \e if \e the \e SSID \e is \e needed \e to \e be \e read \e from \e a \e PROBE \e REQ 
	\e packet, \e the \e u16Offset \e MUST \e be \e set \e to \e 0.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@warning        
    This mode available as sniffer ONLY, you can not be connected in any modes (Station, Access Point, 
	or P2P).
	
 @see
    - tstrM2MWifiMonitorModeCtrl
    - tstrM2MWifiRxPacketInfo
    - tstrWifiInitParam
    - tenuM2mScanCh
    - m2m_wifi_disable_monitoring_mode               

\section WIFIExample10 Example
    The example demonstrates the main function where-by the monitoring enable function is called after 
	the initialization of the driver and the packets are handled in the callback function.

@code
    #include "m2m_wifi.h"
    #include "m2m_types.h"

    //Declare receive buffer 
    uint8 gmgmt[1600];
    
    //Callback functions
    void wifi_cb(uint8 u8WiFiEvent, void * pvMsg)
    {
        ; 
    }

    void wifi_monitoring_cb(tstrM2MWifiRxPacketInfo *pstrWifiRxPacket, uint8 *pu8Payload, uint16 u16PayloadSize)
    {
        if((NULL != pstrWifiRxPacket) && (0 != u16PayloadSize)) {
            if(MANAGEMENT == pstrWifiRxPacket->u8FrameType) {
                M2M_INFO("***# MGMT PACKET #***\n");
            } else if(DATA_BASICTYPE == pstrWifiRxPacket->u8FrameType) {
                M2M_INFO("***# DATA PACKET #***\n");
            } else if(CONTROL == pstrWifiRxPacket->u8FrameType) {
                M2M_INFO("***# CONTROL PACKET #***\n");
            }
        }
    }
            
    int main()
    {
        //Register wifi_monitoring_cb 
        tstrWifiInitParam param;
        param.pfAppWifiCb = wifi_cb;
        param.pfAppMonCb  = wifi_monitoring_cb;
                
        nm_bsp_init();
                
        if(!m2m_wifi_init(&param)) {
            //Enable Monitor Mode with filter to receive all data frames on channel 1
            tstrM2MWifiMonitorModeCtrl	strMonitorCtrl = {0};
            strMonitorCtrl.u8ChannelID		= M2M_WIFI_CH_1;
            strMonitorCtrl.u8FrameType		= DATA_BASICTYPE;
            strMonitorCtrl.u8FrameSubtype	= M2M_WIFI_FRAME_SUB_TYPE_ANY; //Receive any subtype of data frame
            m2m_wifi_enable_monitoring_mode(&strMonitorCtrl, gmgmt, sizeof(gmgmt), 0);
                    
            while(1) {
                m2m_wifi_handle_events(NULL);
            }
        }
        return 0;
    }
@endcode
*/
NMI_API sint8 m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl *pstrMtrCtrl, uint8 *pu8PayloadBuffer, 
                                           uint16 u16BufferSize, uint16 u16DataOffset);

/*!
@fn             NMI_API sint8 m2m_wifi_disable_monitoring_mode(void);

@brief
    Asynchronous API to disable Wi-Fi monitoring (promiscuous receive) mode.

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@see
    m2m_wifi_enable_monitoring_mode               
 */
NMI_API sint8 m2m_wifi_disable_monitoring_mode(void);

/*!
@fn             NMI_API sint8 m2m_wifi_send_wlan_pkt(uint8 *, uint16, uint16);

@brief
    Asynchronous API to queue a raw Wi-Fi packet for transmission by the WINC IC.

@param [in]     pu8WlanPacket
    Pointer to a buffer holding the whole WIFI frame.
 
@param [in]     u16WlanHeaderLength
    The size of the WIFI packet header ONLY.

@param [in]     u16WlanPktSize
    The size of the whole bytes in packet. 
	
@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.
	
@see
    - m2m_wifi_enable_monitoring_mode
 	- m2m_wifi_disable_monitoring_mode
	
@warning
    This function is only available in builds supporting monitoring mode.
*/
NMI_API sint8 m2m_wifi_send_wlan_pkt(uint8 *pu8WlanPacket, uint16 u16WlanHeaderLength, uint16 u16WlanPktSize);

/*!
@fn           NMI_API sint8 m2m_wifi_send_ethernet_pkt(uint8* pu8Packet,uint16 u16PacketSize)

@brief
    Asynchronous API to queue an Ethernet packet for transmission by the WINC IC.

@param [in]     pu8Packet
    Pointer to a buffer holding the whole Ethernet frame.

@param [in]     u16PacketSize
    The size of the whole bytes in packet.
	
@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@see
    - m2m_wifi_enable_monitoring_mode
 	- m2m_wifi_disable_monitoring_mode
	
@warning
    This function is only available in builds supporting monitoring mode.
*/
NMI_API sint8 m2m_wifi_send_ethernet_pkt(uint8* pu8Packet,uint16 u16PacketSize);

/*!
@fn             NMI_API sint8 m2m_wifi_enable_sntp(uint8);

@brief
    Asynchronous API to enable or disable the SNTP client running on the WINC IC.

@details
    The SNTP client is enabled by default during chip initialisation. This function can be used to 
	disable or subsequently re-enable the service.
	
	The service is capable of syncrhonising the WINC's system clock to the UTC time from a well-known
	(and trusted) time server, for example "time.nist.gov". By default the SNTP client will update the
	system time once every 24 hours. The ability to track the time accurately is important for various
	applications such as checking expiry of X509 certificates during TLS session establishment.
	
    It is highly recommended to leave SNTP enabled if there is no alternative source of timing 
	information. For systems including an RTC device, SNTP may not be needed and the WINC IC's time 
	may be set using the @ref m2m_wifi_set_sytem_time function.
	
@param [in]     bEnable
    Enables or disables the SNTP service
      - '0' : disable SNTP
      - '1' : enable SNTP  

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise. 

@see             m2m_wifi_set_sytem_time       
 */
NMI_API sint8 m2m_wifi_enable_sntp(uint8 bEnable);

/*!
@fn             NMI_API sint8 m2m_wifi_set_sytem_time(uint32);   

@brief
    Asynchronous function for setting the system time within the WINC IC.

@param [in]     u32UTCSeconds
    UTC value in seconds. 

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@see
    - m2m_wifi_enable_sntp
    - tstrSystemTime   

@note
    If there is an RTC on the host MCU, the SNTP may be disabled and the host may set the system 
	time within the firmware using the API @ref m2m_wifi_set_sytem_time.
 */
NMI_API sint8 m2m_wifi_set_sytem_time(uint32 u32UTCSeconds);

/*!
@fn             NMI_API sint8 m2m_wifi_get_sytem_time(void);   

@brief 
    Asynchronous API to obtain the system time in use by the WINC IC.
    
@details
    This function will request that the WINC IC send it's current system time to the application. The
	information will arrive at the application via the @ref tpfAppWifiCb and event @ref 
	M2M_WIFI_RESP_GET_SYS_TIME.
	
@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@pre
    - A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered 
	during initialization. Registration of the callback is done via the 
	[m2m_wifi_init](@ref m2m_wifi_init) API.
    - The event @ref M2M_WIFI_RESP_GET_SYS_TIME must be handled in the callback.       
	
@see
    - m2m_wifi_enable_sntp
    - tstrSystemTime   
 */
NMI_API sint8 m2m_wifi_get_sytem_time(void);

/*!
@fn             NMI_API sint8 m2m_wifi_set_cust_InfoElement(uint8*);

@brief
    Asynchronous API to add or remove a user-defined Information Element 
	
@details
    This function allows the application to provide a custom Information Element to the WINC IC that will
	be included in all beacon and probe response frames.

@param [in]     pau8M2mCustInfoElement
    Pointer to Buffer containing the IE to be used. If null, this removes any current custom IE. If 
	non-null, the pointer must reference data in the following format:

@verbatim 
               --------------- ---------- ---------- ------------------- -------- -------- ----------- ----------------------  
              | Byte[0]       | Byte[1]  | Byte[2]  | Byte[3:length1+2] | ..... | Byte[n] | Byte[n+1] | Byte[n+2:lengthx+2]  | 
              |---------------|----------|----------|-------------------|-------- --------|-----------|------------------| 
              | #of all Bytes | IE1 ID   | Length1  | Data1(Hex Coded)  | ..... | IEx ID  | Lengthx   | Datax(Hex Coded)     | 
               --------------- ---------- ---------- ------------------- -------- -------- ----------- ---------------------- 
@endverbatim

@return
     The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	 and a negative value otherwise.
	
@warning
    - Size of All elements combined must not exceed 255 byte.\n
    - Used in Access Point Mode \n

\section WIFIExample11 Example
   The example demonstrates how the information elements are set using this function.
@code
    char elementData[21];
    static char state = 0; // To Add, Append, and Delete
    if(0 == state) {	//Add 3 IEs
        state = 1;
        //Total Number of Bytes
        elementData[0]=12;
        //First IE
        elementData[1]=200; elementData[2]=1; elementData[3]='A';
        //Second IE
        elementData[4]=201; elementData[5]=2; elementData[6]='B'; elementData[7]='C';
        //Third IE
        elementData[8]=202; elementData[9]=3; elementData[10]='D'; elementData[11]=0; elementData[12]='F';
    } else if(1 == state) {	
        //Append 2 IEs to others, Notice that we keep old data in array starting with\n
        //element 13 and total number of bytes increased to 20
        state = 2; 
        //Total Number of Bytes
        elementData[0]=20;
        //Fourth IE
        elementData[13]=203; elementData[14]=1; elementData[15]='G';
        //Fifth IE
        elementData[16]=204; elementData[17]=3; elementData[18]='X'; elementData[19]=5; elementData[20]='Z';
    } else if(2 == state) {	//Delete All IEs
        state = 0; 
        //Total Number of Bytes
        elementData[0]=0;
    }
    m2m_wifi_set_cust_InfoElement(elementData);	
@endcode
 */
NMI_API sint8 m2m_wifi_set_cust_InfoElement(uint8* pau8M2mCustInfoElement);

/*!
@fn			NMI_API sint8 m2m_wifi_set_power_profile(uint8 u8PwrMode);

@brief		Change the power profile mode 

@param [in]	u8PwrMode
    Change the WINC power profile to one of the modes: PWR_LOW1, PWR_LOW2, PWR_HIGH, PWR_AUTO.

@return
    The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.

@see
    tenuM2mPwrMode

@warning
    May only be called after initialization, before any connection request, and may not be used to change 
	the power mode thereafter. 
*/
sint8 m2m_wifi_set_power_profile(uint8 u8PwrMode);

/*!
@fn			NMI_API sint8 m2m_wifi_set_tx_power(uint8 u8TxPwrLevel);

@brief		set the TX power tenuM2mTxPwrLevel

@param [in]	u8TxPwrLevel
    change the TX power tenuM2mTxPwrLevel

@return
    The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.

@sa			tenuM2mTxPwrLevel
*/
sint8 m2m_wifi_set_tx_power(uint8 u8TxPwrLevel);

/*!
@fn			NMI_API sint8 m2m_wifi_enable_firmware_logs(uint8 u8Enable);

@brief
    Enable or Disable logs in run time (Disable Firmware logs will enhance the firmware start-up time 
	and performance)
	
@param [in]	u8Enable
    Set 1 to enable the logs 0 for disable
	
@return
    The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.

@see
    __DISABLE_FIRMWARE_LOGS__ (build option to disable logs from initializations)
*/
sint8 m2m_wifi_enable_firmware_logs(uint8 u8Enable);
/*!
@fn			NMI_API sint8 m2m_wifi_set_battery_voltage(uint8 u8BattVolt)

@brief
    Set the battery voltage to update the firmware calculations

@param [in]	dbBattVolt
    Battery Voltage
	
@return
    The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.
*/
sint8 m2m_wifi_set_battery_voltage(uint16 u16BattVoltx100);

/*!
@fn             NMI_API sint8 m2m_wifi_ble_api_send(const uint8* const msg, const uint32 len);

@brief
    Asynchronous API to send an encapsulated Atmel BLE message over the Wifi Host Interface 

@param [in] msg
    Pointer to the start of the BLE message to transfer down to the WINC
	
@param [in] len
    The length of the message in octets
	
@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.
 */
NMI_API sint8 m2m_wifi_ble_api_send(uint8* msg, uint32 len);


/*!
@fn             NMI_API sint8 m2m_wifi_enable_mac_mcast(uint8 *, uint8);

@brief
    Asynchronous API to add or remove MAC addresses in the multicast filter

@details
    This function allows the application to configure the capability of the WINC IC to receive multicast
	packets when operating in bypass mode.
  
@param [in]     pu8MulticastMacAddress
    Pointer to MAC address

@param [in]     u8AddRemove
    A flag to add or remove the MAC ADDRESS, based on the following values:
      -  0 : remove MAC address
      -  1 : add MAC address    

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.

@note
    Maximum number of MAC addresses that could be added is 8.

@warning
    This functionality is not supported by current WINC firmware.
 */
NMI_API sint8 m2m_wifi_enable_mac_mcast(uint8* pu8MulticastMacAddress, uint8 u8AddRemove);

/*!
@fn             NMI_API sint8 m2m_wifi_set_receive_buffer(void *, uint16);

@brief
    Asynchronous API to set or change the size of the WINC IC's receive buffer 

@param [in]     pvBuffer
    Pointer to Buffer to receive data. A NULL pointer causes a negative error @ref M2M_ERR_FAIL.

@param [in]     u16BufferLen
    Length of data to be received.  Maximum length of data should not exceed the size defined by TCP/IP
    defined as @ref SOCKET_BUFFER_MAX_LENGTH

@return
    The function returns @ref M2M_SUCCESS if the command has been successfully queued to the WINC, 
	and a negative value otherwise.
 
@warning
    This functionality is not supported by current WINC firmware.
*/
NMI_API sint8 m2m_wifi_set_receive_buffer(void* pvBuffer,uint16 u16BufferLen);

/**@}*/

/**@defgroup  VERSIONAPI Functions
 @ingroup VERSION
 */
/**@{*/


/*!
@fn		uint32 m2m_wifi_get_chipId(void)

@brief
	Synchronous API to obtain the firmware WINC chip ID

@return
    The function SHALL return chipID > 0 or 0 for failure.
*/
uint32 m2m_wifi_get_chipId(void);
/*!
@fn     sint8 m2m_wifi_check_ota_rb(void);

@brief
    Synchronous API to check presence and compatibility of the WINC image that is stored in the inactive flash partition.
    This is the image that would run on the WINC IC if @ref m2m_ota_switch_firmware or @ref m2m_ota_rollback were called,
    followed by a reset of the WINC IC.

@return
    The function SHALL return @ref M2M_SUCCESS for compatible image and a negative value otherwise.
*/
sint8 m2m_wifi_check_ota_rb(void);
/*!
@fn		sint8 m2m_wifi_get_firmware_version(tstrM2mRev* pstrRev)

@brief
	Synchronous API to obtain the firmware version currently running on the WINC IC

@param [out]	pstrRev
    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters

@return
    The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.
*/
sint8 m2m_wifi_get_firmware_version(tstrM2mRev *pstrRev);

/*!
@fn	\
    NMI_API sint8 m2m_ota_get_firmware_version(tstrM2mRev *pstrRev);

@brief
    Synchronous API to obtain the firmware version of the WINC image that is stored in the inactive flash partition.
    This is the image that would run on the WINC IC if @ref m2m_ota_switch_firmware or @ref m2m_ota_rollback were called, 
    followed by a reset of the WINC IC.

@param [out]	pstrRev
    pointer holds address of structure "tstrM2mRev" that contains the ota fw version parameters
	
@return
    The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_get_firmware_version(tstrM2mRev *pstrRev);

/**@}*/

/*!
 * @fn                  sint8 m2m_wifi_prng_get_random_bytes(uint8 * pu8PRNGBuff,uint16 u16PRNGSize)
 * @param [in]      pu8PrngBuff
 *                 		Pointer to Buffer to receive data.
 *		    		Size greater than the maximum specified (@ref M2M_BUFFER_MAX_SIZE - sizeof(tstrPrng))
 *				causes a negative error @ref M2M_ERR_FAIL.
 * @param [in]      u16PrngSize
 					request size in bytes  
 * @return       The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
sint8 m2m_wifi_prng_get_random_bytes(uint8 * pu8PrngBuff,uint16 u16PrngSize);

#ifdef __cplusplus
}
#endif
#endif /* __M2M_WIFI_H__ */

