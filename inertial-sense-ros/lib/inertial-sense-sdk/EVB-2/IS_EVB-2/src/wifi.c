/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// #define WIFI_AUTH	M2M_WIFI_SEC_WPA_PSK
// #define WIFI_SSID	"BobaMachine"
// #define WIFI_PSK	"pineapple"

// #define WIFI_AUTH	M2M_WIFI_SEC_WPA_PSK
// #define WIFI_SSID	"Walt Johnson's iPhone"
// #define WIFI_PSK	"magicctech"

#define WIFI_AUTH                   M2M_WIFI_SEC_WPA_PSK
#define MAIN_WIFI_M2M_PRODUCT_NAME  "EVB-2"

#include <asf.h>
#include <string.h>
#include "wifi.h"
#include "driver/include/m2m_wifi.h"
#include "driver/source/nmasic.h"
#include "common/include/nm_common.h"
#include "socket/include/socket.h"
#include "globals.h"

extern StreamBufferHandle_t         g_xStreamBufferWiFiRx;
extern StreamBufferHandle_t         g_xStreamBufferWiFiTx;

/** Socket for client */
static SOCKET tcp_client_socket = -1;
static int g_init_wifi = 0;       // 1=enable, -1=disable
static uint8_t g_wifi_connected = 0;

static struct sockaddr_in g_addr = {0};

/** Message format definitions. */
typedef struct s_msg_wifi_product {
	uint8_t name[20];
} t_msg_wifi_product;

static t_msg_wifi_product msg_wifi_product = {
    .name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

/** Receive buffer definition. */
#define WIFI_BUFFER_SIZE          1460
static uint8_t          socketRxBuffer[WIFI_BUFFER_SIZE];
static uint8_t          socketTxBuffer[WIFI_BUFFER_SIZE];
static int              g_indicateWiFiRxMs=0;


/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) 
	{
		case M2M_WIFI_RESP_CON_STATE_CHANGED:
		{
			tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
			if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) 
			{
                g_wifi_connected = 1;
                g_status.evbStatus |= EVB_STATUS_WIFI_CONNECTED;
				m2m_wifi_request_dhcp_client();
			} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) 
			{
                g_wifi_connected = 0;
                g_status.evbStatus &= ~EVB_STATUS_WIFI_CONNECTED;
                char* wifi_ssid = (char*)(g_flashCfg->wifi[EVB_CFG_BITS_IDX_WIFI(g_flashCfg->bits)].ssid);
                char* wifi_psk  = (char*)(g_flashCfg->wifi[EVB_CFG_BITS_IDX_WIFI(g_flashCfg->bits)].psk);
				m2m_wifi_connect(wifi_ssid, strlen(wifi_ssid), WIFI_AUTH, wifi_psk, M2M_WIFI_CH_ALL);
			}

			break;
		}

		case M2M_WIFI_REQ_DHCP_CONF:
		{
//             uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
//             printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
            g_status.wifiIpAddr = *(uint32_t *)pvMsg;
			break;
		}

		default:
		{
			break;
		}
	}
}


/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch (u8Msg) {
	/* Socket connected */
	case SOCKET_MSG_CONNECT:
	{
		tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
		if (pstrConnect && pstrConnect->s8Error >= 0) 
        {
			//socket_cb: connect success!
			//Start receive process
			recv(tcp_client_socket, socketRxBuffer, sizeof(socketRxBuffer), 0);
// 			send(tcp_client_socket, &msg_wifi_product, sizeof(t_msg_wifi_product), 0);
		} 
        else 
        {
			//socket_cb: connect error!
			close(tcp_client_socket);
			tcp_client_socket = -1;
		}
	}
	break;

	/* Message send */
	case SOCKET_MSG_SEND:
	{
		//socket_cb: send success!
		LED_ON(LED_WIFI_TXD_PIN);
	}
	break;

	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
		if (pstrRecv && pstrRecv->s16BufferSize > 0) {
			//socket_cb: recv success!
            g_indicateWiFiRxMs = 20;

			if(g_flashCfg->cbf[EVB2_PORT_WIFI] & (1<<EVB2_PORT_UINS1))
			{
#if 0           // Send data to com task
                xStreamBufferSend(g_xStreamBufferWiFiRx, (void*)socketRxBuffer, pstrRecv->s16BufferSize, 0);
#else
				BEGIN_CRITICAL_SECTION
				comWrite(EVB2_PORT_UINS1, socketRxBuffer, pstrRecv->s16BufferSize, LED_INS_TXD_PIN);
				END_CRITICAL_SECTION
#endif
			}

			//Start new receive process
			recv(tcp_client_socket, socketRxBuffer, sizeof(socketRxBuffer), 0);
		} else {
			//socket_cb: recv error!
			close(tcp_client_socket);
			tcp_client_socket = -1;
		}
	}

	break;

	default:
		break;
	}
}


static void init_tcp_socket(void)
{
    if(tcp_client_socket != -1)
    {
        close(tcp_client_socket);
        tcp_client_socket = -1;
    }

    /* Initialize socket module */
    socketInit();
    registerSocketCallback(socket_cb, NULL);
}


static void deinit_tcp_socket(void)
{
    if(tcp_client_socket != -1)
    {
        close(tcp_client_socket);
        tcp_client_socket = -1;
        socketDeinit();
    }    
}


void wifi_enable(int enable)
{
    if(enable)
    {   // Init wifi ONLY if it was off
        if(!(g_status.evbStatus&EVB_STATUS_WIFI_ENABLED))
        {
            g_init_wifi = 1;
        }
    }
    else
    {   // Turn off wifi
        g_init_wifi = -1;
    }
}


void wifi_reinit(void)
{
    g_init_wifi = 1;
}


static void init_wifi(void)
{
    g_wifi_connected = 0;
    g_status.evbStatus &= ~EVB_STATUS_WIFI_CONNECTED;
    g_status.evbStatus |= EVB_STATUS_WIFI_ENABLED;
    
    tstrWifiInitParam param;
    int8_t ret;

    /* Initialize the BSP. */
    nm_bsp_init();

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));
    
    /* Initialize socket address structure. */
    g_addr.sin_family = AF_INET;
    g_addr.sin_addr.s_addr = g_flashCfg->server[EVB_CFG_BITS_IDX_SERVER(g_flashCfg->bits)].ipAddr;
    g_addr.sin_port = _htons(g_flashCfg->server[EVB_CFG_BITS_IDX_SERVER(g_flashCfg->bits)].port);

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = m2m_wifi_reinit(&param);
    if (M2M_SUCCESS != ret)
    {
        //m2m_wifi_init call error!
        while (1) {}
    }

    /* Initialize socket module */
    init_tcp_socket();

    /* Connect to router. */
    char* wifi_ssid = (char*)(g_flashCfg->wifi[EVB_CFG_BITS_IDX_WIFI(g_flashCfg->bits)].ssid);
    char* wifi_psk  = (char*)(g_flashCfg->wifi[EVB_CFG_BITS_IDX_WIFI(g_flashCfg->bits)].psk);
    m2m_wifi_connect(wifi_ssid, strlen(wifi_ssid), WIFI_AUTH, wifi_psk, M2M_WIFI_CH_ALL);
}


static void deinit_wifi(void)
{
    deinit_tcp_socket();    
    m2m_wifi_deinit(NULLPTR);    

    /* Power down the BSP. */
    nm_bsp_deinit();    

    g_wifi_connected = 0;    
    g_status.evbStatus &= ~EVB_STATUS_WIFI_CONNECTED;
    g_status.evbStatus &= ~EVB_STATUS_WIFI_ENABLED;
}    


static void update_wifi_leds(rtos_task_t * task)
{
    if(g_wifi_connected == M2M_WIFI_CONNECTED)
    {
        // Inverted LED.  On shows hot spot connectivity and blink off shows data received.
        if(g_indicateWiFiRxMs > 0)
        {    
            g_indicateWiFiRxMs-=task->periodMs;
            LED_OFF(LED_WIFI_RXD_PIN);
        }
        else
        {
            LED_ON(LED_WIFI_RXD_PIN);
        }                
    }
    else
    {
        LED_OFF(LED_WIFI_RXD_PIN);
    }                    

    LED_OFF(LED_WIFI_TXD_PIN);
}


static void step_wifi(rtos_task_t * task)
{
    int8_t ret;

	/* Handle pending events from network controller. */
	m2m_wifi_handle_events(NULL);

	if (g_wifi_connected == M2M_WIFI_CONNECTED) 
	{
		/* Open client socket. */
		if (tcp_client_socket < 0)	// Open socket
		{
			if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
	        {
				//Failed to create TCP client socket error!
				return;
			}

			/* Connect server */
			ret = connect(tcp_client_socket, (struct sockaddr *)&g_addr, sizeof(struct sockaddr_in));

			if (ret < 0) 
	        {
				close(tcp_client_socket);
				tcp_client_socket = -1;
			}
		}
		else	// Socket open, process data
		{
			int len = 0;

			// Forward uINS Ser1 port data to WiFi socket
			if(g_flashCfg->cbf[EVB2_PORT_UINS1] & (1<<EVB2_PORT_WIFI))
			{
#if 0
	            len = xStreamBufferReceive(g_xStreamBufferWiFiTx, (void*)socketTxBuffer, STREAM_BUFFER_SIZE, 0);
#else
				BEGIN_CRITICAL_SECTION
				len = comRead(EVB2_PORT_UINS1, socketTxBuffer, WIFI_BUFFER_SIZE, LED_INS_RXD_PIN);
				END_CRITICAL_SECTION
#endif
				if(len > 0)	
	            {
					send(tcp_client_socket, socketTxBuffer, len, 1000);
		            LED_ON(LED_WIFI_TXD_PIN);
				}
			}
		}

		// Cause loop to sleep a while
		vTaskDelay(task->periodMs);
	}
	else
	{   // We are not connected, wait and try again
	    deinit_tcp_socket();
		vTaskDelay(500);
	}    
}


void vTaskWiFi(void *pvParameters)
{
    (void)pvParameters;
    rtos_task_t *task = &g_rtos.task[EVB_TASK_WIFI];

    LED_OFF(LED_WIFI_TXD_PIN);
    LED_OFF(LED_WIFI_RXD_PIN);

	while (1) 
    {        
        switch(g_init_wifi)
        {
        case 1:
            g_init_wifi = 0;
            deinit_wifi();
            init_wifi();
            break;
            
        case -1:    // Disabled wifi
            g_init_wifi = 0;
            deinit_wifi();
            break;
        }

        step_wifi(task);

        update_wifi_leds(task);     // Call right after step_wifi() (task delay)
	}
}
