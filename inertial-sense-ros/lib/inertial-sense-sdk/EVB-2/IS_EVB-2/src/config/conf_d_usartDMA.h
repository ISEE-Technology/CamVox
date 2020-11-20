/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef CONF_D_USARTDMA_H_
#define CONF_D_USARTDMA_H_

//Define what port number goes to which hardware UART/USART and some configuration
//						UART/USART	DMA TX Channel				TX Buffer Size	DMA RX Channel				RX Buffer Size
#define PORT0_CONFIG  (	UART1,		DMA_CH_UART_UINS0_TX,		4096,			DMA_CH_UART_UINS0_RX,		4096 )
#define PORT1_CONFIG  (	UART0,		DMA_CH_UART_UINS1_TX,		4096,			DMA_CH_UART_UINS1_RX,		4096 )
#define PORT2_CONFIG  (	USART0,		DMA_CH_UART_XBEE_TX,		2048,			DMA_CH_UART_XBEE_RX,		2048 )
#define PORT3_CONFIG  (	UART2,		DMA_CH_UART_XRADIO_TX,		2048,			DMA_CH_UART_XRADIO_RX,		2048 )
#define PORT4_CONFIG  (	USART2,		DMA_CH_UART_WINC_BLE_TX,	2048,			DMA_CH_UART_WINC_BLE_RX,	2048 )
#define PORT5_CONFIG  (	UART3,		DMA_CH_UART_SP330_TX,		2048,			DMA_CH_UART_SP330_RX,		2048 )
#define PORT6_CONFIG  (	USART1,		DMA_CH_UART_GPIO_TTL_TX,	2048,			DMA_CH_UART_GPIO_TTL_RX,	2048 )
#define PORT7_CONFIG  (	0,			0,							2048,			0,							0	 )	// USB port

#define DEFAULT_BAUDRATE	115200

#define CONF_BOARD_USART_SPI            0   // Configure USART SPI pins
#define CONF_BOARD_USART_SPI_DATAREADY_ENABLE	0	//Enables the use of the data ready pin when in USART SPI mode

#define CONF_LOOPBACK_PORT              0   // DEBUGGING: Select port 0 or 1
#define ENABLE_COMM_LOOPBACK_DRIVER     0   // DEBUGGING: Forward serial port Rx onto Tx (Driver level)
#define ENABLE_COMM_LOOPBACK_RTOS       0   // DEBUGGING: Forward serial port Rx onto Tx (RTOS level).
#define MAX_NUMBER_SERIAL_PORTS         8

#define SER_INDICATE_TX()
#define SER_INDICATE_RX()

#endif /* CONF_D_USARTDMA_H_ */
