/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef CAN_H_
#define CAN_H_

#include "compiler.h"
#include "mcan.h"

#ifdef __cplusplus
extern "C" {
#endif

void CAN_init(void);
bool mcan_send_message(uint32_t id_value, uint8_t *data, uint32_t data_length);
int mcan_set_rx_filter(uint32_t id_value);
uint8_t mcan_receive_message(uint32_t *id_value, uint8_t *data);
void CAN_set_baudrate(uint32_t baudrate);
	
void mcan_test_master(void);
void mcan_test_slave(void);
	
#ifdef __cplusplus
}
#endif

#endif /* CAN_H_ */
