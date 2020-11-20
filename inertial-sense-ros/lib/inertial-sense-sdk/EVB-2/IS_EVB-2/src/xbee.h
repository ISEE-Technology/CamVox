/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef __XBEE_H__
#define __XBEE_H__

#include "../../../src/ISComm.h"

#ifdef __cplusplus
extern "C" {
#endif

void xbee_init(void);

void xbee_step(is_comm_instance_t *comm);

int xbee_runtime_mode(void);

// static uint8_t xbee_packetize(unsigned char *raw_buf, uint8_t len_raw_buf, unsigned char *pack_buf);
// static uint8_t xbee_depacketize(unsigned char *pack_buf, unsigned char *raw_buf);

#ifdef __cplusplus
}
#endif

#endif /* __XBEE_H__ */