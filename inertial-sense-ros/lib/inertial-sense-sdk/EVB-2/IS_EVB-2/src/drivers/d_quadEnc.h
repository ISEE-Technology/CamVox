/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef D_QUADENC_H_
#define D_QUADENC_H_

#define QE_LEFT_WHEEL	0
#define QE_RIGHT_WHEEL	1

#ifdef __cplusplus
	extern "C" {
#endif
	
/**
 * \brief Initialize quadrature encoder driver.
 */
void quadEncInit(void);

/**
 * \brief Reads the current position of the encoders.
 */
void quadEncReadPositionAll(int *pos0, bool *dir0, int *pos1, bool *dir1);
void quadEncReadSpeedAll(uint32_t *speed0, uint32_t *speed1);

void test_quad_encoders(void);

#ifdef __cplusplus
	}
#endif

#endif /* D_QUADENC_H_ */