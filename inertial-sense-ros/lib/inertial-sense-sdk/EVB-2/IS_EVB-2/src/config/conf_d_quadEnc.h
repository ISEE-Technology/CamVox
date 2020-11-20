/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef CONF_D_QUADENC_H_
#define CONF_D_QUADENC_H_

#if 1

/***********  Left Wheel Encoder ***********/
/*** Position ***/
// EVB H8-9:  GPIO5 - QDEC2-A - TIOA6 - PC5
// EVB H8-10: GPIO6 - QDEC2-B - TIOB6 - PC6
#define QE0_POS				TC2
#define ID_QE0_POS			ID_TC6

#define PIN_QE0_POS_PHA		(PIO_PC5_IDX)		//TC2 TIOA6
#define PIN_QE0_POS_PHA_MUX	(IOPORT_MODE_MUX_B)

#define PIN_QE0_POS_PHB		(PIO_PC6_IDX)		//TC2 TIOB6
#define PIN_QE0_POS_PHB_MUX	(IOPORT_MODE_MUX_B)

/*** Speed ***/
#if 1
    // EVB H8-9: GPIO5 - TIOA5 - PC29
	#define TCCAP0_SPD			TC1
	#define TCCAP0_SPD_CHANNEL	2
	#define ID_TCCAP0_SPD		ID_TC5
	
	#define PIN_TCCAP0_SPD		(PIO_PC29_IDX)
	#define PIN_TCCAP0_SPD_MUX	(IOPORT_MODE_MUX_B)
	
	#define TCCAP0_SPD_Handler  TC5_Handler
	#define TCCAP0_SPD_IRQn     TC5_IRQn
#else
	// EVB H8-11: GPIO7 - TIOA10 - PE3
	#define TCCAP0_SPD			TC3
	#define TCCAP0_SPD_CHANNEL	1
	#define ID_TCCAP0_SPD		ID_TC10

	#define PIN_TCCAP0_SPD		(PIO_PE3_IDX)
	#define PIN_TCCAP0_SPD_MUX	(IOPORT_MODE_MUX_B)

	#define TCCAP0_SPD_Handler  TC10_Handler
	#define TCCAP0_SPD_IRQn     TC10_IRQn
#endif

/***********  Right Wheel Encoder ***********/
/*** Position ***/
// EVB H8-13: GPIO9  - QDEC3-A - TIOA9 - PE0
// EVB H8-14: GPIO10 - QDEC3-B - TIOB9 - PE1
#define QE1_POS				TC3
#define ID_QE1_POS			ID_TC9

#define PIN_QE1_POS_PHA		(PIO_PE0_IDX)		//TC3 TIOA9
#define PIN_QE1_POS_PHA_MUX	(IOPORT_MODE_MUX_B)

#define PIN_QE1_POS_PHB		(PIO_PE1_IDX)		//TC3 TIOB9
#define PIN_QE1_POS_PHB_MUX	(IOPORT_MODE_MUX_B)

/*** Speed ***/
// EVB H8-7: GPIO3 - TIOA0 - PA0
#define TCCAP1_SPD			TC0
#define TCCAP1_SPD_CHANNEL	0
#define ID_TCCAP1_SPD		ID_TC0

#define PIN_TCCAP1_SPD		(PIO_PA0_IDX)
#define PIN_TCCAP1_SPD_MUX	(IOPORT_MODE_MUX_B)

#define TCCAP1_SPD_Handler  TC0_Handler
#define TCCAP1_SPD_IRQn     TC0_IRQn

#else
/*********** Use for testing with EVB-2 rev 2.0.0 *********/

	/*** Position ***/
	// EVB H8-7: GPIO3 - QDEC0-A - TIOA0 - PA0
	// EVB H8-8: GPIO4 - QDEC0-B - TIOB0 - PA1
	#define QE0_POS				TC0
	#define ID_QE0_POS			ID_TC0

	#define PIN_QE0_POS_PHA		(PIO_PA0_IDX)		//TC0 TIOA0
	#define PIN_QE0_POS_PHA_MUX	(IOPORT_MODE_MUX_B)

	#define PIN_QE0_POS_PHB		(PIO_PA1_IDX)		//TC0 TIOB0
	#define PIN_QE0_POS_PHB_MUX	(IOPORT_MODE_MUX_B)

	/*** Speed ***/
	// EVB H8-7: GPIO3 - QDEC0-A - TIOA0 - PA0
	#define TCCAP0_SPD			TC0
	#define TCCAP0_SPD_CHANNEL	0
	#define ID_TCCAP0_SPD		ID_TC0
	
	#define PIN_TCCAP0_SPD		(PIO_PA0_IDX)
	#define PIN_TCCAP0_SPD_MUX	(IOPORT_MODE_MUX_B)
	
	#define TCCAP0_SPD_Handler  TC0_Handler
	#define TCCAP0_SPD_IRQn     TC0_IRQn

	/***********  Right Wheel Encoder ***********/
	/*** Position ***/
	// EVB H8-13: GPIO9  - QDEC3-A - TIOA9 - PE0
	// EVB H8-14: GPIO10 - QDEC3-B - TIOB9 - PE1
	#define QE1_POS				TC3
	#define ID_QE1_POS			ID_TC9

	#define PIN_QE1_POS_PHA		(PIO_PE0_IDX)		//TC3 TIOA9
	#define PIN_QE1_POS_PHA_MUX	(IOPORT_MODE_MUX_B)

	#define PIN_QE1_POS_PHB		(PIO_PE1_IDX)		//TC3 TIOB9
	#define PIN_QE1_POS_PHB_MUX	(IOPORT_MODE_MUX_B)

	/*** Speed ***/
	// EVB H8-13: GPIO1 - TIOA9 - PE0
	#define TCCAP1_SPD			TC3
	#define TCCAP1_SPD_CHANNEL	0
	#define ID_TCCAP1_SPD		ID_TC9
	
	#define PIN_TCCAP1_SPD		(PIO_PE0_IDX)
	#define PIN_TCCAP1_SPD_MUX	(IOPORT_MODE_MUX_B)
	
	#define TCCAP1_SPD_Handler  TC9_Handler
	#define TCCAP1_SPD_IRQn     TC9_IRQn
	
#endif

#endif /* CONF_D_QUADENC_H_ */