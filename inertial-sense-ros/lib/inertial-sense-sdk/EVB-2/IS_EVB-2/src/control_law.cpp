/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "globals.h"
#include "arm_math.h"
#include "drivers/d_quadEnc.h"
#include "control_law.h"


void init_control(void)
{
#ifdef CONF_BOARD_QUAD_ENCODER
    if(g_flashCfg->bits&EVB_CFG_BITS_ENABLE_WHEEL_ENCODER)
    {   
	    quadEncInit();
    }
#endif
}


void velocity_control(is_comm_instance_t &comm)
{
#ifdef CONF_BOARD_QUAD_ENCODER           // Encoder Rx   =======================================================

    if(!(g_flashCfg->bits&EVB_CFG_BITS_ENABLE_WHEEL_ENCODER))
    {   // Wheel encoders disabled
        return;
    }

	int chL, chR;
	bool dirL, dirR;
	int n=0;
	int speedL, speedR;
	static int encoderSendTimeMs=0;
	static wheel_encoder_t wheelEncoderLast = {0};

	if(abs(g_comm_time_ms-encoderSendTimeMs)>=20)
	{	// Send data at 50Hz
		encoderSendTimeMs = g_comm_time_ms;
		
		// Call read encoders
		quadEncReadPositionAll(&chL, &dirL, &chR, &dirR);
		quadEncReadSpeedAll((uint32_t*)&speedL, (uint32_t*)&speedR);
		g_wheelEncoder.timeOfWeek = time_seclf();

		// Set velocity direction
		if(dirL)
		{
			speedL = -speedL;
		}
		if(dirR)
		{
			speedR = -speedR;
		}
				
		// Convert encoder ticks to radians.
		g_wheelEncoder.theta_l = (chL * g_flashCfg->encoderTickToWheelRad)/2; /*Division by 2 to account for 4x encoding*/
		g_wheelEncoder.theta_r = (chR * g_flashCfg->encoderTickToWheelRad)/2;

        // Convert TC pulse period to rad/sec.  20us per TC LSB x 2 (measure rising to rising edge).
        if(speedL)
        {
            g_wheelEncoder.omega_l = g_flashCfg->encoderTickToWheelRad / (0.00002f * (float)speedL);
        }
        else
        {
            g_wheelEncoder.omega_l = 0.0f;
        }
        if(speedR)
		{
            g_wheelEncoder.omega_r = g_flashCfg->encoderTickToWheelRad / (0.00002f * (float)speedR);
        }
        else
        {
            g_wheelEncoder.omega_r = 0.0f;
        }
		
		// Encoder Wrap count (currently counting revolutions) 
//		g_wheelEncoder.wrap_count_l = g_wheelEncoder.theta_l / (2*PI);
//		g_wheelEncoder.wrap_count_r = g_wheelEncoder.theta_r / (2*PI);	
				
		n = is_comm_data(&comm, DID_WHEEL_ENCODER, 0, sizeof(wheel_encoder_t), (void*)&(g_wheelEncoder));
		comWrite(EVB2_PORT_UINS0, comm.buf.start, n, LED_INS_TXD_PIN);

		// Update history
		wheelEncoderLast = g_wheelEncoder;
	}
	
#endif
}
