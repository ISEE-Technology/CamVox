#include <math.h>

#include "misc/sig_gen.h"

#define PI			3.14159265358979
#define TWOPI		6.28318530717959

//static float g_freqHz, g_amplitude, g_periodSec, g_time;


//void init_sig_gen( float loopPeriodSec, float freqHz, float amplitude )
//{
	//g_freqHz = freqHz;
	//g_amplitude = amplitude;
	//g_periodSec = loopPeriodSec;
	//g_time = 0;
//}


float step_sinwave( float *sig_gen, float freqHz, float amplitude, float periodSec )
{
	*sig_gen += freqHz * periodSec * TWOPI;
	
	// Unwrap Angle
	if( *sig_gen > PI )	
		*sig_gen -= TWOPI;
	
	return amplitude * sinf( *sig_gen );
}
