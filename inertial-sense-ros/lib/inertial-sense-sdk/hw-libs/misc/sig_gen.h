#ifndef SIG_GEN_H_
#define SIG_GEN_H_


/*!
 * \brief Initialize signal generator
 *
 * param float loopPeriodSec	Timestep interval in seconds
 * param float freqHz			Frequency of output sin wave
 * param float amplitude		Amplitude (peak to center/mean) of output sin wave
 */
//void init_sig_gen( float loopPeriodSec, float freqHz, float amplitude );

/*!
 * \brief Step the signal generator output
 */
//float step_sig_gen( );
float step_sinwave( float *sig_gen, float freqHz, float amplitude, float periodSec );

#endif /* SIG_GEN_H_ */
