std_msgs/Header header
GTime time              # time of observation
uint8 sat 		# satellite number
uint8 rcv 		# receiver number
uint8 SNR 		# Signal Strength (0.25 dBHz)
uint8 LLI 		# Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)
uint8 code 		# code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )
uint8 qualL 	# Estimated carrier phase measurement standard deviation (0.004 cycles)
uint8 qualP 	# Estimated pseudorange measurement standard deviation (0.01 m)
float64 L      	# observation data carrier-phase (cycle)
float64 P      	# observation data pseudorange (m)
float32 D      	# observation data doppler frequency (0.002 Hz)