int32 sat 	# satellite number 
int32 iode 	# IODE (0-6 bit of tb field) 
int32 frq 	# satellite frequency number 
int32 svh 	# satellite health 
int32 sva 	# satellite accuracy 
int32 age 	# satellite age of operation 
GTime toe 	# epoch of epherides (gpst) 
GTime tof 	# message frame time (gpst) 
float64[3] pos 	# satellite position (ecef) (m) 
float64[3] vel 	# satellite velocity (ecef) (m/s) 
float64[3] acc 	# satellite acceleration (ecef) (m/s^2) 
float64 taun 	# SV clock bias (s) 
float64 gamn 	# relative freq bias 
float64 dtaun 	# delay between L1 and L2 (s) 