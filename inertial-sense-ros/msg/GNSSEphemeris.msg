Header header
int32 sat 		# satellite number
int32 iode 		# IODE Issue of Data, Ephemeris (ephemeris version)
int32 iodc 		# IODC Issue of Data, Clock (clock version)
int32 sva 		# SV accuracy (URA index) IRN-IS-200H p.97            
int32 svh 		# SV health GPS/QZS (0:ok)            
int32 week 		# GPS/QZS: gps week, GAL: galileo week
int32 code 		# GPS/QZS: code on L2 * (00=Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid) * GAL/CMP: data sources
int32 flag 		# GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel) *  CMP: nav type
GTime toe 		# Toe
GTime toc 		# clock data reference time (s) (20.3.4.5)
GTime ttr 		# T_trans
float64 A 		# Semi-Major Axis m
float64 e 		# Eccentricity (no units) 
float64 i0 		# Inclination Angle at Reference Time (rad)
float64 OMG0 	# Longitude of Ascending Node of Orbit Plane at Weekly Epoch (rad)
float64 omg 	# Argument of Perigee (rad)
float64 M0 		# Mean Anomaly at Reference Time (rad)
float64 deln 	# Mean Motion Difference From Computed Value (rad)
float64 OMGd 	# Rate of Right Ascension (rad/s)
float64 idot 	# Rate of Inclination Angle (rad/s)
float64 crc 	# Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
float64 crs 	# Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (m)
float64 cuc 	# Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (rad) 
float64 cus 	# Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (rad)
float64 cic 	# Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (rad)
float64 cis 	# Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (rad)
float64 toes 	# Reference Time Ephemeris in week (s)
float64 fit 	# fit interval (h) (0: 4 hours, 1:greater than 4 hours)
float64 f0 		# SV clock parameters - af0
float64 f1 		# SV clock parameters - af1
float64 f2 		# SV clock parameters - af2
float64[4] tgd 	# group delay parameters: GPS/QZS:tgd[0]=TGD (IRN-IS-200H p.103) * GAL:tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1	* CMP :tgd[0]=BGD1,tgd[1]=BGD2
float64 Adot 	# Adot for CNAV
float64 ndot 	# ndot for CNAV