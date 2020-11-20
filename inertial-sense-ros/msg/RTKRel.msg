Header header
float32 differential_age 				# Age of differential (seconds)
float32 ar_ratio 						# Ambiguity resolution ratio factor for validation
geometry_msgs/Vector3 vector_base_to_rover 	# Vector to base (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1
float32 distance_base_to_rover 				# Distance to Base (m)
float32 heading_base_to_rover 				# Angle from north to vectorToBase in local tangent plane. (rad)