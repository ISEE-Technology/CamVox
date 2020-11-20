Header header                       # GPS time of week (since Sunday morning) in seconds
geometry_msgs/Quaternion quatEcef   # Quaternion body rotation with respect to ECEF
geometry_msgs/Vector3 velEcef       # (m/s) Velocity in ECEF frame
geometry_msgs/Vector3 posEcef       # (m) Position in ECEF frame
geometry_msgs/Vector3 gyroBias      # (rad/s) Gyro bias
geometry_msgs/Vector3 accelBias     # (m/s^2) Accelerometer bias
float32 baroBias                    # (m) Barometer bias
float32 magDec                      # (rad) Magnetic declination
float32 magInc                      # (rad) Magnetic inclination
