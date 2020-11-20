#pragma once

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdlib>

#include "InertialSense.h"

#include "ros/ros.h"
#include "ros/timer.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/JointState.h"
#include "inertial_sense/GPS.h"
#include "inertial_sense/GPSInfo.h"
#include "inertial_sense/PreIntIMU.h"
#include "inertial_sense/FirmwareUpdate.h"
#include "inertial_sense/refLLAUpdate.h"
#include "inertial_sense/RTKRel.h"
#include "inertial_sense/RTKInfo.h"
#include "inertial_sense/GNSSEphemeris.h"
#include "inertial_sense/GlonassEphemeris.h"
#include "inertial_sense/GNSSObservation.h"
#include "inertial_sense/GNSSObsVec.h"
#include "inertial_sense/INL2States.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <tf/transform_broadcaster.h>
//#include "geometry/xform.h"

# define GPS_UNIX_OFFSET 315964800 // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in seconds
# define LEAP_SECONDS 18 // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast)
# define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS)

#define SET_CALLBACK(DID, __type, __cb_fun, __periodmultiple) \
    IS_.BroadcastBinaryData(DID, __periodmultiple, \
    [this](InertialSense*i, p_data_t* data, int pHandle)\
    { \
       /* ROS_INFO("Got message %d", DID);*/\
       this->__cb_fun(reinterpret_cast<__type*>(data->buf));\
    })


class InertialSenseROS //: SerialListener
{
public:
  typedef enum
  {
    NMEA_GPGGA = 0x01,
    NMEA_GPGLL = 0x02,
    NMEA_GPGSA = 0x04,
    NMEA_GPRMC = 0x08,
    NMEA_SER0 = 0x01,
    NMEA_SER1 = 0x02
  } NMEA_message_config_t;
      
  InertialSenseROS();
  void callback(p_data_t* data);
  void update();

  void connect();
  void set_navigation_dt_ms();
  void configure_parameters();
  void configure_rtk();
  void configure_data_streams();
  void configure_ascii_output();
  void start_log();
  
  template<typename T> void set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset);
  template<typename T>  void set_flash_config(std::string param_name, uint32_t offset, T def) __attribute__ ((optimize(0)));
  void get_flash_config();
  void reset_device();
  void flash_config_callback(const nvm_flash_cfg_t* const msg);
  // Serial Port Configuration
  std::string port_;
  int baudrate_;
  bool initialized_;
  bool log_enabled_;

  std::string frame_id_;

  // ROS Stream handling
  typedef struct
  {
    bool enabled;
    ros::Publisher pub;
    ros::Publisher pub2;
  } ros_stream_t;

  ros_stream_t INS_;
  void INS1_callback(const ins_1_t* const msg);
  void INS2_callback(const ins_2_t* const msg);
//  void INS_variance_callback(const inl2_variance_t* const msg);

  ros_stream_t INL2_states_;
  void INL2_states_callback(const inl2_states_t* const msg);
  tf::TransformBroadcaster br;
  bool publishTf;
  tf::Transform transform;
  int LTCF;
  enum
  {
    NED,
    ENU
  }ltcf;

  ros_stream_t IMU_;
  void IMU_callback(const dual_imu_t* const msg);

  ros_stream_t GPS_;
  ros_stream_t GPS_obs_;
  ros_stream_t GPS_eph_;
  void GPS_pos_callback(const gps_pos_t* const msg);
  void GPS_vel_callback(const gps_vel_t* const msg);
  void GPS_raw_callback(const gps_raw_t* const msg);
  void GPS_obs_callback(const obsd_t * const msg, int nObs);
  void GPS_eph_callback(const eph_t* const msg);
  void GPS_geph_callback(const geph_t* const msg);
  void GPS_obs_bundle_timer_callback(const ros::TimerEvent& e);
  inertial_sense::GNSSObsVec obs_Vec_;
  ros::Timer obs_bundle_timer_;
  ros::Time last_obs_time_;

  ros_stream_t GPS_info_;
  void GPS_info_callback(const gps_sat_t* const msg);

  ros_stream_t mag_;
  void mag_callback(const magnetometer_t* const msg);

  ros_stream_t baro_;
  void baro_callback(const barometer_t* const msg);

  ros_stream_t dt_vel_;
  void preint_IMU_callback(const preintegrated_imu_t * const msg);
  
  ros::Publisher strobe_pub_;
  void strobe_in_time_callback(const strobe_in_time_t * const msg);

  ros_stream_t diagnostics_;
  void diagnostics_callback(const ros::TimerEvent& event);
  ros::Timer diagnostics_timer_;
  float diagnostic_ar_ratio_, diagnostic_differential_age_, diagnostic_heading_base_to_rover_;

  ros::ServiceServer mag_cal_srv_;
  ros::ServiceServer multi_mag_cal_srv_;
  ros::ServiceServer firmware_update_srv_;
  ros::ServiceServer refLLA_set_current_srv_;
  ros::ServiceServer refLLA_set_value_srv_;
  bool set_current_position_as_refLLA(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response & res);
  bool set_refLLA_to_value(inertial_sense::refLLAUpdate::Request &req, inertial_sense::refLLAUpdate::Response &res);
  bool perform_mag_cal_srv_callback(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool perform_multi_mag_cal_srv_callback(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool update_firmware_srv_callback(inertial_sense::FirmwareUpdate::Request & req, inertial_sense::FirmwareUpdate::Response & res);

  void publishGPS();

  typedef enum
  {
    RTK_NONE,
    RTK_ROVER,
    RTK_BASE,
    DUAL_GNSS
  } rtk_state_t;
  rtk_state_t RTK_state_ = RTK_NONE;
  ros_stream_t RTK_;
  void RTK_Misc_callback(const gps_rtk_misc_t* const msg);
  void RTK_Rel_callback(const gps_rtk_rel_t* const msg);

  
  /**
   * @brief ros_time_from_week_and_tow
   * Get current ROS time from week and tow
   * @param week Weeks since January 6th, 1980
   * @param timeOfWeek Time of week (since Sunday morning) in seconds, GMT
   * @return equivalent ros::Time
   */
  ros::Time ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek);
  
  /**
   * @brief ros_time_from_start_time
   * @param time - Time since boot up in seconds - Convert to GPS time of week by adding gps.towOffset
   * @return equivalent ros::Time
   */
  ros::Time ros_time_from_start_time(const double time);
  
  /**
   * @brief ros_time_from_tow
   * Get equivalent ros time from tow and internal week counter
   * @param tow Time of Week (seconds)
   * @return equivalent ros::Time
   */
  ros::Time ros_time_from_tow(const double tow);

  double tow_from_ros_time(const ros::Time& rt);
  ros::Time ros_time_from_gtime(const uint64_t sec, double subsec);

  double GPS_towOffset_ = 0; // The offset between GPS time-of-week and local time on the uINS
                             //  If this number is 0, then we have not yet got a fix
  uint64_t GPS_week_ = 0; // Week number to start of GPS_towOffset_ in GPS time
  // Time sync variables
  double INS_local_offset_ = 0.0; // Current estimate of the uINS start time in ROS time seconds
  bool got_first_message_ = false; // Flag to capture first uINS start time guess

  // Data to hold on to in between callbacks
  double lla_[3];
  double ecef_[3];
  sensor_msgs::Imu imu1_msg, imu2_msg;
  nav_msgs::Odometry odom_msg;
  inertial_sense::GPS gps_msg; 
  geometry_msgs::Vector3Stamped gps_velEcef;
  inertial_sense::GPSInfo gps_info_msg;
  inertial_sense::INL2States inl2_states_msg;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Connection to the uINS
  InertialSense IS_;
};