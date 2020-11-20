#include "inertial_sense.h"
#include <chrono>
#include <stddef.h>
#include <unistd.h>
#include <tf/tf.h>
#include <ros/console.h>

InertialSenseROS::InertialSenseROS() :
  nh_(), nh_private_("~"), initialized_(false)
{
  connect();
  set_navigation_dt_ms();

  /// Start Up ROS service servers
  refLLA_set_current_srv_ = nh_.advertiseService("set_refLLA_current", &InertialSenseROS::set_current_position_as_refLLA, this);
  refLLA_set_value_srv_ = nh_.advertiseService("set_refLLA_value", &InertialSenseROS::set_refLLA_to_value, this);
  mag_cal_srv_ = nh_.advertiseService("single_axis_mag_cal", &InertialSenseROS::perform_mag_cal_srv_callback, this);
  multi_mag_cal_srv_ = nh_.advertiseService("multi_axis_mag_cal", &InertialSenseROS::perform_multi_mag_cal_srv_callback, this);
  firmware_update_srv_ = nh_.advertiseService("firmware_update", &InertialSenseROS::update_firmware_srv_callback, this);

  configure_parameters();
  configure_rtk();
  configure_data_streams();

  nh_private_.param<bool>("enable_log", log_enabled_, false);
  if (log_enabled_)
  {
    start_log();//start log should always happen last, does not all stop all message streams.
  }


//  configure_ascii_output(); //does not work right now

  initialized_ = true;
}


void InertialSenseROS::configure_data_streams()
{
  SET_CALLBACK(DID_GPS1_POS, gps_pos_t, GPS_pos_callback,1); // we always need GPS for Fix status
  SET_CALLBACK(DID_GPS1_VEL, gps_vel_t, GPS_vel_callback,1); // we always need GPS for Fix status
  SET_CALLBACK(DID_STROBE_IN_TIME, strobe_in_time_t, strobe_in_time_callback,1); // we always want the strobe
  

  nh_private_.param<bool>("stream_INS", INS_.enabled, true);
  if (INS_.enabled)
  {
    INS_.pub = nh_.advertise<nav_msgs::Odometry>("ins", 1);
    SET_CALLBACK(DID_INS_1, ins_1_t, INS1_callback, 5);
    SET_CALLBACK(DID_INS_2, ins_2_t, INS2_callback,5);
    SET_CALLBACK(DID_DUAL_IMU, dual_imu_t, IMU_callback,1);
//    SET_CALLBACK(DID_INL2_VARIANCE, nav_dt_ms, inl2_variance_t, INS_variance_callback);
  }
  nh_private_.param<bool>("publishTf", publishTf, true);
  nh_private_.param<int>("LTCF", LTCF, NED);
  // Set up the IMU ROS stream
  nh_private_.param<bool>("stream_IMU", IMU_.enabled, true);

  //std::cout << "\n\n\n\n\n\n\n\n\n\n stream_GPS: " << GPS_.enabled << "\n\n\n\n\n\n\n\n\n\n\n";
  if (IMU_.enabled)
  {
    IMU_.pub = nh_.advertise<sensor_msgs::Imu>("imu", 1);
    SET_CALLBACK(DID_INS_1, ins_1_t, INS1_callback,1);
    SET_CALLBACK(DID_INS_2, ins_2_t, INS2_callback,1);
    SET_CALLBACK(DID_DUAL_IMU, dual_imu_t, IMU_callback,1);
  }

  // Set up the IMU bias ROS stream
  nh_private_.param<bool>("stream_INL2_states", INL2_states_.enabled, false);
  if (INL2_states_.enabled)
  {
    INL2_states_.pub = nh_.advertise<inertial_sense::INL2States>("inl2_states", 1);
    SET_CALLBACK(DID_INL2_STATES, inl2_states_t, INL2_states_callback,1);
  }

  // Set up the GPS ROS stream - we always need GPS information for time sync, just don't always need to publish it
  nh_private_.param<bool>("stream_GPS", GPS_.enabled, true);
  if (GPS_.enabled)
      GPS_.pub = nh_.advertise<inertial_sense::GPS>("gps", 1);

  nh_private_.param<bool>("stream_GPS_raw", GPS_obs_.enabled, false);
  nh_private_.param<bool>("stream_GPS_raw", GPS_eph_.enabled, false);
  if (GPS_obs_.enabled)
  {
    GPS_obs_.pub = nh_.advertise<inertial_sense::GNSSObsVec>("gps/obs", 50);
    GPS_eph_.pub = nh_.advertise<inertial_sense::GNSSEphemeris>("gps/eph", 50);
    GPS_eph_.pub2 = nh_.advertise<inertial_sense::GlonassEphemeris>("gps/geph", 50);
    SET_CALLBACK(DID_GPS1_RAW, gps_raw_t, GPS_raw_callback,1);
    SET_CALLBACK(DID_GPS_BASE_RAW, gps_raw_t, GPS_raw_callback,1);
    SET_CALLBACK(DID_GPS2_RAW, gps_raw_t, GPS_raw_callback,1);
    obs_bundle_timer_ = nh_.createTimer(ros::Duration(0.001), InertialSenseROS::GPS_obs_bundle_timer_callback, this);
  }

  // Set up the GPS info ROS stream
  nh_private_.param<bool>("stream_GPS_info", GPS_info_.enabled, false);
  if (GPS_info_.enabled)
  {
    GPS_info_.pub = nh_.advertise<inertial_sense::GPSInfo>("gps/info", 1);
    SET_CALLBACK(DID_GPS1_SAT, gps_sat_t, GPS_info_callback,1);
  }

  // Set up the magnetometer ROS stream
  nh_private_.param<bool>("stream_mag", mag_.enabled, false);
  if (mag_.enabled)
  {
    mag_.pub = nh_.advertise<sensor_msgs::MagneticField>("mag", 1);
    //    mag_.pub2 = nh_.advertise<sensor_msgs::MagneticField>("mag2", 1);
    SET_CALLBACK(DID_MAGNETOMETER_1, magnetometer_t, mag_callback,1);
  }

  // Set up the barometer ROS stream
  nh_private_.param<bool>("stream_baro", baro_.enabled, false);
  if (baro_.enabled)
  {
    baro_.pub = nh_.advertise<sensor_msgs::FluidPressure>("baro", 1);
    SET_CALLBACK(DID_BAROMETER, barometer_t, baro_callback,1);
  }

  // Set up the preintegrated IMU (coning and sculling integral) ROS stream
  nh_private_.param<bool>("stream_preint_IMU", dt_vel_.enabled, false);
  if (dt_vel_.enabled)
  {
    dt_vel_.pub = nh_.advertise<inertial_sense::PreIntIMU>("preint_imu", 1);
    SET_CALLBACK(DID_PREINTEGRATED_IMU, preintegrated_imu_t, preint_IMU_callback,1);
  }

  // Set up ROS dianostics for rqt_robot_monitor
  nh_private_.param<bool>("stream_diagnostics", diagnostics_.enabled, true);
  if (diagnostics_.enabled)
  {
    diagnostics_.pub = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    diagnostics_timer_ = nh_.createTimer(ros::Duration(0.5), &InertialSenseROS::diagnostics_callback , this); // 2 Hz
  }
}

void InertialSenseROS::start_log()
{
  std::string filename = cISLogger::CreateCurrentTimestamp();
  ROS_INFO_STREAM("Creating log in " << filename << " folder");
  IS_.SetLoggerEnabled(true, filename, cISLogger::LOGTYPE_DAT, RMC_PRESET_PPD_ROBOT);
}

void InertialSenseROS::configure_ascii_output()
{
  //  int NMEA_rate = nh_private_.param<int>("NMEA_rate", 0);
  //  int NMEA_message_configuration = nh_private_.param<int>("NMEA_configuration", 0x00);
  //  int NMEA_message_ports = nh_private_.param<int>("NMEA_ports", 0x00);
  //  ascii_msgs_t msgs = {};
  //  msgs.options = (NMEA_message_ports & NMEA_SER0) ? RMC_OPTIONS_PORT_SER0 : 0; // output on serial 0
  //  msgs.options |= (NMEA_message_ports & NMEA_SER1) ? RMC_OPTIONS_PORT_SER1 : 0; // output on serial 1
  //  msgs.gpgga = (NMEA_message_configuration & NMEA_GPGGA) ? NMEA_rate : 0;
  //  msgs.gpgll = (NMEA_message_configuration & NMEA_GPGLL) ? NMEA_rate : 0;
  //  msgs.gpgsa = (NMEA_message_configuration & NMEA_GPGSA) ? NMEA_rate : 0;
  //  msgs.gprmc = (NMEA_message_configuration & NMEA_GPRMC) ? NMEA_rate : 0;
  //  IS_.SendData(DID_ASCII_BCAST_PERIOD, (uint8_t*)(&msgs), sizeof(ascii_msgs_t), 0);

}

void InertialSenseROS::connect()
{
  nh_private_.param<std::string>("port", port_, "/dev/ttyUSB0");
  nh_private_.param<int>("baudrate", baudrate_, 921600);
  nh_private_.param<std::string>("frame_id", frame_id_, "body");

  /// Connect to the uINS
  ROS_INFO("Connecting to serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
  if (! IS_.Open(port_.c_str(), baudrate_))
  {
    ROS_FATAL("inertialsense: Unable to open serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
    exit(0);
  }
  else
  {
    // Print if Successful
    ROS_INFO("Connected to uINS %d on \"%s\", at %d baud", IS_.GetDeviceInfo().serialNumber, port_.c_str(), baudrate_);
  }
}

void InertialSenseROS::set_navigation_dt_ms()
{
  // Make sure the navigation rate is right, if it's not, then we need to change and reset it.
  int nav_dt_ms = IS_.GetFlashConfig().startupNavDtMs;
  if (nh_private_.getParam("navigation_dt_ms", nav_dt_ms))
  {
    if (nav_dt_ms != IS_.GetFlashConfig().startupNavDtMs)
    {
      uint32_t data = nav_dt_ms;
      IS_.SendData(DID_FLASH_CONFIG, (uint8_t*)(&data), sizeof(uint32_t), offsetof(nvm_flash_cfg_t, startupNavDtMs));
      ROS_INFO("navigation rate change from %dms to %dms, resetting uINS to make change", IS_.GetFlashConfig().startupNavDtMs, nav_dt_ms);
      sleep(3);
      reset_device();
    }
  }
}

void InertialSenseROS::configure_parameters()
{
  set_vector_flash_config<float>("INS_rpy_radians", 3, offsetof(nvm_flash_cfg_t, insRotation));
  set_vector_flash_config<float>("INS_xyz", 3, offsetof(nvm_flash_cfg_t, insOffset));
  set_vector_flash_config<float>("GPS_ant1_xyz", 3, offsetof(nvm_flash_cfg_t, gps1AntOffset));
  set_vector_flash_config<float>("GPS_ant2_xyz", 3, offsetof(nvm_flash_cfg_t, gps2AntOffset));
  set_vector_flash_config<double>("GPS_ref_lla", 3, offsetof(nvm_flash_cfg_t, refLla));

  set_flash_config<float>("inclination", offsetof(nvm_flash_cfg_t, magInclination), 0.0f);
  set_flash_config<float>("declination", offsetof(nvm_flash_cfg_t, magDeclination), 0.0f);
  set_flash_config<int>("dynamic_model", offsetof(nvm_flash_cfg_t, insDynModel), 8);
  set_flash_config<int>("ser1_baud_rate", offsetof(nvm_flash_cfg_t, ser1BaudRate), 921600);
}

void InertialSenseROS::configure_rtk()
{
  bool RTK_rover, RTK_rover_radio_enable, RTK_base, dual_GNSS;
  std::string gps_type;
  nh_private_.param<std::string>("gps_type", gps_type, "M8");
  nh_private_.param<bool>("RTK_rover", RTK_rover, false);
  nh_private_.param<bool>("RTK_rover_radio_enable", RTK_rover_radio_enable, false);
  nh_private_.param<bool>("RTK_base", RTK_base, false);
  nh_private_.param<bool>("dual_GNSS", dual_GNSS, false);
  nh_private_.param<bool>("dual_GNSS", dual_GNSS, false);
  std::string RTK_server_IP, RTK_correction_type;
  int RTK_server_port;
  nh_private_.param<std::string>("RTK_server_IP", RTK_server_IP, "127.0.0.1");
  nh_private_.param<int>("RTK_server_port", RTK_server_port, 7777);
  nh_private_.param<std::string>("RTK_correction_type", RTK_correction_type, "UBLOX");
  ROS_ERROR_COND(RTK_rover && RTK_base, "unable to configure uINS to be both RTK rover and base - default to rover");
  ROS_ERROR_COND(RTK_rover && dual_GNSS, "unable to configure uINS to be both RTK rover as dual GNSS - default to dual GNSS");

  uint32_t RTKCfgBits = 0;
  if (dual_GNSS)
  {
    RTK_rover = false;
    ROS_INFO("InertialSense: Configured as dual GNSS (compassing)");
    RTK_state_ = DUAL_GNSS;
    RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING;
    SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback,1);
    SET_CALLBACK(DID_GPS2_RTK_CMP_REL, gps_rtk_rel_t, RTK_Rel_callback,1);
    RTK_.enabled = true;
    RTK_.pub = nh_.advertise<inertial_sense::RTKInfo>("RTK/info", 10);
    RTK_.pub2 = nh_.advertise<inertial_sense::RTKRel>("RTK/rel", 10);
  }

  if (RTK_rover_radio_enable)
  {
    RTK_base = false;
    ROS_INFO("InertialSense: Configured as RTK Rover with radio enabled");
    RTK_state_ = RTK_ROVER;
    RTKCfgBits |= (gps_type=="F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_F9P : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);

    SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback,1);
    SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback,1);
    RTK_.enabled = true;
    RTK_.pub = nh_.advertise<inertial_sense::RTKInfo>("RTK/info", 10);
    RTK_.pub2 = nh_.advertise<inertial_sense::RTKRel>("RTK/rel", 10);
  }
  else if (RTK_rover)
  {
    RTK_base = false;
    std::string RTK_connection =  RTK_correction_type + ":" + RTK_server_IP + ":" + std::to_string(RTK_server_port);
    ROS_INFO("InertialSense: Configured as RTK Rover");
    RTK_state_ = RTK_ROVER;
    RTKCfgBits |= (gps_type=="F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_F9P : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);

    if (IS_.OpenServerConnection(RTK_connection))
      ROS_INFO_STREAM("Successfully connected to " << RTK_connection << " RTK server");
    else
      ROS_ERROR_STREAM("Failed to connect to base server at " << RTK_connection);

    SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback,1);
    SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback,1);
    RTK_.enabled = true;
    RTK_.pub = nh_.advertise<inertial_sense::RTKInfo>("RTK/info", 10);
    RTK_.pub2 = nh_.advertise<inertial_sense::RTKRel>("RTK/rel", 10);
  }
  else if (RTK_base)
  {
    std::string RTK_connection =  RTK_server_IP + ":" + std::to_string(RTK_server_port);
    RTK_.enabled = true;
    ROS_INFO("InertialSense: Configured as RTK Base");
    RTK_state_ = RTK_BASE;
    RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0;

    if (IS_.CreateHost(RTK_connection))
    {
      ROS_INFO_STREAM("Successfully created " << RTK_connection << " as RTK server");
      initialized_ = true;
      return;
    }
    else
      ROS_ERROR_STREAM("Failed to create base server at " << RTK_connection);
  }
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&RTKCfgBits), sizeof(RTKCfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
}

template <typename T>
void InertialSenseROS::set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset){
  std::vector<double> tmp(size,0);
  T v[size];
  if (nh_private_.hasParam(param_name))
    nh_private_.getParam(param_name, tmp);
  for (int i = 0; i < size; i++)
  {
    v[i] = tmp[i];
  }
  
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&v), sizeof(v), offset);
  IS_.GetFlashConfig() = IS_.GetFlashConfig();
}

template <typename T>
void InertialSenseROS::set_flash_config(std::string param_name, uint32_t offset, T def)
{
  T tmp;
  nh_private_.param<T>(param_name, tmp, def);
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&tmp), sizeof(T), offset);
}

void InertialSenseROS::INS1_callback(const ins_1_t * const msg)
{
  odom_msg.header.frame_id = frame_id_;
  if (LTCF == NED)
  {
    odom_msg.pose.pose.position.x = msg->ned[0];
    odom_msg.pose.pose.position.y = msg->ned[1];
    odom_msg.pose.pose.position.z = msg->ned[2];
  }
  else if (LTCF == ENU)
  {
    odom_msg.pose.pose.position.x = msg->ned[1];
    odom_msg.pose.pose.position.y = msg->ned[0];
    odom_msg.pose.pose.position.z = -msg->ned[2];
  }

}

//void InertialSenseROS::INS_variance_callback(const inl2_variance_t * const msg)
//{
//  // We have to convert NED velocity covariance into body-fixed
//  tf::Matrix3x3 cov_vel_NED;
//  cov_vel_NED.setValue(msg->PvelNED[0], 0, 0, 0, msg->PvelNED[1], 0, 0, 0, msg->PvelNED[2]);
//  tf::Quaternion att;
//  tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, att);
//  tf::Matrix3x3 R_NED_B(att);
//  tf::Matrix3x3 cov_vel_B = R_NED_B.transposeTimes(cov_vel_NED * R_NED_B);

//  // Populate Covariance Matrix
//  for (int i = 0; i < 3; i++)
//  {
//    // Position and velocity covariance is only valid if in NAV mode (with GPS)
//    if (insStatus_ & INS_STATUS_NAV_MODE)
//    {
//      odom_msg.pose.covariance[7*i] = msg->PxyzNED[i];
//      for (int j = 0; j < 3; j++)
//        odom_msg.twist.covariance[6*i+j] = cov_vel_B[i][j];
//    }
//    else
//    {
//      odom_msg.pose.covariance[7*i] = 0;
//      odom_msg.twist.covariance[7*i] = 0;
//    }
//    odom_msg.pose.covariance[7*(i+3)] = msg->PattNED[i];
//    odom_msg.twist.covariance[7*(i+3)] = msg->PWBias[i];
//  }
//}


void InertialSenseROS::INS2_callback(const ins_2_t * const msg)
{
  if (!(msg->hdwStatus&HDW_STATUS_GPS_TIME_OF_WEEK_VALID))
  { // Don't run if msg->timeOfWeek is not valid
    return;
  }

  odom_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
  odom_msg.header.frame_id = frame_id_;

  odom_msg.pose.pose.orientation.w = msg->qn2b[0];
  if (LTCF == NED)
  {
    odom_msg.pose.pose.orientation.x = msg->qn2b[1];
    odom_msg.pose.pose.orientation.y = msg->qn2b[2];
    odom_msg.pose.pose.orientation.z = msg->qn2b[3];
  }
  else if (LTCF == ENU)
  {
    odom_msg.pose.pose.orientation.x = msg->qn2b[2];
    odom_msg.pose.pose.orientation.y = msg->qn2b[1];
    odom_msg.pose.pose.orientation.z = -msg->qn2b[3];
  }

  odom_msg.twist.twist.linear.x = msg->uvw[0];
  odom_msg.twist.twist.linear.y = msg->uvw[1];
  odom_msg.twist.twist.linear.z = msg->uvw[2];

  lla_[0] = msg->lla[0];
  lla_[1] = msg->lla[1];
  lla_[2] = msg->lla[2];

  odom_msg.pose.covariance[0] = lla_[0];
  odom_msg.pose.covariance[1] = lla_[1];
  odom_msg.pose.covariance[2] = lla_[2];
  odom_msg.pose.covariance[3] = ecef_[0];
  odom_msg.pose.covariance[4] = ecef_[1];
  odom_msg.pose.covariance[5] = ecef_[2];
  odom_msg.pose.covariance[6] = LTCF;         //Defined in inertial_sense.h: enum InertialSenseROS::ltcf

  odom_msg.twist.twist.angular.x = imu1_msg.angular_velocity.x;
  odom_msg.twist.twist.angular.y = imu1_msg.angular_velocity.y;
  odom_msg.twist.twist.angular.z = imu1_msg.angular_velocity.z;

  if (publishTf)
  {
    // Calculate the TF from the pose...
		transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, q);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ins", "base_link"));
  }

  if (INS_.enabled)
    INS_.pub.publish(odom_msg);
}


void InertialSenseROS::INL2_states_callback(const inl2_states_t* const msg)
{
  inl2_states_msg.header.stamp = ros_time_from_tow(msg->timeOfWeek);
  inl2_states_msg.header.frame_id = frame_id_;

  inl2_states_msg.quatEcef.w = msg->qe2b[0];
  inl2_states_msg.quatEcef.x = msg->qe2b[1];
  inl2_states_msg.quatEcef.y = msg->qe2b[2];
  inl2_states_msg.quatEcef.z = msg->qe2b[3];

  inl2_states_msg.velEcef.x = msg->ve[0];
  inl2_states_msg.velEcef.y = msg->ve[1];
  inl2_states_msg.velEcef.z = msg->ve[2];

  inl2_states_msg.posEcef.x = msg->ecef[0];
  inl2_states_msg.posEcef.y = msg->ecef[1];
  inl2_states_msg.posEcef.z = msg->ecef[2];

  inl2_states_msg.gyroBias.x = msg->biasPqr[0];
  inl2_states_msg.gyroBias.y = msg->biasPqr[1];
  inl2_states_msg.gyroBias.z = msg->biasPqr[2];

  inl2_states_msg.accelBias.x = msg->biasAcc[0];
  inl2_states_msg.accelBias.y = msg->biasAcc[1];
  inl2_states_msg.accelBias.z = msg->biasAcc[2];

  inl2_states_msg.baroBias = msg->biasBaro;
  inl2_states_msg.magDec = msg->magDec;
  inl2_states_msg.magInc = msg->magInc;

  // Use custom INL2 states message
  if(INL2_states_.enabled)
  {
    INL2_states_.pub.publish(inl2_states_msg);
  }
}


void InertialSenseROS::IMU_callback(const dual_imu_t* const msg)
{
  imu1_msg.header.stamp = imu2_msg.header.stamp = ros_time_from_start_time(msg->time);
  imu1_msg.header.frame_id = imu2_msg.header.frame_id = frame_id_;

  imu1_msg.angular_velocity.x = msg->I[0].pqr[0];
  imu1_msg.angular_velocity.y = msg->I[0].pqr[1];
  imu1_msg.angular_velocity.z = msg->I[0].pqr[2];
  imu1_msg.linear_acceleration.x = msg->I[0].acc[0];
  imu1_msg.linear_acceleration.y = msg->I[0].acc[1];
  imu1_msg.linear_acceleration.z = msg->I[0].acc[2];

  //  imu2_msg.angular_velocity.x = msg->I[1].pqr[0];
  //  imu2_msg.angular_velocity.y = msg->I[1].pqr[1];
  //  imu2_msg.angular_velocity.z = msg->I[1].pqr[2];
  //  imu2_msg.linear_acceleration.x = msg->I[1].acc[0];
  //  imu2_msg.linear_acceleration.y = msg->I[1].acc[1];
  //  imu2_msg.linear_acceleration.z = msg->I[1].acc[2];

  if (IMU_.enabled)
  {
    IMU_.pub.publish(imu1_msg);
    //    IMU_.pub2.publish(imu2_msg);
  }
}


void InertialSenseROS::GPS_pos_callback(const gps_pos_t * const msg)
{
  GPS_week_ = msg->week;
  GPS_towOffset_ = msg->towOffset;
  if (GPS_.enabled && msg->status&GPS_STATUS_FLAGS_FIX_OK)
  {
    gps_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs/1e3);
    gps_msg.fix_type = msg->status & GPS_STATUS_FIX_MASK;
    gps_msg.header.frame_id =frame_id_;
    gps_msg.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
    gps_msg.cno = msg->cnoMean;
    gps_msg.latitude = msg->lla[0];
    gps_msg.longitude = msg->lla[1];
    gps_msg.altitude = msg->lla[2];
    gps_msg.posEcef.x = ecef_[0] = msg->ecef[0];
    gps_msg.posEcef.y = ecef_[1] = msg->ecef[1];
    gps_msg.posEcef.z = ecef_[2] = msg->ecef[2];
    gps_msg.hMSL = msg->hMSL;
    gps_msg.hAcc = msg->hAcc;
    gps_msg.vAcc = msg->vAcc;
    gps_msg.pDop = msg->pDop;
    publishGPS();
  }
}

void InertialSenseROS::GPS_vel_callback(const gps_vel_t * const msg)
{
	if (GPS_.enabled && GPS_towOffset_ > 0.001)
	{
		gps_velEcef.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs/1e3);
		gps_velEcef.vector.x = msg->vel[0];
		gps_velEcef.vector.y = msg->vel[1];
		gps_velEcef.vector.z = msg->vel[2];
		publishGPS();
	}
}

void InertialSenseROS::publishGPS()
{
    if ((gps_velEcef.header.stamp - gps_msg.header.stamp).toSec() < 2e-3)
	{
		gps_msg.velEcef = gps_velEcef.vector;
		GPS_.pub.publish(gps_msg);
	}
}

void InertialSenseROS::update()
{
	IS_.Update();
}

void InertialSenseROS::strobe_in_time_callback(const strobe_in_time_t * const msg)
{
  // create the subscriber if it doesn't exist
  if (strobe_pub_.getTopic().empty())
    strobe_pub_ = nh_.advertise<std_msgs::Header>("strobe_time", 1);
  
  if (GPS_towOffset_ > 0.001)
  {
  std_msgs::Header strobe_msg;
  strobe_msg.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs * 1e-3);
  strobe_pub_.publish(strobe_msg);
}
}


void InertialSenseROS::GPS_info_callback(const gps_sat_t* const msg)
{
  if(GPS_towOffset_ < 0.001)
  { // Wait for valid msg->timeOfWeekMs
    return;
  }

  gps_info_msg.header.stamp =ros_time_from_tow(msg->timeOfWeekMs/1e3);
  gps_info_msg.header.frame_id = frame_id_;
  gps_info_msg.num_sats = msg->numSats;
  for (int i = 0; i < 50; i++)
  {
    gps_info_msg.sattelite_info[i].sat_id = msg->sat[i].svId;
    gps_info_msg.sattelite_info[i].cno = msg->sat[i].cno;
  }
  GPS_info_.pub.publish(gps_info_msg);
}


void InertialSenseROS::mag_callback(const magnetometer_t* const msg)
{
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.stamp = ros_time_from_start_time(msg->time);
  mag_msg.header.frame_id = frame_id_;
  mag_msg.magnetic_field.x = msg->mag[0];
  mag_msg.magnetic_field.y = msg->mag[1];
  mag_msg.magnetic_field.z = msg->mag[2];

  mag_.pub.publish(mag_msg);
}

void InertialSenseROS::baro_callback(const barometer_t * const msg)
{
  sensor_msgs::FluidPressure baro_msg;
  baro_msg.header.stamp = ros_time_from_start_time(msg->time);
  baro_msg.header.frame_id = frame_id_;
  baro_msg.fluid_pressure = msg->bar;
  baro_msg.variance = msg-> barTemp;

  baro_.pub.publish(baro_msg);
}

void InertialSenseROS::preint_IMU_callback(const preintegrated_imu_t * const msg)
{
  inertial_sense::PreIntIMU preintIMU_msg;
  preintIMU_msg.header.stamp = ros_time_from_start_time(msg->time);
  preintIMU_msg.header.frame_id = frame_id_;
  preintIMU_msg.dtheta.x = msg->theta1[0];
  preintIMU_msg.dtheta.y = msg->theta1[1];
  preintIMU_msg.dtheta.z = msg->theta1[2];

  preintIMU_msg.dvel.x = msg->vel1[0];
  preintIMU_msg.dvel.y = msg->vel1[1];
  preintIMU_msg.dvel.z = msg->vel1[2];

  preintIMU_msg.dt = msg->dt;

  dt_vel_.pub.publish(preintIMU_msg);
}

void InertialSenseROS::RTK_Misc_callback(const gps_rtk_misc_t* const msg)
{
  if (RTK_.enabled && GPS_towOffset_ > 0.001)
  {
    inertial_sense::RTKInfo rtk_info;
    rtk_info.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs/1000.0);
    rtk_info.baseAntcount = msg->baseAntennaCount;
    rtk_info.baseEph = msg->baseBeidouEphemerisCount + msg->baseGalileoEphemerisCount + msg->baseGlonassEphemerisCount
                       + msg->baseGpsEphemerisCount;
    rtk_info.baseObs = msg->baseBeidouObservationCount + msg->baseGalileoObservationCount + msg->baseGlonassObservationCount
                       + msg->baseGpsObservationCount;
    rtk_info.BaseLLA[0] = msg->baseLla[0];
    rtk_info.BaseLLA[1] = msg->baseLla[1];
    rtk_info.BaseLLA[2] = msg->baseLla[2];

    rtk_info.roverEph = msg->roverBeidouEphemerisCount + msg->roverGalileoEphemerisCount + msg->roverGlonassEphemerisCount
                        + msg->roverGpsEphemerisCount;
    rtk_info.roverObs = msg->roverBeidouObservationCount + msg->roverGalileoObservationCount + msg->roverGlonassObservationCount
                        + msg->roverGpsObservationCount;
    rtk_info.cycle_slip_count = msg->cycleSlipCount;
    RTK_.pub.publish(rtk_info);
  }
}


void InertialSenseROS::RTK_Rel_callback(const gps_rtk_rel_t* const msg)
{
  if (RTK_.enabled && GPS_towOffset_ > 0.001)
  {
    inertial_sense::RTKRel rtk_rel;
    rtk_rel.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs/1000.0);
    rtk_rel.differential_age = msg->differentialAge;
    rtk_rel.ar_ratio = msg->arRatio;
    rtk_rel.vector_base_to_rover.x = msg->baseToRoverVector[0];
    rtk_rel.vector_base_to_rover.y = msg->baseToRoverVector[1];
    rtk_rel.vector_base_to_rover.z = msg->baseToRoverVector[2];
    rtk_rel.distance_base_to_rover = msg->baseToRoverDistance;
    rtk_rel.heading_base_to_rover = msg->baseToRoverHeading;
    RTK_.pub2.publish(rtk_rel);

    // save for diagnostics
    diagnostic_ar_ratio_ = rtk_rel.ar_ratio;
    diagnostic_differential_age_ = rtk_rel.differential_age;
    diagnostic_heading_base_to_rover_ = rtk_rel.heading_base_to_rover;
  }
}

void InertialSenseROS::GPS_raw_callback(const gps_raw_t * const msg)
{
  switch(msg->dataType)
  {
  case raw_data_type_observation:
    GPS_obs_callback((obsd_t*)&msg->data.obs, msg->obsCount);
    break;

  case raw_data_type_ephemeris:
    GPS_eph_callback((eph_t*)&msg->data.eph);
    break;

  case raw_data_type_glonass_ephemeris:
    GPS_geph_callback((geph_t*)&msg->data.gloEph);
    break;

  default:
    break;
  }
}

void InertialSenseROS::GPS_obs_callback(const obsd_t * const msg, int nObs)
{
  if (obs_Vec_.obs.size() > 0 &&
       (msg[0].time.time != obs_Vec_.obs[0].time.time ||
        msg[0].time.sec != obs_Vec_.obs[0].time.sec))
      GPS_obs_bundle_timer_callback(ros::TimerEvent());

  for (int i = 0; i < nObs; i++)
  {
      inertial_sense::GNSSObservation obs;
      obs.header.stamp = ros_time_from_gtime(msg[i].time.time, msg[i].time.sec);
      obs.time.time = msg[i].time.time;
      obs.time.sec = msg[i].time.sec;
      obs.sat = msg[i].sat;
      obs.rcv = msg[i].rcv;
      obs.SNR = msg[i].SNR[0];
      obs.LLI = msg[i].LLI[0];
      obs.code = msg[i].code[0];
      obs.qualL = msg[i].qualL[0];
      obs.qualP = msg[i].qualP[0];
      obs.L = msg[i].L[0];
      obs.P = msg[i].P[0];
      obs.D = msg[i].D[0];
      obs_Vec_.obs.push_back(obs);
      last_obs_time_ = ros::Time::now();
  }
}

void InertialSenseROS::GPS_obs_bundle_timer_callback(const ros::TimerEvent &e)
{
    if (obs_Vec_.obs.size() == 0)
        return;

    if ((ros::Time::now() - last_obs_time_).toSec() > 1e-2)
    {
        obs_Vec_.header.stamp = ros_time_from_gtime(obs_Vec_.obs[0].time.time, obs_Vec_.obs[0].time.sec);
        obs_Vec_.time = obs_Vec_.obs[0].time;
        GPS_obs_.pub.publish(obs_Vec_);
        obs_Vec_.obs.clear();
//        cout << "dt" << (obs_Vec_.header.stamp - ros::Time::now()) << endl;
    }
}


void InertialSenseROS::GPS_eph_callback(const eph_t * const msg)
{
  inertial_sense::GNSSEphemeris eph;
  eph.sat = msg->sat;
  eph.iode = msg->iode;
  eph.iodc = msg->iodc;
  eph.sva = msg->sva;
  eph.svh = msg->svh;
  eph.week = msg->week;
  eph.code = msg->code;
  eph.flag = msg->flag;
  eph.toe.time = msg->toe.time;
  eph.toc.time = msg->toc.time;
  eph.ttr.time = msg->ttr.time;
  eph.toe.sec = msg->toe.sec;
  eph.toc.sec = msg->toc.sec;
  eph.ttr.sec = msg->ttr.sec;
  eph.A = msg->A;
  eph.e = msg->e;
  eph.i0 = msg->i0;
  eph.OMG0 = msg->OMG0;
  eph.omg = msg->omg;
  eph.M0 = msg->M0;
  eph.deln = msg->deln;
  eph.OMGd = msg->OMGd;
  eph.idot = msg->idot;
  eph.crc = msg->crc;
  eph.crs = msg->crs;
  eph.cuc = msg->cuc;
  eph.cus = msg->cus;
  eph.cic = msg->cic;
  eph.cis = msg->cis;
  eph.toes = msg->toes;
  eph.fit = msg->fit;
  eph.f0 = msg->f0;
  eph.f1 = msg->f1;
  eph.f2 = msg->f2;
  eph.tgd[0] = msg->tgd[0];
  eph.tgd[1] = msg->tgd[1];
  eph.tgd[2] = msg->tgd[2];
  eph.tgd[3] = msg->tgd[3];
  eph.Adot = msg->Adot;
  eph.ndot = msg->ndot;
  GPS_eph_.pub.publish(eph);
}

void InertialSenseROS::GPS_geph_callback(const geph_t * const msg)
{
  inertial_sense::GlonassEphemeris geph;
  geph.sat = msg->sat;
  geph.iode = msg->iode;
  geph.frq = msg->frq;
  geph.svh = msg->svh;
  geph.sva = msg->sva;
  geph.age = msg->age;
  geph.toe.time = msg->toe.time;
  geph.tof.time = msg->tof.time;
  geph.toe.sec = msg->toe.sec;
  geph.tof.sec = msg->tof.sec;
  geph.pos[0] = msg->pos[0];
  geph.pos[1] = msg->pos[1];
  geph.pos[2] = msg->pos[2];
  geph.vel[0] = msg->vel[0];
  geph.vel[1] = msg->vel[1];
  geph.vel[2] = msg->vel[2];
  geph.acc[0] = msg->acc[0];
  geph.acc[1] = msg->acc[1];
  geph.acc[2] = msg->acc[2];
  geph.taun = msg->taun;
  geph.gamn = msg->gamn;
  geph.dtaun = msg->dtaun;
  GPS_eph_.pub2.publish(geph);
}

void InertialSenseROS::diagnostics_callback(const ros::TimerEvent& event)
{
  // Create diagnostic objects
  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.header.stamp = ros::Time::now();

  // CNO mean
  diagnostic_msgs::DiagnosticStatus cno_mean;
  cno_mean.name = "CNO Mean";
  cno_mean.level =  diagnostic_msgs::DiagnosticStatus::OK;
  cno_mean.message = std::to_string(gps_msg.cno);
  diag_array.status.push_back(cno_mean);

  if (RTK_.enabled){
    diagnostic_msgs::DiagnosticStatus rtk_status;
    rtk_status.name = "RTK";
    rtk_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    std::string rtk_message;

    // AR ratio
    diagnostic_msgs::KeyValue ar_ratio;
    ar_ratio.key = "AR Ratio";
    ar_ratio.value = std::to_string(diagnostic_ar_ratio_);
    rtk_status.values.push_back(ar_ratio);
    if (diagnostic_ar_ratio_ < 3.0){
      rtk_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      rtk_message = "Float: " + std::to_string(diagnostic_ar_ratio_);
    } else if (diagnostic_ar_ratio_ < 6.0){
      rtk_message = "Fix: " + std::to_string(diagnostic_ar_ratio_);
    } else {
      rtk_message = "Fix and Hold: " + std::to_string(diagnostic_ar_ratio_);
    }

    // Differential age
    diagnostic_msgs::KeyValue differential_age;
    differential_age.key = "Differential Age";
    differential_age.value = std::to_string(diagnostic_differential_age_);
    rtk_status.values.push_back(differential_age);
    if (diagnostic_differential_age_ > 1.5){
      rtk_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      rtk_message += " Differential Age Large";
    }

    // Heading base to rover
    diagnostic_msgs::KeyValue heading_base_to_rover;
    heading_base_to_rover.key = "Heading Base to Rover (rad)";
    heading_base_to_rover.value = std::to_string(diagnostic_heading_base_to_rover_);
    rtk_status.values.push_back(heading_base_to_rover);
    
    rtk_status.message = rtk_message;
    diag_array.status.push_back(rtk_status);
  }

  diagnostics_.pub.publish(diag_array);
}

bool InertialSenseROS::set_current_position_as_refLLA(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  (void)req;
  double current_lla_[3];
  current_lla_[0] = lla_[0];
  current_lla_[1] = lla_[1];
  current_lla_[2] = lla_[2];

  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&current_lla_), sizeof(current_lla_), offsetof(nvm_flash_cfg_t, refLla));

  comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

  int i = 0;
  nvm_flash_cfg_t current_flash = IS_.GetFlashConfig();
  while (current_flash.refLla[0] == IS_.GetFlashConfig().refLla[0] && current_flash.refLla[1] == IS_.GetFlashConfig().refLla[1] && current_flash.refLla[2] == IS_.GetFlashConfig().refLla[2])
  {
      comManagerStep();
      i++;
      if(i>100){break;}
    }

  if(current_lla_[0] == IS_.GetFlashConfig().refLla[0] && current_lla_[1] == IS_.GetFlashConfig().refLla[1] && current_lla_[2] == IS_.GetFlashConfig().refLla[2])
  {
      comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
      res.success = true;
      res.message = ("Update was succesful.  refLla: Lat: " + to_string(current_lla_[0]) + "  Lon: " +to_string( current_lla_[1]) + "  Alt: " + to_string(current_lla_[2]));
    }
  else
  {
      comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
      res.success = false;
      res.message = "Unable to update refLLA. Please try again.";

  }
    
  }

bool InertialSenseROS::set_refLLA_to_value(inertial_sense::refLLAUpdate::Request &req, inertial_sense::refLLAUpdate::Response &res)
{
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&req.lla), sizeof(req.lla), offsetof(nvm_flash_cfg_t, refLla));

  comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

  int i = 0;
  nvm_flash_cfg_t current_flash = IS_.GetFlashConfig();
  while (current_flash.refLla[0] == IS_.GetFlashConfig().refLla[0] && current_flash.refLla[1] == IS_.GetFlashConfig().refLla[1] && current_flash.refLla[2] == IS_.GetFlashConfig().refLla[2])
{
      comManagerStep();
      i++;
      if(i>100){break;}
}

  if(req.lla[0] == IS_.GetFlashConfig().refLla[0] && req.lla[1] == IS_.GetFlashConfig().refLla[1] && req.lla[2] == IS_.GetFlashConfig().refLla[2])
{
      comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
  res.success = true;
      res.message = ("Update was succesful.  refLla: Lat: " + to_string(req.lla[0]) + "  Lon: " +to_string( req.lla[1]) + "  Alt: " + to_string(req.lla[2]));
}
  else
{
      comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
      res.success = false;
      res.message = "Unable to update refLLA. Please try again.";
  }
}

bool InertialSenseROS::perform_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  (void)req;
    uint32_t single_axis_command = 2;
    IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t*>(&single_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, recalCmd));

    is_comm_instance_t comm;
    uint8_t buffer[2048];
  is_comm_init(&comm, buffer, sizeof(buffer));
    serial_port_t* serialPort = IS_.GetSerialPort();
  uint8_t inByte;
  int n;

  while ((n = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
    {
    // Search comm buffer for valid packets
    if (is_comm_parse_byte(&comm, inByte) == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_INS_1)
        {
      ins_1_t* msg = (ins_1_t*)(comm.dataPtr + comm.dataHdr.offset);
            if (msg->insStatus & 0x00400000)
            {
  res.success = true;
                res.message = "Successfully initiated mag recalibration.";
                return true;
            }
        }
    }
}

bool InertialSenseROS::perform_multi_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  (void)req;
  uint32_t multi_axis_command = 1;
  IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t*>(&multi_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, recalCmd));

  is_comm_instance_t comm;
  uint8_t buffer[2048];
  is_comm_init(&comm, buffer, sizeof(buffer));
  serial_port_t* serialPort = IS_.GetSerialPort();
  uint8_t inByte;
  int n;

  while ((n = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
  {
    // Search comm buffer for valid packets
    if (is_comm_parse_byte(&comm, inByte) == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_INS_1)
      {
      ins_1_t* msg = (ins_1_t*)(comm.dataPtr + comm.dataHdr.offset);
          if (msg->insStatus & 0x00400000)
          {
  res.success = true;
        res.message = "Successfully initiated mag recalibration.";
              return true;
          }
      }
  }
}

void InertialSenseROS::reset_device()
{
  // send reset command
  system_command_t reset_command;
  reset_command.command = 99;
  reset_command.invCommand = ~reset_command.command;
  IS_.SendData(DID_SYS_CMD, reinterpret_cast<uint8_t*>(&reset_command), sizeof(system_command_t), 0);
  sleep(1);
}

bool InertialSenseROS::update_firmware_srv_callback(inertial_sense::FirmwareUpdate::Request &req, inertial_sense::FirmwareUpdate::Response &res)
{
  IS_.Close();
  vector<InertialSense::bootloader_result_t> results = IS_.BootloadFile("*", req.filename, 921600);
  if (!results[0].error.empty())
  {
    res.success = false;
    res.message = results[0].error;
    return false;
  }
  IS_.Open(port_.c_str(), baudrate_);
  return true;
}


ros::Time InertialSenseROS::ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek)
{
  ros::Time rostime(0, 0);
  //  If we have a GPS fix, then use it to set timestamp
  if (GPS_towOffset_ > 0.001)
  {
    uint64_t sec = UNIX_TO_GPS_OFFSET + floor(timeOfWeek) + week*7*24*3600;
    uint64_t nsec = (timeOfWeek - floor(timeOfWeek))*1e9;
    rostime = ros::Time(sec, nsec);
  }
  else
  {
    // Otherwise, estimate the uINS boot time and offset the messages
    if (!got_first_message_)
    {
      got_first_message_ = true;
      INS_local_offset_ = ros::Time::now().toSec() - timeOfWeek;
    }
    else // low-pass filter offset to account for drift
    {
      double y_offset = ros::Time::now().toSec() - timeOfWeek;
      INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
    }
    // Publish with ROS time
    rostime = ros::Time(INS_local_offset_ + timeOfWeek);
  }
  return rostime;
}

ros::Time InertialSenseROS::ros_time_from_start_time(const double time)
{
  ros::Time rostime(0, 0);
  
  //  If we have a GPS fix, then use it to set timestamp
  if (GPS_towOffset_ > 0.001)
  {
    uint64_t sec = UNIX_TO_GPS_OFFSET + floor(time + GPS_towOffset_) + GPS_week_*7*24*3600;
    uint64_t nsec = (time + GPS_towOffset_ - floor(time + GPS_towOffset_))*1e9;
    rostime = ros::Time(sec, nsec);
  }
  else
  {
    // Otherwise, estimate the uINS boot time and offset the messages
    if (!got_first_message_)
    {
      got_first_message_ = true;
      INS_local_offset_ = ros::Time::now().toSec() - time;
    }
    else // low-pass filter offset to account for drift
    {
      double y_offset = ros::Time::now().toSec() - time;
      INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
    }
    // Publish with ROS time
    rostime = ros::Time(INS_local_offset_ + time);
  }
  return rostime;
}

ros::Time InertialSenseROS::ros_time_from_tow(const double tow)
{
  return ros_time_from_week_and_tow(GPS_week_, tow);
}

double InertialSenseROS::tow_from_ros_time(const ros::Time &rt)
{
  return (rt.sec - UNIX_TO_GPS_OFFSET - GPS_week_*604800) + rt.nsec*1.0e-9;
}

ros::Time InertialSenseROS::ros_time_from_gtime(const uint64_t sec, double subsec)
{
    ros::Time out;
    out.sec = sec - LEAP_SECONDS;
    out.nsec = subsec * 1e9;
    return out;
}

