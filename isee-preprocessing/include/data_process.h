#ifndef LOAM_HORIZON_DATA_PROCESS_H
#define LOAM_HORIZON_DATA_PROCESS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include "gyr_int.h"
#include "common.h"
#include "sophus/se3.hpp"

struct MeasureGroup {
  sensor_msgs::PointCloud2ConstPtr lidar;
  std::vector<sensor_msgs::Imu::ConstPtr> imu;
};

class ImuProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Process(const MeasureGroup &meas);
  void Reset();

  void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu);

  void UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out, double dt_be,
                    const Sophus::SE3d &Tbe);
  void set_T_i_l(Eigen::Quaterniond& q, Eigen::Vector3d& t){
    T_i_l = Sophus::SE3d(q, t);
  }

  ros::NodeHandle nh;

 private:
  /// Whether is the first frame, init for first frame
  bool b_first_frame_ = true;

  //// Input pointcloud
  PointCloudXYZI::Ptr cur_pcl_in_;
  //// Undistorted pointcloud
  PointCloudXYZI::Ptr cur_pcl_un_;

  double dt_l_c_;

  /// Transform form lidar to imu
  Sophus::SE3d T_i_l;
  //// For timestamp usage
  sensor_msgs::PointCloud2ConstPtr last_lidar_;
  sensor_msgs::ImuConstPtr last_imu_;

  /// For gyroscope integration
  GyrInt gyr_int_;
};

#endif  // LOAM_HORIZON_DATA_PROCESS_H
