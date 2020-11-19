#ifndef LOAM_HORIZON_GYR_INT_H
#define LOAM_HORIZON_GYR_INT_H

#include <sensor_msgs/Imu.h>
#include "sophus/so3.hpp"

class GyrInt {
 public:
  GyrInt();
  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  const Sophus::SO3d GetRot() const;

 private:
  // Sophus::SO3d r_;
  /// last_imu_ is
  double start_timestamp_;
  sensor_msgs::ImuConstPtr last_imu_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::vector<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Sophus::SO3d> v_rot_;
};

#endif  // LOAM_HORIZON_GYR_INT_H
