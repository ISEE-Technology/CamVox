#include <ros/ros.h>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <mutex>
#include <thread>

#include "data_process.h"

/// *************Config data
std::string topic_pcl = "/livox_pcl0";
std::string topic_imu = "/imu";
/// *************

/// To notify new data
std::mutex mtx_buffer;
std::condition_variable sig_buffer;
bool b_exit = false;
bool b_reset = false;

/// Buffers for measurements
double last_timestamp_lidar = -1;
std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
double last_timestamp_imu = -1;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

void SigHandle(int sig) {
  b_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

void pointcloud_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  const double timestamp = msg->header.stamp.toSec();
  // ROS_DEBUG("get point cloud at time: %.6f", timestamp);

  mtx_buffer.lock();

  if (timestamp < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }
  last_timestamp_lidar = timestamp;

  lidar_buffer.push_back(msg);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  double timestamp = msg->header.stamp.toSec();
  // ROS_DEBUG("get imu at time: %.6f", timestamp);

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    ROS_ERROR("imu loop back, clear buffer");
    imu_buffer.clear();
    b_reset = true;
  }
  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool SyncMeasure(MeasureGroup &measgroup) {
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    /// Note: this will happen
    return false;
  }

  if (imu_buffer.front()->header.stamp.toSec() >
      lidar_buffer.back()->header.stamp.toSec()) {
    lidar_buffer.clear();
    ROS_ERROR("clear lidar buffer, only happen at the beginning");
    return false;
  }

  if (imu_buffer.back()->header.stamp.toSec() <
      lidar_buffer.front()->header.stamp.toSec()) {
    return false;
  }

  /// Add lidar data, and pop from buffer
  measgroup.lidar = lidar_buffer.front();
  lidar_buffer.pop_front();
  double lidar_time = measgroup.lidar->header.stamp.toSec();

  /// Add imu data, and pop from buffer
  measgroup.imu.clear();
  int imu_cnt = 0;
  for (const auto &imu : imu_buffer) {
    double imu_time = imu->header.stamp.toSec();
    if (imu_time <= lidar_time) {
      measgroup.imu.push_back(imu);
      imu_cnt++;
    }
  }
  for (int i = 0; i < imu_cnt; ++i) {
    imu_buffer.pop_front();
  }
  // ROS_DEBUG("add %d imu msg", imu_cnt);

  return true;
}

void ProcessLoop(std::shared_ptr<ImuProcess> p_imu) {
  ROS_INFO("Start ProcessLoop");

  ros::Rate r(1000);
  while (ros::ok()) {
    MeasureGroup meas;
    std::unique_lock<std::mutex> lk(mtx_buffer);
    sig_buffer.wait(lk,
                    [&meas]() -> bool { return SyncMeasure(meas) || b_exit; });
    lk.unlock();

    if (b_exit) {
      ROS_INFO("b_exit=true, exit");
      break;
    }

    if (b_reset) {
      ROS_WARN("reset when rosbag play back");
      p_imu->Reset();
      b_reset = false;
      continue;
    }
    p_imu->Process(meas);

    r.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "data_process");
  ros::NodeHandle nh;
  signal(SIGINT, SigHandle);

  ros::Subscriber sub_pcl = nh.subscribe(topic_pcl, 100, pointcloud_cbk);
  ros::Subscriber sub_imu = nh.subscribe(topic_imu, 1000, imu_cbk);

  std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

  std::vector<double> vec;
  if( nh.getParam("/ExtIL", vec) ){
    Eigen::Quaternion<double> q_il;
    Eigen::Vector3d t_il;
    q_il.w() = vec[0];
    q_il.x() = vec[1];
    q_il.y() = vec[2];
    q_il.z() = vec[3];
    t_il << vec[4], vec[5], vec[6];
    p_imu->set_T_i_l(q_il, t_il);
    ROS_INFO("Extrinsic Parameter RESET ... ");
  }

  /// for debug
  p_imu->nh = nh;

  std::thread th_proc(ProcessLoop, p_imu);

  // ros::spin();
  ros::Rate r(1000);
  while (ros::ok()) 
  {
    if (b_exit) break;

    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Wait for process loop exit");
  if (th_proc.joinable()) th_proc.join();

  return 0;
}
