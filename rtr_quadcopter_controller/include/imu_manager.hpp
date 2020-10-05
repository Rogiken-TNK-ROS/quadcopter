/* copyright [2020] BrownieAlice */

#ifndef QUADCOPTERCONTROLLERWRS_IMU_MANAGER_HPP
#define QUADCOPTERCONTROLLERWRS_IMU_MANAGER_HPP

#include <ros/ros.h>

#include <cnoid/AccelerationSensor>
#include <cnoid/EigenUtil>
#include <cnoid/RateGyroSensor>
#include <cnoid/SimpleController>
#include <string>
#include <tuple>

#include "imu_filter.h"

namespace imu {
class imu_manager {
 public:
  imu_manager();
  bool init(const std::string acc_name, const std::string gyro_name,
            cnoid::SimpleControllerIO* io, const double time_step,
            ros::Publisher pub);
  void update();
  std::tuple<cnoid::Vector3, cnoid::Vector3, cnoid::Vector3, cnoid::Vector3,
             cnoid::Vector3, cnoid::Vector3>
  get();

  void set(
      const std::tuple<cnoid::Vector3, cnoid::Vector3, cnoid::Vector3,
                       cnoid::Vector3, cnoid::Vector3, cnoid::Vector3>& data);
  void publish();

 private:
  double time_step;
  cnoid::AccelerationSensor* accel;
  cnoid::RateGyroSensor* gyro;

  ImuFilter filter;

  cnoid::Vector3 xyz, dxyz, ddxyz;
  cnoid::Vector3 rpy, drpy, ddrpy;
  cnoid::Quaterniond quat;

  ros::Publisher pub;
};
}  // namespace imu
#endif  //  QUADCOPTERCONTROLLERWRS_IMU_MANAGER_HPP
