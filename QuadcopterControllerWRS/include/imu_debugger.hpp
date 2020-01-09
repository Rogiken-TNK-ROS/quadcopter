/* copyright [2020] BrownieAlice */

#ifndef QUADCOPTERCONTROLLERWRS_IMU_DEBUGER_HPP
#define QUADCOPTERCONTROLLERWRS_IMU_DEBUGER_HPP

#include <cnoid/SimpleController>
#include <tuple>

#include "imu_filter.h"

namespace imu {
class imu_debugger {
 public:
  imu_debugger();
  void init(cnoid::BodyPtr io_body, const double time_step);
  void update();
  std::tuple<cnoid::Vector3, cnoid::Vector3, cnoid::Vector3, cnoid::Vector3,
             cnoid::Vector3, cnoid::Vector3>
  get();
  void compare(
      const std::tuple<cnoid::Vector3, cnoid::Vector3, cnoid::Vector3,
                       cnoid::Vector3, cnoid::Vector3, cnoid::Vector3> &data);

 private:
  double time_step;
  cnoid::BodyPtr io_body;

  cnoid::Vector3 xyz, dxyz, ddxyz;
  cnoid::Vector3 rpy, drpy, ddrpy;
};
}  // namespace imu
#endif  //  QUADCOPTERCONTROLLERWRS_IMU_DEBUGER_HPP
