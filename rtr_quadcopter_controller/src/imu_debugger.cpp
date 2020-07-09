/* copyright [2020] BrownieAlice */

#include "imu_debugger.hpp"

#include <cnoid/EigenUtil>
#include <iomanip>
#include <iostream>

namespace imu {
imu_debugger::imu_debugger() {}

void imu_debugger::init(cnoid::BodyPtr io_body, const double time_step) {
  this->io_body = io_body;
  this->time_step = time_step;

  const auto position = io_body->rootLink()->translation();

  xyz = cnoid::Vector3(position.x(), position.y(), position.z());
  rpy = cnoid::rpyFromRot(io_body->rootLink()->position().rotation());

  dxyz = cnoid::Vector3::Zero();
  ddxyz = cnoid::Vector3::Zero();
  drpy = cnoid::Vector3::Zero();
  ddrpy = cnoid::Vector3::Zero();
}

void imu_debugger::update() {
  const auto position = io_body->rootLink()->translation();

  const cnoid::Vector3 xyz_next =
      cnoid::Vector3(position.x(), position.y(), position.z());
  const cnoid::Vector3 dxyz_next = (xyz_next - xyz) / time_step;

  ddxyz = (dxyz_next - dxyz) / time_step;
  dxyz = dxyz_next;
  xyz = xyz_next;

  const cnoid::Vector3 rpy_next =
      cnoid::rpyFromRot(io_body->rootLink()->position().rotation());
  const cnoid::Vector3 drpy_next = (rpy_next - rpy) / time_step;

  ddrpy = (drpy_next - drpy) / time_step;
  drpy = drpy_next;
  rpy = rpy_next;
  // 姿勢角推定.
}

std::tuple<cnoid::Vector3, cnoid::Vector3, cnoid::Vector3, cnoid::Vector3,
           cnoid::Vector3, cnoid::Vector3>
imu_debugger::get() {
  return {xyz, dxyz, ddxyz, rpy, drpy, ddrpy};
}

void imu_debugger::compare(
    const std::tuple<cnoid::Vector3, cnoid::Vector3, cnoid::Vector3,
                     cnoid::Vector3, cnoid::Vector3, cnoid::Vector3> &data) {
  const auto [mxyz, dmxyz, ddmxyz, mrpy, dmrpy, ddmrpy] = data;

  const auto output = [](const auto i, const auto j) {
    std::cout << std::setprecision(5) << "[" << i[0] - j[0] << ", "
              << i[1] - j[1] << ", " << i[2] - j[2] << "]" << std::endl;
  };
  std::cout << "xyz:";
  output(xyz, mxyz);
  std::cout << "rpy:";
  output(rpy, mrpy);
}

}  // namespace imu
