/* copyright [2020] BrownieAlice */

#include "imu_manager.hpp"

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

namespace imu {
imu_manager::imu_manager() {
  filter.setOrientation(1, 0, 0, 0);

  xyz = cnoid::Vector3::Zero();
  dxyz = cnoid::Vector3::Zero();
  ddxyz = cnoid::Vector3::Zero();
  rpy = cnoid::Vector3::Zero();
  drpy = cnoid::Vector3::Zero();
  ddrpy = cnoid::Vector3::Zero();
}

bool imu_manager::init(const std::string acc_name, const std::string gyro_name,
                       cnoid::SimpleControllerIO* io, const double time_step,
                       ros::Publisher pub) {
  const auto io_body = io->body();

  //加速度センサ初期化.
  accel = io_body->findDevice<cnoid::AccelerationSensor>(acc_name);
  if (accel == nullptr) {
    std::cout << acc_name << "(acc_name) is not found" << std::endl;
    return false;
  }
  accel->on(true);
  io->enableInput(accel);

  // ジャイロ初期化.
  gyro = io_body->findDevice<cnoid::RateGyroSensor>(gyro_name);
  if (gyro == nullptr) {
    std::cout << gyro_name << "(gyro_name) is not found" << std::endl;
    return false;
  }
  gyro->on(true);
  io->enableInput(gyro);

  this->time_step = time_step;
  this->pub = pub;
  return true;
}

void imu_manager::update() {
  const cnoid::Vector3 acc_data = accel->dv();
  const cnoid::Vector3 gyro_data = gyro->w();

  filter.madgwickAHRSupdateIMU(gyro_data[0], gyro_data[1], gyro_data[2],
                               acc_data[0], acc_data[1], acc_data[2],
                               time_step);
  double q0, q1, q2, q3;
  filter.getOrientation(q0, q1, q2, q3);
  quat = cnoid::Quaterniond(q0, q1, q2, q3);
  quat.normalize();
  // Madgiwickフィルタによるクォータニオン計算.

  const cnoid::Vector3 rpy_next = cnoid::rpyFromRot(quat.matrix());
  const cnoid::Vector3 drpy_next = (rpy_next - rpy) / time_step;

  ddrpy = (drpy_next - drpy) / time_step;
  drpy = drpy_next;
  rpy = rpy_next;
  // 姿勢角推定.

  ddxyz = quat.matrix() * acc_data;
  ddxyz[2] -= 9.80665;
  dxyz += ddxyz * time_step;
  xyz += dxyz * time_step;
  // 位置推定.
}

std::tuple<cnoid::Vector3, cnoid::Vector3, cnoid::Vector3, cnoid::Vector3,
           cnoid::Vector3, cnoid::Vector3>
imu_manager::get() {
  return {xyz, dxyz, ddxyz, rpy, drpy, ddrpy};
}

void imu_manager::set(
    const std::tuple<cnoid::Vector3, cnoid::Vector3, cnoid::Vector3,
                     cnoid::Vector3, cnoid::Vector3, cnoid::Vector3>& data) {
  xyz = std::get<0>(data);
  dxyz = std::get<1>(data);
  ddxyz = std::get<2>(data);
  rpy = std::get<3>(data);
  drpy = std::get<4>(data);
  ddrpy = std::get<5>(data);

  quat = cnoid::Quaterniond(cnoid::AngleAxisd(rpy[0], cnoid::Vector3::UnitX()) *
                            cnoid::AngleAxisd(rpy[1], cnoid::Vector3::UnitY()) *
                            cnoid::AngleAxisd(rpy[2], cnoid::Vector3::UnitZ()));
  quat.normalize();
  filter.setOrientation(quat.w(), quat.x(), quat.y(), quat.z());
}

void imu_manager::publish() {
  // publishes a Pose msg.
  geometry_msgs::Pose msg;
  msg.position.x = xyz[0];
  msg.position.y = xyz[1];
  msg.position.z = xyz[2];
  msg.orientation.x = quat.x();
  msg.orientation.y = quat.y();
  msg.orientation.z = quat.z();
  msg.orientation.w = quat.w();

  pub.publish(msg);

  // publishes a TF
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = "HORIZONTAL_PLANE";
  tf.child_frame_id = "BODY";
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  broadcaster_.sendTransform(tf);
}

void imu_manager::reset(){
  filter.setOrientation(1, 0, 0, 0);
  xyz = cnoid::Vector3::Zero();
  dxyz = cnoid::Vector3::Zero();
  ddxyz = cnoid::Vector3::Zero();
  rpy = cnoid::Vector3::Zero();
  drpy = cnoid::Vector3::Zero();
  ddrpy = cnoid::Vector3::Zero();
}

}  // namespace imu
