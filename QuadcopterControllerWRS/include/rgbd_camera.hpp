//
// Created by hans on 2020/01/11.
//

#pragma once


#include <string>
#include <ros/ros.h>


//Forward Declaration
namespace cnoid {
class Link;
class SimpleControllerIO;
class Body;
template <class T> class ref_ptr;
typedef ref_ptr<Body> BodyPtr;
class RangeCamera;
} // namespace cnoid

namespace imu {
class imu_manager;
}


namespace rgbd_camera {
class RGBDCameraManager {

public:
  RGBDCameraManager() {}

  void initROS(std::string topic_name, ros::NodeHandle nh);
  void initCnoidDevice(std::string device_name, cnoid::SimpleControllerIO *io);

  void publishCloud(cnoid::BodyPtr io_body, cnoid::Link *device_link,
                    imu::imu_manager *imu_manager, double time_step);

private:
  ros::Publisher publisher_;

  cnoid::RangeCamera *cnoid_cam_;
};
} // namespace rgbd_camera
