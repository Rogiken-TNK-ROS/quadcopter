//
// Created by hans on 2020/01/11.
//

#pragma once

//-------------include----------------//
#include <cnoid/RangeCamera>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include "imu_manager.hpp"

#include <cnoid/EigenUtil>
#include <cnoid/RangeCamera>
#include <cnoid/SimpleController>
//------------namespace---------------//
//--------------class-----------------//
namespace rgbd_camera {
class RGBDCameraManager {
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

public:
  RGBDCameraManager() {}

  void initROS(std::string topic_name, ros::NodeHandle nh) {
    publisher_ = nh.advertise<PointCloud>(topic_name, 10);

    cloud_msg_ = PointCloud::Ptr(new PointCloud);
    cloud_msg_->header.frame_id = "WRS";
  }
  void initCnoidDevice(std::string device_name, cnoid::SimpleControllerIO *io) {
    cnoid_cam_ = io->body()->findDevice<cnoid::RangeCamera>(device_name);
    io->enableInput(cnoid_cam_);
    cnoid_cam_->on(true);
    cnoid_cam_->notifyStateChange();
  }

  void publishCloud(cnoid::BodyPtr io_body, cnoid::Link *device_link,
                    imu::imu_manager &imu_manager, double time_step) {
    static int count = 0;
    if (count++ * time_step < 0.8) {
      return;
    }

    count = 0;

    auto root = io_body->rootLink()->position().translation();
    //    auto camera =
    //        ioBody->findDevice<Camera>("Camera")->link()->position().translation();

    auto cnoid_points = cnoid_cam_->points();
    const auto q = device_link->q();

    cloud_msg_->points.clear();
    cloud_msg_->points.reserve(cnoid_points.size());
    Eigen::Vector3d axis1;
    axis1 << 1, 0, 0;
    auto axis_angle = Eigen::AngleAxisd(M_PI / 2 + q, axis1);
    const auto [xyz, dxyz, ddxyz, rpy, drpy, ddrpy] = imu_manager.get();
    Eigen::Matrix3d rot = cnoid::rotFromRpy(rpy);

    auto &image = cnoid_cam_->image();
    ROS_INFO("num : %d %d %d", cnoid_points.size(),
             image.height() * image.width(), image.numComponents());
    int pixel_num = -1;
    for (const auto &p : cnoid_points) {
      pixel_num++;
      const auto norm = p.norm();
      if (!isinf(norm) && !isnan(norm)) {
        static Eigen::Vector3d poi;
        poi << p[0], p[1], p[2] + 0.03;
        static Eigen::Vector3d offset;
        offset << 0, 0, 0.05;
        const Eigen::Vector3d relative = axis_angle * poi + offset;
        if (0 < relative[2]) {
          continue;
        }

        static Eigen::Matrix3d mirror_xy;
        mirror_xy << -1, 0, 0, 0, -1, 0, 0, 0, 1;
        static Eigen::Vector3d axis2;
        axis2 << 0, 0, 1;
        auto axis_angle2 = Eigen::AngleAxisd(M_PI / 2, axis2);
        const Eigen::Vector3d point =
            rot * mirror_xy * axis_angle2 * relative + xyz;
        auto pixel = image.pixels() + image.numComponents() * pixel_num;
        pcl::PointXYZRGB pcl_point(pixel[0], pixel[1], pixel[2]);
        pcl_point.x = point[0];
        pcl_point.y = point[1];
        pcl_point.z = point[2];
        cloud_msg_->points.push_back(pcl_point);
      }
    }
    cloud_msg_->height = cloud_msg_->points.size();
    cloud_msg_->width = 1;
    //    printf("pcl_size: %d\n", cloud_msg_->points.size());

    publisher_.publish(cloud_msg_);
  }

private:
  ros::Publisher publisher_;
  PointCloud::Ptr cloud_msg_;
  cnoid::RangeCamera *cnoid_cam_;
};
} // namespace rgbd_camera
