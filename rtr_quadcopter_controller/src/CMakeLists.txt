add_cnoid_simple_controller(RTRQuadcopterController
  rtr_quadcopter_controller.cpp
  imu_filter.cpp
  imu_manager.cpp
  imu_debugger.cpp
  rgbd_camera.cpp
)

add_cnoid_simple_controller(RTRQuadcopterCameraController
  camera_controller.cpp
)
