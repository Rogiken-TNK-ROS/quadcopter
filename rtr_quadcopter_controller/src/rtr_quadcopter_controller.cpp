/**
   Copyright (c) 2018 Japan Atomic Energy Agency (JAEA).
   The original version is implemented as an RT-component.
   This is a simple controller version modified by AIST.
*/

#include <geometry_msgs/Pose.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <cnoid/AccelerationSensor> //センサ毎に必要
#include <cnoid/Camera>
#include <cnoid/EigenUtil>
#include <cnoid/RangeCamera>
#include <cnoid/RateGyroSensor>
#include <cnoid/RotorDevice>
// #include <cnoid/SharedJoystick>
#include <cnoid/SimpleController>
#include <iostream>
#include <chrono>

#include "imu_debugger.hpp"
#include "imu_filter.h"
#include "imu_manager.hpp"
#include "rtr_quadcopter_controller/QRPosition.h"

using namespace cnoid;

namespace
{

  const std::string propname[] = {"PROP0", "PROP1", "PROP2", "PROP3"};
  const std::string rotorname[] = {"RotorDevice0", "RotorDevice1", "RotorDevice2",
                                   "RotorDevice3"};

  enum JoyButton
  {
    SQUARE = 0,
    X = 1,
    CIRCLE = 2,
    TRIANGLE = 3,
    L1 = 4,
    R1 = 5,
    L2 = 6,
    R2 = 7,
    SHARE = 8,
    OPTION = 9,
    R_STICK = 10,
    L_STICK = 11,
    PS = 12,
    PAD = 13,
  };

  enum JoyAxis
  {
    L_H = 0,
    L_V = 1,
    R_H = 2,
    R_V = 3,
    L2_VAL = 4,
    R2_VAL = 5,
    GYRO_ROLL = 6,
    GYRO_PITCH = 7,
    ACC_Z = 8,
    DIR_PAD_H = 9,
    DIR_PAD_V = 10,
  };

  constexpr int rotorAxis[4] = {JoyAxis::L_V, JoyAxis::R_H, JoyAxis::R_V, JoyAxis::L_H};

  constexpr int CAMERA_AXIS = JoyAxis::DIR_PAD_V;
  constexpr int POWER_BTN = JoyButton::X;
  constexpr int ZOOM_IN_BTN = JoyButton::CIRCLE;
  constexpr int ZOOM_OUT_BTN = JoyButton::SQUARE;
  constexpr int ZOOM_RESET_BTN = JoyButton::TRIANGLE;

  constexpr float JOY_TIMEOUT_MS = 500;
  // ボタン周りの割当.

  constexpr double sign[4][4] = {{1.0, -1.0, -1.0, 1.0},
                                 {1.0, 1.0, -1.0, -1.0},
                                 {1.0, 1.0, 1.0, 1.0},
                                 {1.0, -1.0, 1.0, -1.0}};
  constexpr double dir[] = {1.0, -1.0, 1.0, -1.0};

  static constexpr double KP[] = {0.4, 0.4, 0.4, 1.0};
  static constexpr double KD[] = {0.02, 1.0, 1.0, 0.05};
  constexpr double RATE[] = {-1.0, 0.1, -0.1, -0.5};

  // For the stable mode
  constexpr double KPX[] = {0.4, 0.4};
  constexpr double KDX[] = {0.4, 0.4};
  constexpr double RATEX[] = {-1.0, -1.0};

  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
struct QRData{
  volatile bool has_request_processed = true;
  ros::ServiceServer server;
  rtr_quadcopter_controller::QRPosition::Request request;
  rtr_quadcopter_controller::QRPosition::Response response;
};
  class RTRQuadcopterController : public SimpleController
  {
  public:
    // SharedJoystickPtr joystick;
    int targetMode;
    BodyPtr ioBody;
    std::ostream *os;
    Link *prop[4];
    Multicopter::RotorDevice *rotor[4];
    Link *cameraT;
    Camera *camera2;
    RangeCamera *camera2_qr;

    sensor_msgs::Joy joy;
    ros::Subscriber joy_sub;
    bool joy_received = false;
    std::chrono::system_clock::time_point last_joy_receive_time;

    double timeStep;
    imu::imu_manager imu_manager;
    imu::imu_debugger imu_debugger;
    // For the stable mode
    bool isStableMode;
    bool prevModeButtonState;

    double qref;
    double qprev;
    bool power;
    bool powerprev;
    bool rotorswitch;

    virtual bool initialize(SimpleControllerIO *io) override;
    void calcPoint();
    bool calcQRPoint();
    using QRReq = rtr_quadcopter_controller::QRPosition::Request;
    using QRRes = rtr_quadcopter_controller::QRPosition::Response;
    bool QRPositionCallback(QRReq &req, QRRes &res);
    void joyconCallback(const sensor_msgs::Joy);
    virtual bool control() override;

    RangeCamera *cam;
    ros::NodeHandle node;
    ros::Publisher pub;
    PointCloud::Ptr msg;

    Vector3d offset;

    Matrix3d AxisAngle1;
    Vector3d axis1;

    Matrix3d AxisAngle2;
    Vector3d axis2;

    Matrix3d mirror_xy;

    double fieldOfView;
    unsigned int wait_camera = 0;
    QRData qr_data;
  };

} // namespace

bool RTRQuadcopterController::initialize(SimpleControllerIO *io)
{
  ioBody = io->body();
  os = &io->os();
  timeStep = io->timeStep();
  io->enableInput(ioBody->rootLink(), LINK_POSITION);

  imu_manager.init("AccelerationSensor", "RateGyroSensor", io, timeStep,
                   node.advertise<geometry_msgs::Pose>("/quadcopter/quadcopter_pose", 10));
  imu_debugger.init(ioBody, timeStep);
  imu_manager.set(imu_debugger.get());

  camera2 = ioBody->findDevice<Camera>("Camera2"); //取得
  io->enableInput(camera2);
  camera2->setResolution(1280, 720); //解像度変更
  fieldOfView = 0.785398;
  camera2->setFieldOfView(fieldOfView);
  camera2->notifyStateChange();

  camera2_qr = ioBody->findDevice<RangeCamera>("Camera2_QR");
  io->enableInput(camera2_qr);
  camera2_qr->notifyStateChange();
  qr_data.server = node.advertiseService("/quadcopter/qr_position", &RTRQuadcopterController::QRPositionCallback, this);

  cameraT = ioBody->link("CAMERA_T");
  cameraT->setActuationMode(Link::JOINT_TORQUE);
  io->enableIO(cameraT);
  qref = qprev = cameraT->q();

  for (int i = 0; i < 4; i++)
  {
    prop[i] = ioBody->link(propname[i]);
    prop[i]->setActuationMode(Link::JOINT_TORQUE);
    io->enableInput(prop[i], JOINT_VELOCITY);
    io->enableOutput(prop[i]);

    rotor[i] = ioBody->findDevice<Multicopter::RotorDevice>(rotorname[i]);
    io->enableInput(rotor[i]);
  }

  isStableMode = true;
  prevModeButtonState = false;

  power = powerprev = false;

  // joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
  // targetMode = joystick->addMode();
  rotorswitch = false;

  cam = ioBody->findDevice<RangeCamera>("Camera");
  io->enableInput(cam);
  cam->on(true);
  cam->notifyStateChange();

  pub = node.advertise<PointCloud>("/quadcopter/output", 10);
  msg = PointCloud::Ptr(new PointCloud);
  msg->header.frame_id = "WRS";

  axis1 << 1, 0, 0;
  offset << 0, 0, 0.05;
  axis2 << 0, 0, 1;
  AxisAngle2 = AngleAxisd(M_PI / 2, axis2);
  mirror_xy << -1, 0, 0, 0, -1, 0, 0, 0, 1;

  joy_sub = node.subscribe<sensor_msgs::Joy>("/joy", 1, &RTRQuadcopterController::joyconCallback, this);

  return true;
}

void RTRQuadcopterController::calcPoint()
{
  static int count = 0;
  if (count++ * timeStep < 0.8)
  {
    return;
  }

  count = 0;

  auto p = cam->points();
  const auto q = cameraT->q();

  msg->points.clear();
  msg->points.reserve(p.size());

  AxisAngle1 = AngleAxisd(M_PI / 2 + q, axis1);
  const auto [xyz, dxyz, ddxyz, rpy, drpy, ddrpy] = imu_manager.get();
  Matrix3 rot = rotFromRpy(rpy);
  for (const auto &z : p)
  {
    const auto norm = z.norm();
    if (!isinf(norm) && !isnan(norm))
    {
      Vector3d poi;
      poi << z[0], z[1], z[2] + 0.03;

      const Vector3d relative = AxisAngle1 * poi + offset;
      if (0 < relative[2])
      {
        continue;
      }
      const Vector3d point = rot * mirror_xy * AxisAngle2 * relative + xyz;
      msg->points.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    }
  }
  msg->height = msg->points.size();
  msg->width = 1;
  printf("pcl_size: %lu\n", msg->points.size());

  pub.publish(msg);
}

bool RTRQuadcopterController::calcQRPoint()
{
  std::cout << "calcQRPoint" << std::endl;
  Vector3f point_raw_;
  try {
    point_raw_ = camera2_qr->points().at(qr_data.request.image_y * camera2_qr->resolutionX() + qr_data.request.image_x);
  } catch (const std::out_of_range& oor) {
    std::cout << "out of range " << oor.what() << std::endl;
    return false;
  } 
  Vector3d point_raw(point_raw_[0], point_raw_[1], point_raw_[2]);
  const auto q = cameraT->q();

  auto axis_angle1 = AngleAxisd(M_PI / 2 + q, axis1);
  const auto [xyz, dxyz, ddxyz, rpy, drpy, ddrpy] = imu_manager.get();
  Matrix3 rot = rotFromRpy(rpy);
  Vector3d translation;
  translation << 0.02, 0.0, -0.02;
  const auto norm = point_raw.norm();
  if (!isinf(norm) && !isnan(norm))
  {
    Vector3d poi = point_raw + translation;
    const Vector3d relative = axis_angle1 * poi + offset;
    if (0 < relative[2])
    {
      std::cout << "MINUS!!!!!" << std::endl;
      return false;
    }
    const Vector3d point = rot * mirror_xy * AxisAngle2 * relative + xyz;
    qr_data.response.qr_global_x = point[0];
    qr_data.response.qr_global_y = point[1];
    qr_data.response.qr_global_z = point[2];
    qr_data.has_request_processed = true;
    std::cout << "SUCCESS" << std::endl;
    return true;
  }
  std::cout << "NAN!!!!!" << std::endl;
  return false;
}

void RTRQuadcopterController::joyconCallback(const sensor_msgs::Joy joy)
{
  this->joy = joy;
  joy_received = true;
  last_joy_receive_time = std::chrono::system_clock::now();
}

bool RTRQuadcopterController::control()
{

  if (!joy_received)
  {
    return false;
  }
  auto now = std::chrono::system_clock::now();
  double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_joy_receive_time).count();

  if (elapsed_ms > JOY_TIMEOUT_MS)
  {
    for (int i = 0; i < 14; i++)
    {
      joy.buttons[i] = 0;
    }
    for (int i = 0; i < 11; i++)
    {
      joy.axes[i] = 0.0;
    }
  }
  const int BUTTON_X = 1;
  const int BUTTON_R_STICK = 11;

  // power = joystick->getButtonState(targetMode,
  //                                  powerButton);  // ローターのOn/Offの切り替え
  if (joy.buttons[JoyButton::X] == 1)
  {
    rotorswitch = true;
  }

  bool modeButtonState = (joy.buttons[JoyButton::PS] == 1); // StableModeの切り替え
  if (modeButtonState)
  { // Rスティックの押し込みで切り替えられる
    if (!prevModeButtonState)
    {
      isStableMode = !isStableMode;
    }
  }
  prevModeButtonState = modeButtonState;

  Vector4 force = Vector4::Zero();
  Vector4 torque = Vector4::Zero();

  if (rotorswitch)
  { // ローターが回ってたら色々処理、ここがメイン

    imu_manager.update();
    imu_manager.publish();
    imu_debugger.update();

    const auto [xyz, dxyz, ddxyz, rpy, drpy, ddrpy] = imu_manager.get();
    imu_debugger.compare(imu_manager.get());

    Vector2 xy = Vector2(xyz[0], xyz[1]);
    Vector2 dxy = Vector2(dxyz[0], dxyz[1]);
    Vector2 ddxy = Vector2(ddxyz[0], ddxyz[1]);
    Vector4 zrpy = Vector4(xyz[2], rpy[0], rpy[1], rpy[2]);
    Vector4 dzrpy = Vector4(dxyz[2], drpy[0], drpy[1], drpy[2]);
    Vector4 ddzrpy = Vector4(ddxyz[2], ddrpy[0], ddrpy[1], ddrpy[2]);

    double cc = cos(zrpy[1]) * cos(zrpy[2]);
    // cos(roll)*cos(pitch)
    double gfcoef = 1.0 * 9.80665 / 4 / cc;
    // ローター1つあたりの、自重を支えるために必要な力

    Vector4 f;

    // For the stable mode
    Vector2 dxy_local = Eigen::Rotation2Dd(-zrpy[3]) * dxy;
    // -yawの回転行列 * xy速度ベクトル
    // (現在のbodyの向きに合わせた座標軸での速度)
    Vector2 ddxy_local = Eigen::Rotation2Dd(-zrpy[3]) * ddxy;
    // -yawの回転行列 * xy加速度ベクトル
    // (現在のbodyの向きに合わせた座標軸での加速度)
    Vector4 zrpyref = Vector4::Zero();
    Vector4 dzrpyref = Vector4::Zero();
    Vector2 xyref = Vector2::Zero();
    Vector2 dxyref = Vector2::Zero();

    for (int i = 0; i < 4; ++i)
    {
      // double pos = joystick->getPosition(targetMode, rotorAxis[axis]);
      double pos = joy.axes[rotorAxis[i]];
      // posにそれぞれのスティックの値を入れる

      if ((rotorAxis[i] == JoyAxis::L_H) || (rotorAxis[i] == JoyAxis::L_V))
      {
        // Lスティック
        if (fabs(pos) > 0.25)
        {
          // スティックの微小な傾きを無視するためのしきい値
          dzrpyref[i] = RATE[i] * -pos;
          // RATEで入力をスケーリングとか、向きの設定をしてる
        }
        else
        {
          dzrpyref[i] = 0.0;
        } // 次の行でそれぞれのaxisに対して速度を対象にPDしてる
        f[i] = KP[i] * (dzrpyref[i] - dzrpy[i]) +
               KD[i] * (0.0 - ddzrpy[i]);
      }
      else
      {
        // ここから　Rスティックの処理
        // で縦方向
        if (!isStableMode)
        { // StableModeじゃないとき
          if (fabs(pos) > 0.25)
          {
            zrpyref[i] = RATE[i] * -pos; // Lスティックと同じ
          }
          else
          {
            zrpyref[i] = 0.0;
          }
        }
        else
        {                      // StableModeのとき
          int axis_xy = i - 1; // Rスティックの横: 0, 縦: 1の値をとる
          if (fabs(pos) > 0.25)
          {
            dxyref[axis_xy] = RATEX[axis_xy] * -pos; // RATEXでスケーリング
          }
          else
          {
            dxyref[axis_xy] = 0.0;
          }
          zrpyref
              [i] = // KPX,KDXを使ってスケールされたスティック入力の値を目標速度としてPDする
              KPX[axis_xy] * (dxyref[axis_xy] - dxy_local[1 - axis_xy]) +
              KDX[axis_xy] * (0.0 - ddxy_local[1 - axis_xy]);

          if (rotorAxis[i] == JoyAxis::R_H)
          {
            zrpyref[i] *=
                -1.0; // スティックの値が増える向きと、X軸の向きが逆だから直してるんだと思う
          }
        } // 得られたzrpyrefで位置を対象にPDする
        f[i] = KP[i] * (zrpyref[i] - zrpy[i]) +
               KD[i] * (0.0 - dzrpy[i]);
      }
    }

    for (int i = 0; i < 4; ++i)
    { // signを使ってそれぞれの出力を決める
      double fi = 0.0;
      fi += gfcoef;
      fi += sign[i][0] * f[0];
      fi += sign[i][1] * f[1];
      fi += sign[i][2] * f[2];
      fi += sign[i][3] * f[3];
      force[i] = fi;
      // forceは推力、torqueはトルク　この2つを出力として設定する。
      torque[i] = dir[i] * fi;
      // ここdirで片側2つを-1倍してるのはモーメントの計算のために力をすべて同じ回転方向で考えていたからかな?
    }

    if (joy.buttons[JoyButton::R1] == 1)
    {
      calcPoint();
    }
    if(qr_data.has_request_processed == false)
    {
      calcQRPoint();
    }
  }

  for (int i = 0; i < 4; ++i)
  {
    double tau = torque[i];
    rotor[i]->setTorque(tau); // トルクを設定する
    if (tau != 0.0)
    { // プロペラを回す処理
      prop[i]->u() = tau * 0.001;
    }
    else
    {
      double dq = prop[i]->dq();
      prop[i]->u() = 0.0005 * (0.0 - dq);
    }
    rotor[i]->setValue(force[i]); // 推力を設定
    rotor[i]->notifyStateChange();
    // 入力した推力とトルクをシミュレーションに反映
  }

  // control camera
  double q = cameraT->q(); // カメラを十字キーで動かせるようにしてる
  static const double P = 0.00002;
  static const double D = 0.00004;
  double dq = (q - qprev) / timeStep;
  double pos = joy.axes[CAMERA_AXIS];
  // double pos = joystick->getPosition(targetMode, cameraAxis) * -1.0;
  double dqref = 0.0;
  if (fabs(pos) > 0.25)
  {
    double deltaq = 0.002 * pos;
    qref += deltaq;
    if (qref > 0)
    {
      qref = 0.0;
    }
    else if (qref < -M_PI)
    {
      qref = -M_PI;
    }
    dqref = deltaq / timeStep;
  }
  cameraT->u() = P * (qref - q) + D * (dqref - dq);
  qprev = q;
  if (joy.axes[JoyAxis::DIR_PAD_H] > 0.25 )
  { //カメラの角度範囲（フォーカス）変更
    fieldOfView += 0.004;
    if (fieldOfView > 1.64)
    {
      fieldOfView = 1.64;
    }
    camera2->setFieldOfView(fieldOfView);
    camera2->notifyStateChange();
  }
  if (joy.axes[JoyAxis::DIR_PAD_H] < -0.25 )
  {
    fieldOfView -= 0.004;
    if (fieldOfView < 0.002)
    {
      fieldOfView = 0.002;
    }
    camera2->setFieldOfView(fieldOfView);
    camera2->notifyStateChange();
  }
  if (joy.buttons[JoyButton::TRIANGLE] == 1)
  {
    fieldOfView = 0.785398;
    camera2->setFieldOfView(fieldOfView);
    camera2->notifyStateChange();
  }
  return true;
}

bool RTRQuadcopterController::QRPositionCallback(rtr_quadcopter_controller::QRPosition::Request &req, rtr_quadcopter_controller::QRPosition::Response &res)
    {
        std::cout << "QRPositionCallback" << std::endl;
        qr_data.has_request_processed = false;
        qr_data.request = req;
        // カメラを起動
        camera2_qr->setResolutionX(camera2->resolutionX());
        camera2_qr->setResolutionY(camera2->resolutionY());
        camera2_qr->setFieldOfView(camera2->fieldOfView());
        camera2_qr->setNearClipDistance(camera2->nearClipDistance());
        camera2_qr->setFarClipDistance(camera2->farClipDistance());
        camera2_qr->on(true);
        camera2_qr->notifyStateChange();

         ros::Duration interval(0, 100000000); // 0.1 seconds
        while(qr_data.has_request_processed == false){
            interval.sleep();
        }
        res = qr_data.response;
        // カメラを停止
        camera2_qr->on(false);
        camera2_qr->notifyStateChange();
        std::cout << "QRPositionCallback" << std::endl;
        return true;
    }

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RTRQuadcopterController)
