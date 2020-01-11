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

#include <cnoid/AccelerationSensor>  //センサ毎に必要
#include <cnoid/Camera>
#include <cnoid/EigenUtil>
#include <cnoid/RangeCamera>
#include <cnoid/RateGyroSensor>
#include <cnoid/RotorDevice>
#include <cnoid/SharedJoystick>
#include <cnoid/SimpleController>
#include <iostream>

#include "imu_debugger.hpp"
#include "imu_filter.h"
#include "imu_manager.hpp"

#include "rgbd_camera.hpp"

using namespace cnoid;

namespace {

const std::string propname[] = {"PROP0", "PROP1", "PROP2", "PROP3"};
const std::string rotorname[] = {"RotorDevice0", "RotorDevice1", "RotorDevice2",
                                 "RotorDevice3"};

constexpr int rotorAxis[] = {Joystick::L_STICK_V_AXIS,  // Vはvertical
                             Joystick::R_STICK_H_AXIS,  // Hはhorizontal 多分
                             Joystick::R_STICK_V_AXIS,
                             Joystick::L_STICK_H_AXIS};

constexpr int cameraAxis = Joystick::DIRECTIONAL_PAD_V_AXIS;
constexpr int powerButton = Joystick::A_BUTTON;
constexpr int cameraFocusUp = Joystick::X_BUTTON;
constexpr int cameraFocusDown = Joystick::B_BUTTON;
constexpr int cameraFocusReset = Joystick::Y_BUTTON;
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

//using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class QuadcopterControllerWRS : public SimpleController {
 public:
  SharedJoystickPtr joystick;
  int targetMode;
  BodyPtr ioBody;
  std::ostream* os;
  Link* prop[4];
  Multicopter::RotorDevice* rotor[4];
  Link* cameraT;
  Camera* camera2;

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

  virtual bool initialize(SimpleControllerIO* io) override;
  virtual bool control() override;

  ros::NodeHandle node;

  rgbd_camera::RGBDCameraManager rgbd_camera_manager_;

  Vector3d offset;

  Matrix3d AxisAngle1;
  Vector3d axis1;

  Matrix3d AxisAngle2;
  Vector3d axis2;

  Matrix3d mirror_xy;

  double fieldOfView;
  unsigned int wait_camera = 0;
};

}  // namespace

bool QuadcopterControllerWRS::initialize(SimpleControllerIO* io) {
  ioBody = io->body();
  os = &io->os();
  timeStep = io->timeStep();
  io->enableInput(ioBody->rootLink(), LINK_POSITION);

  imu_manager.init("AccelerationSensor", "RateGyroSensor", io, timeStep,
                   node.advertise<geometry_msgs::Pose>("/quadcopter/quadcopter_pose", 10));
  imu_debugger.init(ioBody, timeStep);
  imu_manager.set(imu_debugger.get());

  camera2 = ioBody->findDevice<Camera>("Camera2");  //取得
  io->enableInput(camera2);
  camera2->setResolution(1280, 720);  //解像度変更
  fieldOfView = 0.785398;
  camera2->setFieldOfView(fieldOfView);
  camera2->notifyStateChange();

  cameraT = ioBody->link("CAMERA_T");
  cameraT->setActuationMode(Link::JOINT_TORQUE);
  io->enableIO(cameraT);
  qref = qprev = cameraT->q();

  for (int i = 0; i < 4; i++) {
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

  joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
  targetMode = joystick->addMode();
  rotorswitch = false;

  rgbd_camera_manager_.initCnoidDevice("Camera",io);
  rgbd_camera_manager_.initROS("/quadcopter/cloud",node);

  axis1 << 1, 0, 0;
  offset << 0, 0, 0.05;
  axis2 << 0, 0, 1;
  AxisAngle2 = AngleAxisd(M_PI / 2, axis2);
  mirror_xy << -1, 0, 0, 0, -1, 0, 0, 0, 1;

  return true;
}


bool QuadcopterControllerWRS::control() {
  joystick->updateState(targetMode);  // ジョイコンのための謎の処理

  if (wait_camera == 0 || 20 < wait_camera) {
    wait_camera = 0;
  } else {
    wait_camera++;
  }

  power = joystick->getButtonState(targetMode,
                                   powerButton);  // ローターのOn/Offの切り替え
  if (power == true) {
    rotorswitch = true;
  }

  bool modeButtonState = joystick->getButtonState(
      Joystick::R_STICK_BUTTON);  // StableModeの切り替え
  if (modeButtonState) {  // Rスティックの押し込みで切り替えられる
    if (!prevModeButtonState) {
      isStableMode = !isStableMode;
    }
  }
  prevModeButtonState = modeButtonState;

  Vector4 force = Vector4::Zero();
  Vector4 torque = Vector4::Zero();

  if (rotorswitch) {  // ローターが回ってたら色々処理、ここがメイン

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

    for (int axis = 0; axis < 4; ++axis) {
      double pos = joystick->getPosition(targetMode, rotorAxis[axis]);
      // posにそれぞれのスティックの値を入れる

      if ((axis == 0) || (axis == 3)) {
        // Lスティックについてで、axis == 0が縦方向、 axis == 3 が横方向
        if (fabs(pos) > 0.25) {
          // スティックの微小な傾きを無視するためのしきい値
          dzrpyref[axis] = RATE[axis] * pos;
          // RATEで入力をスケーリングとか、向きの設定をしてる
        } else {
          dzrpyref[axis] = 0.0;
        }  // 次の行でそれぞれのaxisに対して速度を対象にPDしてる
        f[axis] = KP[axis] * (dzrpyref[axis] - dzrpy[axis]) +
                  KD[axis] * (0.0 - ddzrpy[axis]);
      } else {
        // ここから　Rスティックの処理 axis == 1で横方向、 axis == 2
        // で縦方向
        if (!isStableMode) {  // StableModeじゃないとき
          if (fabs(pos) > 0.25) {
            zrpyref[axis] = RATE[axis] * pos;  // Lスティックと同じ
          } else {
            zrpyref[axis] = 0.0;
          }
        } else {                   // StableModeのとき
          int axis_xy = axis - 1;  // Rスティックの横: 0, 縦: 1の値をとる
          if (fabs(pos) > 0.25) {
            dxyref[axis_xy] = RATEX[axis_xy] * pos;  // RATEXでスケーリング
          } else {
            dxyref[axis_xy] = 0.0;
          }
          zrpyref
              [axis] =  // KPX,KDXを使ってスケールされたスティック入力の値を目標速度としてPDする
              KPX[axis_xy] * (dxyref[axis_xy] - dxy_local[1 - axis_xy]) +
              KDX[axis_xy] * (0.0 - ddxy_local[1 - axis_xy]);

          if (axis == 1) {
            zrpyref[axis] *=
                -1.0;  // スティックの値が増える向きと、X軸の向きが逆だから直してるんだと思う
          }
        }  // 得られたzrpyrefで位置を対象にPDする
        f[axis] = KP[axis] * (zrpyref[axis] - zrpy[axis]) +
                  KD[axis] * (0.0 - dzrpy[axis]);
      }
    }

    for (int i = 0; i < 4; ++i) {  // signを使ってそれぞれの出力を決める
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

//     if (joystick->getButtonState(targetMode, Joystick::R_BUTTON)) {
        rgbd_camera_manager_.publishCloud(ioBody,cameraT,&imu_manager,timeStep);
//     }
  }
//  rgbd_camera_manager_.publishCloud(ioBody,cameraT,imu_manager,timeStep);

  for (int i = 0; i < 4; ++i) {
    double tau = torque[i];
    rotor[i]->setTorque(tau);  // トルクを設定する
    if (tau != 0.0) {          // プロペラを回す処理
      prop[i]->u() = tau * 0.001;
    } else {
      double dq = prop[i]->dq();
      prop[i]->u() = 0.0005 * (0.0 - dq);
    }
    rotor[i]->setValue(force[i]);  // 推力を設定
    rotor[i]->notifyStateChange();
    // 入力した推力とトルクをシミュレーションに反映
  }

  // control camera
  double q = cameraT->q();  // カメラを十字キーで動かせるようにしてる
  static const double P = 0.00002;
  static const double D = 0.00004;
  double dq = (q - qprev) / timeStep;
  double pos = joystick->getPosition(targetMode, cameraAxis) * -1.0;
  double dqref = 0.0;
  if (fabs(pos) > 0.25) {
    double deltaq = 0.002 * pos;
    qref += deltaq;
    if (qref > 0) {
      qref = 0.0;
    } else if (qref < -M_PI) {
      qref = -M_PI;
    }
    dqref = deltaq / timeStep;
  }
  cameraT->u() = P * (qref - q) + D * (dqref - dq);
  qprev = q;
  if (joystick->getButtonState(targetMode, cameraFocusUp) &&
      wait_camera == 0) {  //カメラの角度範囲（フォーカス）変更
    fieldOfView += 0.02;
    if (fieldOfView > 1.64) {
      fieldOfView = 1.64;
    }
    camera2->setFieldOfView(fieldOfView);
    camera2->notifyStateChange();
    wait_camera = 1;
  }
  if (joystick->getButtonState(targetMode, cameraFocusDown) &&
      wait_camera == 0) {
    fieldOfView -= 0.02;
    if (fieldOfView < 0.003) {
      fieldOfView = 0.003;
    }
    camera2->setFieldOfView(fieldOfView);
    camera2->notifyStateChange();
    wait_camera = 1;
  }
  if (joystick->getButtonState(targetMode, cameraFocusReset)) {
    fieldOfView = 0.785398;
    camera2->setFieldOfView(fieldOfView);
    camera2->notifyStateChange();
  }
  return true;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(QuadcopterControllerWRS)
