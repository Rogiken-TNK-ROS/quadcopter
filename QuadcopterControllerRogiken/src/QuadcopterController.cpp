/**
   Copyright (c) 2018 Japan Atomic Energy Agency (JAEA).
   The original version is implemented as an RT-component.
   This is a simple controller version modified by AIST.
*/

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

#include "imu_filter.h"

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

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class QuadcopterController : public SimpleController {
 public:
  SharedJoystickPtr joystick;
  int targetMode;
  BodyPtr ioBody;
  std::ostream* os;
  Link* prop[4];
  Multicopter::RotorDevice* rotor[4];
  Link* cameraT;
  double timeStep;
  AccelerationSensor* accelSensor;
  RateGyroSensor* rateGyro;
  Camera* camera2;

  Vector4 zrpyref;
  Vector4 zrpyprev;
  Vector4 dzrpyref;
  Vector4 dzrpyprev;

  Vector3 velocity;  //速度ベクトル（加速度センサ用）
  Vector3 xyz_;
  Vector3 gyro;
  // Vector3 prerpy;
  Vector3 rpyFromGyro;
  Vector3 mod;
  // Vector3 wt;
  // Matrix3 rotation;

  // For the stable mode
  bool isStableMode;
  bool prevModeButtonState;
  Vector2 xyref;
  Vector2 xyprev;
  Vector2 dxyref;
  Vector2 dxyprev;

  double qref;
  double qprev;
  bool power;
  bool powerprev;
  bool rotorswitch;

  ImuFilter filter;

  virtual bool initialize(SimpleControllerIO* io) override;
  Vector4 getZRPY();
  Vector2 getXY();
  Vector3 getdv();
  void calcPoint();
  virtual bool control() override;

  RangeCamera* cam;
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
};

}  // namespace

bool QuadcopterController::initialize(SimpleControllerIO* io) {
  ioBody = io->body();
  os = &io->os();
  timeStep = io->timeStep();
  std::cout << "timeStep: " << timeStep << std::endl;
  io->enableInput(ioBody->rootLink(), LINK_POSITION);

  // 加速度センサ関係初期化
  accelSensor = ioBody->findDevice<AccelerationSensor>("AccelerationSensor");
  if (accelSensor == nullptr) {
    std::cout << "AccelerationSensor is not found" << std::endl;
  }
  accelSensor->on(true);
  io->enableInput(accelSensor);  // 読み取り許可

  // ジャイロ初期化
  rateGyro = ioBody->findDevice<RateGyroSensor>("RateGyroSensor");
  if (rateGyro == nullptr) {
    std::cout << "RateGyroSensor is not found" << std::endl;
  }
  rateGyro->on(true);
  io->enableInput(rateGyro);  // 読み取り許可
  gyro = Vector3::Zero();     // 角速度ベクトル初期化

  velocity = Vector3::Zero();
  rpyFromGyro = Vector3::Zero();
  mod = Vector3::Zero();
  xyz_ << 1, 0, 0;
  // wt = Vector3::Zero();
  // rotation = ioBody->rootLink()->position().rotation();
  //カメラ関連//
  camera2 = ioBody->findDevice<Camera>("Camera2");  //取得
  io->enableInput(camera2);
  camera2->setResolution(1280, 720);  //解像度変更
  fieldOfView = 0.785398;
  camera2->setFieldOfView(fieldOfView);
  camera2->notifyStateChange();
  //ここまで//
  //ここまで//

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

  zrpyref = zrpyprev = getZRPY();
  dzrpyref = dzrpyprev = Vector4::Zero();

  // isStableMode = false;
  isStableMode = true;
  prevModeButtonState = false;
  xyref = xyprev = getXY();
  dxyref = dxyprev = Vector2::Zero();

  power = powerprev = false;

  joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
  targetMode = joystick->addMode();
  rotorswitch = false;

  filter.setOrientation(1, 0, 0, 0);

  cam = ioBody->findDevice<RangeCamera>("Camera");
  io->enableInput(cam);
  cam->on(true);
  cam->notifyStateChange();

  pub = node.advertise<PointCloud>("output", 10);
  msg = PointCloud::Ptr(new PointCloud);
  msg->header.frame_id = "WRS";

  axis1 << 1, 0, 0;
  offset << 0, 0, 0.05;
  axis2 << 0, 0, 1;
  AxisAngle2 = AngleAxisd(M_PI / 2, axis2);
  mirror_xy << -1, 0, 0, 0, -1, 0, 0, 0, 1;

  return true;
}

Vector4 QuadcopterController::getZRPY() {
  auto T = ioBody->rootLink()->position();
  double z = T.translation().z();
  Vector3 rpy =
      rpyFromRot(T.rotation());  // rotation から roll, pitch, yawを得る
  return Vector4(
      z, rpy[0], rpy[1],
      rpy[2]);  // ioBodyの座標をVector4(z, roll, pitch, yaw)の形式で返す
}

Vector2 QuadcopterController::getXY() {
  auto p = ioBody->rootLink()
               ->translation();  // 位置成分に対応する３次元ベクトルを得る。
  return Vector2(p.x(), p.y());  // ioBodyの座標をVector2(x, y)の形式で返す
}

// Eigen::Matrix3d OmegaMatrix(Vector3 rpy)
// {
//     double phi = rpy[0];
//     double theta = rpy[1];
//     double psy = rpy[2];
//     Eigen::Matrix3d mat;
//     mat << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
//             0, cos(phi), -sin(phi),
//             0, sin(phi)/cos(theta), cos(phi)/cos(theta);
//     return mat;
// }

void QuadcopterController::calcPoint() {
  static int count = 0;
  if (count++ * timeStep < 0.8) {
    return;
  }

  count = 0;

  auto root = ioBody->rootLink()->position().translation();
  auto camera =
      ioBody->findDevice<Camera>("Camera")->link()->position().translation();

  // printf("id:%d\n", cam->id());
  // printf("name:%s\n", cam->name().c_str());
  auto p = cam->points();
  const auto q = cameraT->q();
  // printf("num: %d\n", p.size());
  // printf("on/off: %d\n", cam->on());

  msg->points.clear();
  msg->points.reserve(p.size());
  // msg->points.reserve(p.size() + msg->points.size());

  AxisAngle1 = AngleAxisd(M_PI / 2 + q, axis1);

  for (const auto& z : p) {
    const auto norm = z.norm();
    if (!isinf(norm) && !isnan(norm)) {
      Vector3d poi;
      poi << z[0], z[1], z[2] + 0.03;

      const Vector3d relative = AxisAngle1 * poi + offset;
      if (0 < relative[2]) {
        continue;
      }
      const Vector3d point =
          rotFromRpy(mod) * mirror_xy * AxisAngle2 * relative + xyz_;
      // const auto point = rotFromRpy(mod) * mirror_xy * AxisAngle2 * relative
      // + xyz_;
      // const auto point = AxisAngle * z;
      msg->points.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    }
  }
  msg->height = msg->points.size();
  msg->width = 1;
  printf("pcl_size: %d\n", msg->points.size());

  pub.publish(msg);
  // Camera *c = ioBody->findDevice<Camera>("Camera");
  // c->constImage().save("po.png");
  // printf("root: %f, %f, %f\ncamera: %f, %f, %f\n", root[0], root[1], root[2],
  // camera[0], camera[1], camera[2]);
}

bool QuadcopterController::control() {
  joystick->updateState(targetMode);  // ジョイコンのための謎の処理

  if (wait_camera == 0 || 20 < wait_camera) {
    wait_camera = 0;
  } else {
    wait_camera++;
  }

  // control rotors
  Vector4 zrpy = getZRPY();
  double cc = cos(zrpy[1]) * cos(zrpy[2]);  // cos(roll)*cos(pitch)
  double gfcoef = 1.0 * 9.80665 / 4 /
                  cc;  // ローター1つあたりの、自重を支えるために必要な力
  Vector4 force = Vector4::Zero();
  Vector4 torque = Vector4::Zero();

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

  gyro = rateGyro->w();
  Vector3 a = accelSensor->dv();

  filter.madgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], a[0], a[1], a[2],
                               timeStep);
  double q0, q1, q2, q3;
  filter.getOrientation(q0, q1, q2, q3);
  Quaterniond quat = Quaterniond(q0, q1, q2, q3);
  mod = rpyFromRot(quat.matrix());
  // フィルタによる姿勢角推定.

  if (rotorswitch) {  // ローターが回ってたら色々処理、ここがメイン
    Vector4 f;

    Vector4 dzrpy = (zrpy - zrpyprev) /
                    timeStep;  // zrpyの一階微分(z座標軸の速度、rpyの角速度)
    Vector4 ddzrpy =
        (dzrpy - dzrpyprev) /
        timeStep;  // zrpyの二階微分(z座標軸の加速度、rpyの角加速度)

    // gyro
    Vector3 rpy = Vector3(zrpy[1], zrpy[2], zrpy[3]);

    //加速度センサ関連//
    Vector3 dv =
        rotFromRpy(mod) *
        a;  // rpyはまだ補正できていないので、とりあえずシステムのやつを使っている
    // dv[0] *= -1; dv[1] *= -1;
    dv[2] -= 9.80665;  //重力加速度引く
    velocity += dv * timeStep;
    xyz_ += velocity * timeStep;  // 加速度センサから得た絶対座標

    //ここまで//

    // For the stable mode
    Vector2 xy = getXY();                    // xy座標
    Vector2 dxy = (xy - xyprev) / timeStep;  // xy座標の一階微分(xyの速度)
    Vector2 ddxy = (dxy - dxyprev) / timeStep;  // xy座標二階微分(xyの加速度)
    Vector2 dxy_local = Eigen::Rotation2Dd(-zrpy[3]) *
                        dxy;  // -yawの回転行列 * xy速度ベクトル
                              // (現在のbodyの向きに合わせた座標軸での速度)
    Vector2 ddxy_local = Eigen::Rotation2Dd(-zrpy[3]) *
                         ddxy;  // -yawの回転行列 * xy加速度ベクトル
                                // (現在のbodyの向きに合わせた座標軸での加速度)

    printf("diff_x: %+8.6f, %+8.6f, %+8.6f,\t%+8.6f, %+8.6f, %+8.6f\n",
           xy[0] - xyz_[0], xy[1] - xyz_[1], zrpy[0] - xyz_[2], xy[0], xy[1],
           zrpy[0]);
    printf("diff: %+8.6f, %+8.6f, %+8.6f\n\n", zrpy[1] - mod[0],
           zrpy[2] - mod[1], zrpy[3] - mod[2]);

    //以下、加速度センサの値使用
    // Vector2 ddxy_local = Vector2(dv[0], dv[1]);
    // Vector2 dxy_local = Vector2(velocity[0], velocity[1]);
    // Vector2 dxy = Eigen::Rotation2Dd(zrpy[3]) * dxy_local;
    // //yawの回転行列
    // * xy速度ベクトル Vector2 ddxy = Eigen::Rotation2Dd(zrpy[3]) * ddxy_local;

    for (int axis = 0; axis < 4; ++axis) {
      double pos = joystick->getPosition(
          targetMode,
          rotorAxis[axis]);  // posにそれぞれのスティックの値を入れる

      if ((axis == 0) || (axis == 3)) {  // Lスティックについてで、axis == 0
                                         // が縦方向、 axis == 3 が横方向
        if (fabs(pos) >
            0.25) {  // スティックの微小な傾きを無視するためのしきい値
          dzrpyref[axis] =
              RATE[axis] *
              pos;  // RATEで入力をスケーリングとか、向きの設定をしてる
        } else {
          dzrpyref[axis] = 0.0;
        }  // 次の行でそれぞれのaxisに対して速度を対象にPDしてる
        f[axis] = KP[axis] * (dzrpyref[axis] - dzrpy[axis]) +
                  KD[axis] * (0.0 - ddzrpy[axis]);
      } else {  // ここから　Rスティックの処理 axis == 1で横方向、 axis == 2
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
    zrpyprev = zrpy;  // 次のサイクルのために保存
    dzrpyprev = dzrpy;

    // For the stable mode
    xyprev = xy;  // 同上
    dxyprev = dxy;

    for (int i = 0; i < 4; ++i) {  // signを使ってそれぞれの出力を決める
      double fi = 0.0;
      fi += gfcoef;
      fi += sign[i][0] * f[0];
      fi += sign[i][1] * f[1];
      fi += sign[i][2] * f[2];
      fi += sign[i][3] * f[3];
      force[i] =
          fi;  // forceは推力、torqueはトルク　この2つを出力として設定する。
      torque[i] =
          dir[i] *
          fi;  // ここdirで片側2つを-1倍してるのはモーメントの計算のために力をすべて同じ回転方向で考えていたからかな?
    }

    if (joystick->getButtonState(targetMode, Joystick::R_BUTTON)) {
      calcPoint();
    }
  }

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
    rotor[i]
        ->notifyStateChange();  // 入力した推力とトルクをシミュレーションに反映
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

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(QuadcopterController)
