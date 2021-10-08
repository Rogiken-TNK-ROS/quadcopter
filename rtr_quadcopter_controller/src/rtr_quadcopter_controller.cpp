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
#include <cnoid/Light>
#include <cnoid/RangeSensor>
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
#include "rtr_msgs/QRPosition.h"

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

  enum ControllableCameraID{
    CAMERA_2 = 0,
    CAMERA_3 = 1,
    NUM = 2,
  };

  constexpr int rotorAxis[4] = {JoyAxis::L_V, JoyAxis::R_H, JoyAxis::R_V, JoyAxis::L_H};

  constexpr int CAMERA_AXIS = JoyAxis::DIR_PAD_V;
  constexpr int POWER_BTN = JoyButton::X;
  constexpr int ZOOM_IN_BTN = JoyButton::CIRCLE;
  constexpr int ZOOM_OUT_BTN = JoyButton::SQUARE;
  constexpr int ZOOM_RESET_BTN = JoyButton::TRIANGLE;
  constexpr int LIGHT_BRIGHTEN_BTN = JoyButton::L1;
  constexpr int LIGHT_DARKEN_BTN = JoyButton::L2;

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
  rtr_msgs::QRPosition::Request request;
  rtr_msgs::QRPosition::Response response;
};

struct JointData{
  Link *link;
  double qref;
  double qprev;
  double upper_limit;
  double lower_limit;
  void enable(std::string name,BodyPtr body,SimpleControllerIO *io){
    link = body->link(name.c_str());
    link->setActuationMode(Link::JOINT_TORQUE);
    io->enableIO(link);
    qref = qprev = link->q();
  }
};

template <typename CameraT>
struct CameraData{
  CameraT *camera;
  double fov;
  void enable(std::string name, BodyPtr body, SimpleControllerIO *io, double fov_in = -1){
    camera = body->findDevice<CameraT>(name.c_str());
    io->enableInput(camera);
    if(fov_in > 0){
      fov = fov_in;
    }else{
      fov = camera->fieldOfView();
    }
    camera->setFieldOfView(fov);
    camera->notifyStateChange();
  }
};

template <typename CameraT>
struct CameraWithJointData{
  JointData joint;
  CameraData<CameraT> camera;
};

struct Button{
  JoyButton button_id;
  bool prev_state = false;
  bool is_on = false;
  bool update(sensor_msgs::Joy joy){
    bool state = (joy.buttons[button_id] == 1);
    if(state < prev_state){
      is_on = !is_on;
    }
    prev_state = state;
    return is_on;
  }
};

struct LightSource{
  Light* light;
  void enable(std::string name, BodyPtr body, SimpleControllerIO *io, double init_intensity){
    light = body->findDevice<Light>(name.c_str());
    io->enableInput(light);
    light->setIntensity(init_intensity);
    light->notifyStateChange();
  }
  void brighten(double sp = 0.005){
    double intensity = light->intensity();
    if (intensity<1.0-sp){
      intensity+=sp;
    }
    light->setIntensity(intensity);
    light->notifyStateChange();
  }
  void darken(double sp = 0.005){
    double intensity = light->intensity();
    if (intensity>0.0+sp){
      intensity-=sp;
    }
    light->setIntensity(intensity);
    light->notifyStateChange();
  }
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

    CameraWithJointData<Camera> cam2;
    CameraWithJointData<Camera> cam3;
    CameraData<RangeCamera> cam2_qr;

    LightSource light[4];

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

    // double qref;
    // double qprev;
    bool power;
    bool powerprev;
    bool rotorswitch;

    virtual bool initialize(SimpleControllerIO *io) override;
    using QRReq = rtr_msgs::QRPosition::Request;
    using QRRes = rtr_msgs::QRPosition::Response;
    bool QRPositionCallback(QRReq &req, QRRes &res);
    void joyconCallback(const sensor_msgs::Joy);
    virtual bool control() override;
    template<typename CameraT>
    void controlFov(CameraData<CameraT>& camera);
    void controlJoint(JointData& joint,double input);

    RangeCamera *cam;
    RangeSensor *range;

    ros::NodeHandle node;
    ros::Publisher pub;
    PointCloud::Ptr msg;

    Vector3d offset;

    Matrix3d AxisAngle1;
    Vector3d axis1;

    Matrix3d AxisAngle2;
    Vector3d axis2;

    Matrix3d mirror_xy;


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

  cam2.camera.enable("Camera2",ioBody,io);
  cam3.camera.enable("Camera3",ioBody,io);

  cam2_qr.enable("Camera2_QR",ioBody,io);

  qr_data.server = node.advertiseService("/quadcopter/qr_position", &RTRQuadcopterController::QRPositionCallback, this);

  cam2.joint.enable("CAMERA_2",ioBody,io);
  cam2.joint.lower_limit = -M_PI;
  cam2.joint.upper_limit = 0.0;
  cam3.joint.enable("CAMERA_3",ioBody,io);
  cam3.joint.lower_limit = 0.0;
  cam3.joint.upper_limit = M_PI;

  // 光源オブジェクト
  light[0].enable("Light_main1",ioBody,io,0.7); // 第4引数は初期の光強度値 [0,1]
  light[1].enable("Light_main2",ioBody,io,0.7);
  light[2].enable("Light_for_Camera2",ioBody,io,0.7);
  light[3].enable("Light_for_Camera3",ioBody,io,0.7);

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
//  prevModeButtonState = false;

  power = powerprev = false;

  // joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
  // targetMode = joystick->addMode();
  rotorswitch = false;

  cam = ioBody->findDevice<RangeCamera>("Camera");
  io->enableInput(cam);
  cam->on(true);
  cam->notifyStateChange();

  pub = node.advertise<PointCloud>("/quadcopter/output", 1);
  msg = PointCloud::Ptr(new PointCloud);
  msg->header.frame_id = "WRS";

  axis1 << 1, 0, 0;
  offset << 0, 0, 0.05;
  axis2 << 0, 0, 1;
  AxisAngle2 = AngleAxisd(M_PI / 2, axis2);
  mirror_xy << -1, 0, 0, 0, -1, 0, 0, 0, 1;

  joy_sub = node.subscribe<sensor_msgs::Joy>("/quadcopter/joy", 1, &RTRQuadcopterController::joyconCallback, this);

  range = ioBody->findDevice<RangeSensor>("UpperRangeSensor");
  io->enableInput(range);
  range->on(true);
  range->notifyStateChange();
  
  return true;
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

  static Button rotor_button;
  rotor_button.button_id = JoyButton::X;
  rotorswitch = rotor_button.update(joy);

  static Button stable_button;
  stable_button.button_id = JoyButton::R_STICK;
  isStableMode = rotor_button.update(joy);

  static Button camera_switch_button;
  camera_switch_button.button_id = JoyButton::PS;
  bool camera3_on = camera_switch_button.update(joy);

  ControllableCameraID current_control_cam;
  if(camera3_on){
    current_control_cam = ControllableCameraID::CAMERA_3;
  }else{
    current_control_cam = ControllableCameraID::CAMERA_2;
  }

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
    double gfcoef = ioBody->mass() * 9.80665 / 4 / cc;
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
        }
        if (abs(ddzrpy[i])<1e-4)
        { // 不感帯の設定
          ddzrpy[i] = 0;
        }
        // 次の行でそれぞれのaxisに対して速度を対象にPDしてる
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

  switch(current_control_cam){
    case ControllableCameraID::CAMERA_2:
      controlJoint(cam2.joint,joy.axes[CAMERA_AXIS]);
      controlJoint(cam3.joint,0.0);
      controlFov(cam2.camera);
      break;
    case ControllableCameraID::CAMERA_3:
      controlJoint(cam2.joint,0.0);
      controlJoint(cam3.joint,joy.axes[CAMERA_AXIS]);
      controlFov(cam3.camera);
      break;
  }

  // ライト操作
  if (joy.buttons[JoyButton::R1]) {
    for (int i=0; i<4; i++){
      light[i].brighten();
    }
  }
  if (joy.buttons[JoyButton::L1]) {
    for (int i=0; i<4; i++){
      light[i].darken();
    }
  }
  return true;
}

void RTRQuadcopterController::controlJoint(JointData& joint,double input){
  double q = joint.link->q();
  static const double P = 0.00002;
  static const double D = 0.00004;
  double dq = (q - joint.qprev) / timeStep;

  double dqref = 0.0;
  if (fabs(input) > 0.25)
  {
    double deltaq = 0.002 * input;
    joint.qref += deltaq;
    if (joint.qref > joint.upper_limit)
    {
      joint.qref = joint.upper_limit;
    }
    else if (joint.qref < joint.lower_limit)
    {
      joint.qref = joint.lower_limit;
    }
    dqref = deltaq / timeStep;
  }
  joint.link->u() = P * (joint.qref - q) + D * (dqref - dq);
  joint.qprev = q;
}
template <typename CameraT>
void RTRQuadcopterController::controlFov(CameraData<CameraT>& camera){

  if (joy.axes[JoyAxis::DIR_PAD_H] > 0.25 )
  { //カメラの角度範囲（フォーカス）変更
    camera.fov += 0.004;
    if (camera.fov > 1.64)
    {
      camera.fov = 1.64;
    }
    camera.camera->setFieldOfView(camera.fov);
    camera.camera->notifyStateChange();
  }
  if (joy.axes[JoyAxis::DIR_PAD_H] < -0.25 )
  {
    camera.fov -= 0.004;
    if (camera.fov < 0.002)
    {
      camera.fov = 0.002;
    }
    camera.camera->setFieldOfView(camera.fov);
    camera.camera->notifyStateChange();
  }
  if (joy.buttons[JoyButton::TRIANGLE] == 1)
  {
    camera.fov = 0.785398;
    camera.camera->setFieldOfView(camera.fov);
    camera.camera->notifyStateChange();
  }
}

bool RTRQuadcopterController::QRPositionCallback(rtr_msgs::QRPosition::Request &req, rtr_msgs::QRPosition::Response &res)
    {
        std::cout << "QRPositionCallback" << std::endl;
        qr_data.has_request_processed = false;
        qr_data.request = req;

         ros::Duration interval(0, 100000000); // 0.1 seconds
        while(qr_data.has_request_processed == false){
            interval.sleep();
        }
        res = qr_data.response;

        std::cout << "QRPositionCallback" << std::endl;
        return true;
    }

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RTRQuadcopterController)
