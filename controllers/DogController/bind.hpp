#pragma once
/* Includes ------------------------------------------------------------------*/
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <algorithm/vmc.hpp>
/* Types ---------------------------------------------------------------------*/
enum class JointEnumdef
{
  Shoulder,
  LegUp,
  LegDown,
  Wheel
};
enum class DirEnumdef
{
  LF,
  RF,
  LB,
  RB
};

class DogLegClassdef
{
public:
  webots::Motor *LegJoint[4]; //关节电机
  webots::PositionSensor *LegPosSensor[4];  //关节位置传感器
  /* 关节信息 */
  double JointPosition[4];
  double JointTorque[4];
  double WheelVelocity;
  /* 关节限位MAX-MIN */
  double ShoulderScope[2];
  double LegUpScope[2];
  double LegDownScope[2];
  
  bool PositiveFold = true; // 逆解算LegDown正褶
  double LegHeight;         // 狗腿高度

  DogLegClassdef(/* args */){};
  ~DogLegClassdef(){};

  bool Reset(double shoulder, double legup, double legdown);
  void JointPositionUpdate();
  void SetHeight(double height);
  void SetWheelVelocity(double velocity);
  void SetTorque(double shoulderTor, double legupTor, double legdownTor);
  void SetTorque(double shoulderTor, double legupTor, double legdownTor, double wheelTor);
};

class DogClassdef
{
private:
  /* data */
public:
  DogLegClassdef Legs[4];
  webots::InertialUnit *IMU;
  const double *IMU_Quaternion; // xyzw
  const double *IMU_RPY;        // xyzw

  void DogInit(int timestep, webots::Robot *(&robot), DogLegClassdef legs[4], webots::InertialUnit *imu, Spring_Damper &sdpara);
  void IMUUpdate();

  DogClassdef(/* args */){};
  ~DogClassdef(){};
};


/* Macros --------------------------------------------------------------------*/
int timeStep; /* 仿真周期 */
#ifndef PI
#define PI 3.14159265358979
#endif
Spring_Damper SDParam={0};
DogClassdef Dog;
/* Function declarations -----------------------------------------------------*/

#ifndef Radians
#define Radians(degrees) ((degrees) * PI / 180.0f) // 角度转弧度
#endif

#ifndef Degrees
#define Degrees(radians) ((radians) * 180.0f / PI) // 弧度转角度
#endif

template <typename T>
const T &abs(const T &input)
{
  return input < (T)0 ? -input : input;
}


/**
 * @brief 复位关节位置
 * 
 * @param shoulder 
 * @param legup 
 * @param legdown 
 * @return true 复位完成
 * @return false 
 */
bool DogLegClassdef::Reset(double shoulder, double legup, double legdown)
{
  if (abs(JointPosition[(int)JointEnumdef::Shoulder] - shoulder) > 0.0001 || abs(JointPosition[(int)JointEnumdef::LegUp] - legup) > 0.0001 || abs(JointPosition[(int)JointEnumdef::LegDown] - legdown) > 0.0001)
  {
    for (auto j : LegJoint)
    {
      /* 设置复位速度 */
      j->setVelocity(0.5);
    }

    /* 关闭轮子输出 */
    LegJoint[(int)JointEnumdef::Wheel]->setTorque(0);
    
    LegJoint[(int)JointEnumdef::Shoulder]->setPosition(shoulder);
    LegJoint[(int)JointEnumdef::LegUp]->setPosition(legup);
    LegJoint[(int)JointEnumdef::LegDown]->setPosition(legdown);
    for (int s = 0; s < 4; s++)
    {
      /* 更新位置传感器信息 */
      JointPosition[s] = LegPosSensor[s]->getValue();
    }
    return false;
  }
  else
  {
    for (auto j : LegJoint)
    {
      j->setVelocity(2);
      if(j==LegJoint[(int)JointEnumdef::Wheel])
      {
        /* 重新开启轮子输出 */
        j->setPosition(JointPosition[(int)JointEnumdef::Wheel]);
      }
    }
    return true;
  }
  return false;
}

/**
 * @brief 位置传感器信息更新
 * 
 */
void DogLegClassdef::JointPositionUpdate()
{
  for (int s = 0; s < 4; s++)
  {
    JointPosition[s] = LegPosSensor[s]->getValue();
  }
}

/**
 * @brief 设置关节力矩
 * 
 * @param shoulderTor 
 * @param legupTor 
 * @param legdownTor 
 */
void DogLegClassdef::SetTorque(double shoulderTor, double legupTor, double legdownTor)
{
  LegJoint[(int)JointEnumdef::Shoulder]->setTorque(shoulderTor);
  LegJoint[(int)JointEnumdef::LegUp]->setTorque(legupTor);
  LegJoint[(int)JointEnumdef::LegDown]->setTorque(legdownTor);
}

/**
 * @brief 设置关节力矩
 * 
 * @param shoulderTor 
 * @param legupTor 
 * @param legdownTor 
 * @param wheelTor 
 */
void DogLegClassdef::SetTorque(double shoulderTor, double legupTor, double legdownTor, double wheelTor)
{
  LegJoint[(int)JointEnumdef::Shoulder]->setTorque(shoulderTor);
  LegJoint[(int)JointEnumdef::LegUp]->setTorque(legupTor);
  LegJoint[(int)JointEnumdef::LegDown]->setTorque(legdownTor);
  LegJoint[(int)JointEnumdef::Wheel]->setTorque(wheelTor);
}

/**
 * @brief IMU信息更新
 * 
 */
void DogClassdef::IMUUpdate()
{
  IMU_Quaternion = IMU->getQuaternion();
  IMU_RPY = IMU->getRollPitchYaw();
}