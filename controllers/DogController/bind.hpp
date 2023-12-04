#pragma once
/* Includes ------------------------------------------------------------------*/
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
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

class DogLeg
{
public:
  webots::Motor *LegJoint[4];
  webots::PositionSensor *LegPosSensor[4];
  /* 关节信息 */
  double JointPosition[4];
  double WheelVelocity;
  /* 关节限位MAX-MIN */
  double ShoulderScope[2];
  double LegUpScope[2];
  double LegDownScope[2];
  
  bool PositiveFold = true; // 逆解算LegDown正褶
  double LegHeight;         // 狗腿高度

  DogLeg(/* args */){};
  ~DogLeg(){};

  bool Reset(double shoulder, double legup, double legdown);
  void SetHeight(double height);
  void SetWheelVelocity(double velocity);
};


/* Macros --------------------------------------------------------------------*/
int timeStep; /* 仿真周期 */
DogLeg Legs[4];
#ifndef PI
#define PI 3.14159265358979
#endif

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
 * @brief reset leg position
 * 
 * @param shoulder 
 * @param legup 
 * @param legdown 
 */
bool DogLeg::Reset(double shoulder, double legup, double legdown)
{
  // std::cout << LegPosSensor[0]->getName() << std::endl;
  // std::cout << abs(JointPosition[(int)JointEnumdef::Shoulder] - shoulder) << std::endl;
  // std::cout << abs(JointPosition[(int)JointEnumdef::LegUp] - legup) << std::endl;
  // std::cout << abs(JointPosition[(int)JointEnumdef::LegDown] - legdown) << std::endl;
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
