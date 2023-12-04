#pragma once
/* Includes ------------------------------------------------------------------*/
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
/* Types ---------------------------------------------------------------------*/
enum LegEnumdef
{
  Shoulder,
  LegUp,
  LegDown,
  Wheel
};
enum DirEnumdef
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
  double ShoulderPosition;
  double LegUpPosition;
  double LegDownPosition;
  double WheelPosition;
  double WheelVelocity;
  /* 关节限位 */
  double ShoulderScope[2];
  double LegUpScope[2];
  double LegDownScope[2];
  
  bool PositiveFold = true; // 逆解算LegDown正褶
  double LegHeight;         // 狗腿高度

  DogLeg(/* args */){};
  ~DogLeg(){};

  void Reset();
  void SetHeight(double height);
  void SetWheelVelocity(double velocity);
};


/* Macros --------------------------------------------------------------------*/
int timeStep; /* 仿真周期 */
DogLeg Legs[4];

/* Function declarations -----------------------------------------------------*/

/**
 * @brief reset leg position
 * 
 */
void DogLeg::Reset()
{
  
}
