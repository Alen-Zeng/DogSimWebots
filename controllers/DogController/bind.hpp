#pragma once
/* Includes ------------------------------------------------------------------*/
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
/* -------- macros -----------------------------------------------------------*/
int timeStep; /* 仿真周期 */
/* -------- types ------------------------------------------------------------*/
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
  double ShoulderPosition;
  double LegUpPosition;
  double LegDownPosition;
  double WheelPosition;

  DogLeg(/* args */){};
  ~DogLeg(){};
};


/* -------- function declarations --------------------------------------------*/

