#pragma once
/* Includes ------------------------------------------------------------------*/
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
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
/* Exported function declarations --------------------------------------------*/

void DogLegInit(webots::Robot *robot,webots::Motor* leg[4][4])
{
  leg[Shoulder][LF] = robot->getMotor("JShoulderLF");
  leg[Shoulder][RF] = robot->getMotor("JShoulderRF");
  leg[Shoulder][LB] = robot->getMotor("JShoulderLB");
  leg[Shoulder][RB] = robot->getMotor("JShoulderRB");
  leg[LegUp][LF] = robot->getMotor("JLegUpLF");
  leg[LegUp][RF] = robot->getMotor("JLegUpRF");
  leg[LegUp][LB] = robot->getMotor("JLegUpLB");
  leg[LegUp][RB] = robot->getMotor("JLegUpRB");
  leg[LegDown][LF] = robot->getMotor("JLegDownLF");
  leg[LegDown][RF] = robot->getMotor("JLegDownRF");
  leg[LegDown][LB] = robot->getMotor("JLegDownLB");
  leg[LegDown][RB] = robot->getMotor("JLegDownRB");
  leg[Wheel][LF] = robot->getMotor("JWheelLF");
  leg[Wheel][RF] = robot->getMotor("JWheelRF");
  leg[Wheel][LB] = robot->getMotor("JWheelLB");
  leg[Wheel][RB] = robot->getMotor("JWheelRB");

}



/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
