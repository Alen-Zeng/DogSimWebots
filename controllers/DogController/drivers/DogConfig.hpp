#pragma once
/* Includes ------------------------------------------------------------------*/
#include <bind.hpp>
#include <iostream>
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 狗腿初始化
 * 
 * @param timestep 仿真周期
 * @param legs 
 */
void DogLegInit(int timestep,webots::Robot *(&robot),DogLeg legs[4])
{
  /* 获取关节电机 */
  legs[LF].LegJoint[Shoulder]=robot->getMotor("JShoulderLF");
  legs[LF].LegJoint[LegUp] = robot->getMotor("JLegUpLF");
  legs[LF].LegJoint[LegDown] = robot->getMotor("JLegDownLF");
  legs[LF].LegJoint[Wheel] = robot->getMotor("JWheelLF");

  legs[RF].LegJoint[Shoulder] = robot->getMotor("JShoulderRF");
  legs[RF].LegJoint[LegUp] = robot->getMotor("JLegUpRF");
  legs[RF].LegJoint[LegDown] = robot->getMotor("JLegDownRF");
  legs[RF].LegJoint[Wheel] = robot->getMotor("JWheelRF");

  legs[LB].LegJoint[Shoulder] = robot->getMotor("JShoulderLB");
  legs[LB].LegJoint[LegUp] = robot->getMotor("JLegUpLB");
  legs[LB].LegJoint[LegDown] = robot->getMotor("JLegDownLB");
  legs[LB].LegJoint[Wheel] = robot->getMotor("JWheelLB");

  legs[RB].LegJoint[Shoulder] = robot->getMotor("JShoulderRB");
  legs[RB].LegJoint[LegUp] = robot->getMotor("JLegUpRB");
  legs[RB].LegJoint[LegDown] = robot->getMotor("JLegDownRB");
  legs[RB].LegJoint[Wheel] = robot->getMotor("JWheelRB");

  for (size_t i = 0; i < 4; i++)
  {
    for (size_t j = 0; j < 4; j++)
    {
      legs[i].LegJoint[j]->setVelocity(3);
      legs[i].LegJoint[j]->enableTorqueFeedback(timestep);
      legs[i].LegJoint[j]->enableForceFeedback(timestep);
    }
  }
  std::cout << "Motors Initialized" << std::endl;

    /* 获取关节位置传感器 */
  legs[LF].LegPosSensor[Shoulder]=robot->getPositionSensor("JShoulderLF_sensor");
  legs[LF].LegPosSensor[LegUp] = robot->getPositionSensor("JLegUpLF_sensor");
  legs[LF].LegPosSensor[LegDown] = robot->getPositionSensor("JLegDownLF_sensor");
  legs[LF].LegPosSensor[Wheel] = robot->getPositionSensor("JWheelLF_sensor");

  legs[RF].LegPosSensor[Shoulder] = robot->getPositionSensor("JShoulderRF_sensor");
  legs[RF].LegPosSensor[LegUp] = robot->getPositionSensor("JLegUpRF_sensor");
  legs[RF].LegPosSensor[LegDown] = robot->getPositionSensor("JLegDownRF_sensor");
  legs[RF].LegPosSensor[Wheel] = robot->getPositionSensor("JWheelRF_sensor");

  legs[LB].LegPosSensor[Shoulder] = robot->getPositionSensor("JShoulderLB_sensor");
  legs[LB].LegPosSensor[LegUp] = robot->getPositionSensor("JLegUpLB_sensor");
  legs[LB].LegPosSensor[LegDown] = robot->getPositionSensor("JLegDownLB_sensor");
  legs[LB].LegPosSensor[Wheel] = robot->getPositionSensor("JWheelLB_sensor");

  legs[RB].LegPosSensor[Shoulder] = robot->getPositionSensor("JShoulderRB_sensor");
  legs[RB].LegPosSensor[LegUp] = robot->getPositionSensor("JLegUpRB_sensor");
  legs[RB].LegPosSensor[LegDown] = robot->getPositionSensor("JLegDownRB_sensor");
  legs[RB].LegPosSensor[Wheel] = robot->getPositionSensor("JWheelRB_sensor");
  /* 启动关节位置传感器 */
  for (size_t i = 0; i < 4; i++)
  {
    for (size_t j = 0; j < 4; j++)
    {
      legs[i].LegPosSensor[j]->enable(timestep);
    }
  }
  std::cout << "Position Sensors Initialized" << std::endl;

}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
