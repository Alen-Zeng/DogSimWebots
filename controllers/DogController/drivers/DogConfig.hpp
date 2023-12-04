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
 * @brief 狗腿设备初始化
 * 
 * @param leg 
 * @param posSensor 
 */
void DogLegInit(webots::Robot *(&robot),webots::Motor* (&leg)[4][4],webots::PositionSensor* (&posSensor)[4][4])
{
  /* 获取关节电机 */
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

  for (size_t i = 0; i < 4; i++)
  {
    for (size_t j = 0; j < 4; j++)
    {
      leg[i][j]->setVelocity(0.1);
      // leg[i][j]->setPosition(0);
      // leg[i][j]->setTorque(10);
    }
  }
  /* 获取关节位置传感器 */
  posSensor[Shoulder][LF] = robot->getPositionSensor("JShoulderLF_sensor");
  posSensor[Shoulder][RF] = robot->getPositionSensor("JShoulderRF_sensor");
  posSensor[Shoulder][LB] = robot->getPositionSensor("JShoulderLB_sensor");
  posSensor[Shoulder][RB] = robot->getPositionSensor("JShoulderRB_sensor");
  posSensor[LegUp][LF] = robot->getPositionSensor("JLegUpLF_sensor");
  posSensor[LegUp][RF] = robot->getPositionSensor("JLegUpRF_sensor");
  posSensor[LegUp][LB] = robot->getPositionSensor("JLegUpLB_sensor");
  posSensor[LegUp][RB] = robot->getPositionSensor("JLegUpRB_sensor");
  posSensor[LegDown][LF] = robot->getPositionSensor("JLegDownLF_sensor");
  posSensor[LegDown][RF] = robot->getPositionSensor("JLegDownRF_sensor");
  posSensor[LegDown][LB] = robot->getPositionSensor("JLegDownLB_sensor");
  posSensor[LegDown][RB] = robot->getPositionSensor("JLegDownRB_sensor");
  posSensor[Wheel][LF] = robot->getPositionSensor("JWheelLF_sensor");
  posSensor[Wheel][RF] = robot->getPositionSensor("JWheelRF_sensor");
  posSensor[Wheel][LB] = robot->getPositionSensor("JWheelLB_sensor");
  posSensor[Wheel][RB] = robot->getPositionSensor("JWheelRB_sensor");
  /* 启动关节位置传感器 */
  for (size_t i = 0; i < 4; i++)
  {
    for (size_t j = 0; j < 4; j++)
    {
      posSensor[i][j]->enable(timeStep);
    }
  }
  
  std::cout << "Init Finished" << std::endl;

}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
