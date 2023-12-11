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
 * @param imu 
 * @param sdpara 弹簧阻尼器参数
 */
void DogClassdef::DogInit(int timestep, webots::Robot *(&robot), DogLegClassdef (&legs)[4], webots::InertialUnit *(&imu), Spring_Damper &sdpara)
{
  /* 获取关节电机 */
  legs[(int)DirEnumdef::LF].LegJoint[(int)JointEnumdef::Shoulder]=robot->getMotor("JShoulderLF");
  legs[(int)DirEnumdef::LF].LegJoint[(int)JointEnumdef::LegUp] = robot->getMotor("JLegUpLF");
  legs[(int)DirEnumdef::LF].LegJoint[(int)JointEnumdef::LegDown] = robot->getMotor("JLegDownLF");
  legs[(int)DirEnumdef::LF].LegJoint[(int)JointEnumdef::Wheel] = robot->getMotor("JWheelLF");

  legs[(int)DirEnumdef::RF].LegJoint[(int)JointEnumdef::Shoulder] = robot->getMotor("JShoulderRF");
  legs[(int)DirEnumdef::RF].LegJoint[(int)JointEnumdef::LegUp] = robot->getMotor("JLegUpRF");
  legs[(int)DirEnumdef::RF].LegJoint[(int)JointEnumdef::LegDown] = robot->getMotor("JLegDownRF");
  legs[(int)DirEnumdef::RF].LegJoint[(int)JointEnumdef::Wheel] = robot->getMotor("JWheelRF");

  legs[(int)DirEnumdef::LB].LegJoint[(int)JointEnumdef::Shoulder] = robot->getMotor("JShoulderLB");
  legs[(int)DirEnumdef::LB].LegJoint[(int)JointEnumdef::LegUp] = robot->getMotor("JLegUpLB");
  legs[(int)DirEnumdef::LB].LegJoint[(int)JointEnumdef::LegDown] = robot->getMotor("JLegDownLB");
  legs[(int)DirEnumdef::LB].LegJoint[(int)JointEnumdef::Wheel] = robot->getMotor("JWheelLB");

  legs[(int)DirEnumdef::RB].LegJoint[(int)JointEnumdef::Shoulder] = robot->getMotor("JShoulderRB");
  legs[(int)DirEnumdef::RB].LegJoint[(int)JointEnumdef::LegUp] = robot->getMotor("JLegUpRB");
  legs[(int)DirEnumdef::RB].LegJoint[(int)JointEnumdef::LegDown] = robot->getMotor("JLegDownRB");
  legs[(int)DirEnumdef::RB].LegJoint[(int)JointEnumdef::Wheel] = robot->getMotor("JWheelRB");

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
  legs[(int)DirEnumdef::LF].LegPosSensor[(int)JointEnumdef::Shoulder]=robot->getPositionSensor("JShoulderLF_sensor");
  legs[(int)DirEnumdef::LF].LegPosSensor[(int)JointEnumdef::LegUp] = robot->getPositionSensor("JLegUpLF_sensor");
  legs[(int)DirEnumdef::LF].LegPosSensor[(int)JointEnumdef::LegDown] = robot->getPositionSensor("JLegDownLF_sensor");
  legs[(int)DirEnumdef::LF].LegPosSensor[(int)JointEnumdef::Wheel] = robot->getPositionSensor("JWheelLF_sensor");

  legs[(int)DirEnumdef::RF].LegPosSensor[(int)JointEnumdef::Shoulder] = robot->getPositionSensor("JShoulderRF_sensor");
  legs[(int)DirEnumdef::RF].LegPosSensor[(int)JointEnumdef::LegUp] = robot->getPositionSensor("JLegUpRF_sensor");
  legs[(int)DirEnumdef::RF].LegPosSensor[(int)JointEnumdef::LegDown] = robot->getPositionSensor("JLegDownRF_sensor");
  legs[(int)DirEnumdef::RF].LegPosSensor[(int)JointEnumdef::Wheel] = robot->getPositionSensor("JWheelRF_sensor");

  legs[(int)DirEnumdef::LB].LegPosSensor[(int)JointEnumdef::Shoulder] = robot->getPositionSensor("JShoulderLB_sensor");
  legs[(int)DirEnumdef::LB].LegPosSensor[(int)JointEnumdef::LegUp] = robot->getPositionSensor("JLegUpLB_sensor");
  legs[(int)DirEnumdef::LB].LegPosSensor[(int)JointEnumdef::LegDown] = robot->getPositionSensor("JLegDownLB_sensor");
  legs[(int)DirEnumdef::LB].LegPosSensor[(int)JointEnumdef::Wheel] = robot->getPositionSensor("JWheelLB_sensor");

  legs[(int)DirEnumdef::RB].LegPosSensor[(int)JointEnumdef::Shoulder] = robot->getPositionSensor("JShoulderRB_sensor");
  legs[(int)DirEnumdef::RB].LegPosSensor[(int)JointEnumdef::LegUp] = robot->getPositionSensor("JLegUpRB_sensor");
  legs[(int)DirEnumdef::RB].LegPosSensor[(int)JointEnumdef::LegDown] = robot->getPositionSensor("JLegDownRB_sensor");
  legs[(int)DirEnumdef::RB].LegPosSensor[(int)JointEnumdef::Wheel] = robot->getPositionSensor("JWheelRB_sensor");
  /* 启动关节位置传感器 */
  for (size_t i = 0; i < 4; i++)
  {
    for (size_t j = 0; j < 4; j++)
    {
      legs[i].LegPosSensor[j]->enable(timestep);
    }
    if (robot->step(timestep) != -1)
    {
      for (int s = 0; s < 4; s++)
      {
        legs->JointPosition[s] = legs[i].LegPosSensor[s]->getValue();
        // std::cout << legs[i].LegPosSensor[s]->getName() << ":" << legs[i].LegPosSensor[s]->getValue() << std::endl;
      }
    }
  }
  std::cout << "Position Sensors Initialized" << std::endl;

  /* 获取IMU */
  imu = robot->getInertialUnit("IMU");
  imu->enable(timestep);
  if (robot->step(timestep) != -1)
  {
    for (int i = 0; i < 4; i++)
    {
      IMU_Quaternion[i] = imu->getQuaternion()[i];
    }
    for (int i = 0; i < 3; i++)
    {
      IMU_RPY[i] = imu->getRollPitchYaw()[i];
    }
  }
  std::cout << "IMU Initialized" << std::endl;

  /* 记录关节限位 */
  legs[(int)DirEnumdef::LF].ShoulderScope[0] = 1.57080;
  legs[(int)DirEnumdef::LF].ShoulderScope[1] = -0.34906;
  legs[(int)DirEnumdef::LF].LegUpScope[0] = 3.14159;
  legs[(int)DirEnumdef::LF].LegUpScope[1] = -3.14158;
  legs[(int)DirEnumdef::LF].LegDownScope[0] = 2.61799;
  legs[(int)DirEnumdef::LF].LegDownScope[1] = -2.61799;

  legs[(int)DirEnumdef::RF].ShoulderScope[0] = 0.34906;
  legs[(int)DirEnumdef::RF].ShoulderScope[1] = -1.57080;
  legs[(int)DirEnumdef::RF].LegUpScope[0] = 3.14159;
  legs[(int)DirEnumdef::RF].LegUpScope[1] = -3.14158;
  legs[(int)DirEnumdef::RF].LegDownScope[0] = 2.61799;
  legs[(int)DirEnumdef::RF].LegDownScope[1] = -2.61799;

  legs[(int)DirEnumdef::LB].ShoulderScope[0] = 0.34906;
  legs[(int)DirEnumdef::LB].ShoulderScope[1] = -1.57080;
  legs[(int)DirEnumdef::LB].LegUpScope[0] = 3.14159;
  legs[(int)DirEnumdef::LB].LegUpScope[1] = -3.14158;
  legs[(int)DirEnumdef::LB].LegDownScope[0] = 2.61799;
  legs[(int)DirEnumdef::LB].LegDownScope[1] = -2.61799;
  
  legs[(int)DirEnumdef::RB].ShoulderScope[0] = 1.57080;
  legs[(int)DirEnumdef::RB].ShoulderScope[1] = -0.34906;
  legs[(int)DirEnumdef::RB].LegUpScope[0] = 3.14159;
  legs[(int)DirEnumdef::RB].LegUpScope[1] = -3.14158;
  legs[(int)DirEnumdef::RB].LegDownScope[0] = 2.61799;
  legs[(int)DirEnumdef::RB].LegDownScope[1] = -2.61799;

  /* 关节复位 */
  std::cout << "Legs Reseting" << std::endl;
  while ((!legs[(int)DirEnumdef::LF].Reset(Radians(0), Radians(60), Radians(-140)) 
       || !legs[(int)DirEnumdef::RF].Reset(Radians(0), Radians(-60), Radians(140)) 
       || !legs[(int)DirEnumdef::LB].Reset(Radians(0), Radians(60), Radians(-140)) 
       || !legs[(int)DirEnumdef::RB].Reset(Radians(0), Radians(-60), Radians(140)))
      && robot->step(timestep) != -1)
      {
        legs[(int)DirEnumdef::LF].Reset(Radians(0), Radians(60), Radians(-140));
        legs[(int)DirEnumdef::RF].Reset(Radians(0), Radians(-60), Radians(140));
        legs[(int)DirEnumdef::LB].Reset(Radians(0), Radians(60), Radians(-140));
        legs[(int)DirEnumdef::RB].Reset(Radians(0), Radians(-60), Radians(140));
      }
  std::cout << "Legs Reset" << std::endl;

  /* 初始化弹簧阻尼器参数,其他参数先暂时通过结构体初始化为0 */
  sdpara.zd = 0.55;
  sdpara.Kx = 0;
  sdpara.Ky = 0;
  sdpara.Kz = 10;
  sdpara.Kalpha = 0;
  sdpara.Kbeta = 0.0;
  sdpara.Bx = 0;
  sdpara.By = 0;
  sdpara.Bz = 0;
  sdpara.Balpha = 0;
  sdpara.Bbeta = 0;

  std::cout << "All Initialized" << std::endl;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
