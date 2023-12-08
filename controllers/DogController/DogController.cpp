/**
  ******************************************************************************
  * @file    DogController.cpp
  * @author  ZengXilang chenmoshaoalen@126.com
  * @brief   
  * @date    2023/12/04 13:39:51
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <bind.hpp>
#include <drivers/DogConfig.hpp>

int main(int argc, char **argv)
{
  // create the Robot instance.
  webots::Robot *robot = new webots::Robot();

  // get the time step of the current world.
  timeStep = (int)robot->getBasicTimeStep();
  Dog.DogInit(timeStep, robot, Dog.Legs,Dog.IMU,SDParam);

  while (robot->step(timeStep) != -1) {
    SDParam.alphacurr = Dog.IMU->getRollPitchYaw()[1];
    SDParam.betacurr = Dog.IMU->getRollPitchYaw()[0];
    MatrixHInit(H, SDParam.alphacurr, SDParam.betacurr,
                Dog.Legs[(int)DirEnumdef::LF].JointPosition[(int)JointEnumdef::LegDown], Dog.Legs[(int)DirEnumdef::LF].JointPosition[(int)JointEnumdef::LegUp], Dog.Legs[(int)DirEnumdef::LF].JointPosition[(int)JointEnumdef::Shoulder],
                Dog.Legs[(int)DirEnumdef::RF].JointPosition[(int)JointEnumdef::LegDown], Dog.Legs[(int)DirEnumdef::RF].JointPosition[(int)JointEnumdef::LegUp], Dog.Legs[(int)DirEnumdef::RF].JointPosition[(int)JointEnumdef::Shoulder],
                Dog.Legs[(int)DirEnumdef::LB].JointPosition[(int)JointEnumdef::LegDown], Dog.Legs[(int)DirEnumdef::LB].JointPosition[(int)JointEnumdef::LegUp], Dog.Legs[(int)DirEnumdef::LB].JointPosition[(int)JointEnumdef::Shoulder],
                Dog.Legs[(int)DirEnumdef::RB].JointPosition[(int)JointEnumdef::LegDown], Dog.Legs[(int)DirEnumdef::RB].JointPosition[(int)JointEnumdef::LegUp], Dog.Legs[(int)DirEnumdef::RB].JointPosition[(int)JointEnumdef::Shoulder]);
    TorqueCalculate(H, Fs, SDParam, Dog.Legs[(int)DirEnumdef::LF].JointTorque, Dog.Legs[(int)DirEnumdef::RF].JointTorque, Dog.Legs[(int)DirEnumdef::LB].JointTorque, Dog.Legs[(int)DirEnumdef::RB].JointTorque);
  };

  delete robot;
  return 0;
}