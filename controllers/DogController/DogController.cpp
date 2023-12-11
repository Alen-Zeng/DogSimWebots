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
#include <algorithm/vmc.hpp>


int main(int argc, char **argv)
{
  // create the Robot instance.
  webots::Robot *robot = new webots::Robot();

  // get the time step of the current world.
  timeStep = (int)robot->getBasicTimeStep();
  Dog.DogInit(timeStep, robot, Dog.Legs,Dog.IMU,SDParam);

  while (robot->step(timeStep) != -1) {
    Dog.IMUUpdate();
    Dog.PositionUpdate();
    SDParam.alphacurr = -Dog.IMU_RPY[1];
    SDParam.betacurr = Dog.IMU_RPY[0];
    SDParam.zcurr = Dog.DogPosition[2];
    MatrixHInit(H, SDParam.alphacurr, SDParam.betacurr,
                Dog.Legs[(int)DirEnumdef::LF].JointPosition[(int)JointEnumdef::LegDown], Dog.Legs[(int)DirEnumdef::LF].JointPosition[(int)JointEnumdef::LegUp], -Dog.Legs[(int)DirEnumdef::LF].JointPosition[(int)JointEnumdef::Shoulder],
                -Dog.Legs[(int)DirEnumdef::RF].JointPosition[(int)JointEnumdef::LegDown], -Dog.Legs[(int)DirEnumdef::RF].JointPosition[(int)JointEnumdef::LegUp], -Dog.Legs[(int)DirEnumdef::RF].JointPosition[(int)JointEnumdef::Shoulder],
                Dog.Legs[(int)DirEnumdef::LB].JointPosition[(int)JointEnumdef::LegDown], Dog.Legs[(int)DirEnumdef::LB].JointPosition[(int)JointEnumdef::LegUp], Dog.Legs[(int)DirEnumdef::LB].JointPosition[(int)JointEnumdef::Shoulder],
                -Dog.Legs[(int)DirEnumdef::RB].JointPosition[(int)JointEnumdef::LegDown], -Dog.Legs[(int)DirEnumdef::RB].JointPosition[(int)JointEnumdef::LegUp], Dog.Legs[(int)DirEnumdef::RB].JointPosition[(int)JointEnumdef::Shoulder]);
    TorqueCalculate(H, Fs, SDParam, Dog.Legs[(int)DirEnumdef::LF].JointTargetTorque, Dog.Legs[(int)DirEnumdef::RF].JointTargetTorque, Dog.Legs[(int)DirEnumdef::LB].JointTargetTorque, Dog.Legs[(int)DirEnumdef::RB].JointTargetTorque);
    // for(auto l:Dog.Legs)
    // {
    //   l.SetTorque(l.JointTargetTorque[(int)JointEnumdef::LegUp], l.JointTargetTorque[(int)JointEnumdef::LegDown]);
    // }
    // std::cout << Dog.Legs[(int)DirEnumdef::LF].JointTargetTorque[(int)JointEnumdef::LegUp] << " & " << Dog.Legs[(int)DirEnumdef::LF].JointTargetTorque[(int)JointEnumdef::LegDown] << std::endl;
    // Dog.Legs[(int)DirEnumdef::LF].SetTorque(Dog.Legs[(int)DirEnumdef::LF].JointTargetTorque[(int)JointEnumdef::LegUp], /* Dog.Legs[(int)DirEnumdef::LF].JointTargetTorque[(int)JointEnumdef::LegDown] */0);
  };

  delete robot;
  return 0;
}