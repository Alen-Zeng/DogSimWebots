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
#include <algorithm/PID.hpp>

myPID alphaPID;
myPID betaPID;

int main(int argc, char **argv)
{
  // create the Robot instance.
  robot = new webots::Robot();

  // get the time step of the current world.
  timeStep = (int)robot->getBasicTimeStep();
  Dog.DogInit(timeStep, robot, Dog.Legs,Dog.IMU);
  Dog.Legs[(int)DirEnumdef::RF].PositiveFold = !Dog.Legs[0].PositiveFold;
  Dog.Legs[(int)DirEnumdef::RB].PositiveFold = !Dog.Legs[0].PositiveFold;
  
  alphaPID.SetPIDParam(1, 0, 1, 0, 100);
  alphaPID.Target = 0;
  betaPID.SetPIDParam(1, 0, 1, 0, 100);
  betaPID.Target = 0;
  while (robot->step(timeStep) != -1) {
    Dog.IMUUpdate();
    alphaPID.Current = Dog.IMU_RPY[1];
    betaPID.Current = Dog.IMU_RPY[0];
    alphaPID.Adjust();
    betaPID.Adjust();
    lfhei += -0.015 * alphaPID.Out + 0.015 * betaPID.Out;
    rfhei += -0.015 * alphaPID.Out - 0.015 * betaPID.Out;
    lbhei += 0.015 * alphaPID.Out + 0.015 * betaPID.Out;
    rbhei += 0.015 * alphaPID.Out - 0.015 * betaPID.Out;

    Dog.Legs[(int)DirEnumdef::LF].SetHeight(lfhei);
    Dog.Legs[(int)DirEnumdef::RF].SetHeight(rfhei);
    Dog.Legs[(int)DirEnumdef::LB].SetHeight(lbhei);
    Dog.Legs[(int)DirEnumdef::RB].SetHeight(rbhei);
    
    Dog.GOHead();
  };

  delete robot;
  return 0;
}