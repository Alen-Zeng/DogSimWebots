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

int main(int argc, char **argv)
{
  // create the Robot instance.
  robot = new webots::Robot();

  // get the time step of the current world.
  timeStep = (int)robot->getBasicTimeStep();
  Dog.DogInit(timeStep, robot, Dog.Legs,Dog.IMU);
  
  alphaPID.SetPIDParam(5, 0, 0, 0, 1000);
  alphaPID.Target = 0;
  while (robot->step(timeStep) != -1) {
    Dog.IMUUpdate();
    alphaPID.Current = Dog.IMU_RPY[1];
    alphaPID.Adjust();
    std::cout << alphaPID.Out << std::endl;
  };

  delete robot;
  return 0;
}