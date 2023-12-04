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
  Dog.DogInit(timeStep, robot, Dog.Legs,Dog.IMU);

  while (robot->step(timeStep) != -1) {


  };

  delete robot;
  return 0;
}