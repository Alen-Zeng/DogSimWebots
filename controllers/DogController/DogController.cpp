// File:          DogController.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <bind.hpp>
#include <drivers/DogConfig.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;



Motor *Legs[4][4];
PositionSensor *LegsPos[4][4];
double postest = 0;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  timeStep = (int)robot->getBasicTimeStep();
  DogLegInit(robot, Legs, LegsPos);


  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    Legs[LegDown][LB]->setPosition(postest);
    Legs[LegDown][RB]->setPosition(postest);
    Legs[LegDown][LF]->setPosition(postest);
    Legs[LegDown][RF]->setPosition(postest);

    postest -= 0.001;


  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}