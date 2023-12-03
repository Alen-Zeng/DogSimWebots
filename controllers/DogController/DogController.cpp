// File:          DogController.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <drivers/DogConfig.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;



Motor *Legs[4][4];
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
  int timeStep = (int)robot->getBasicTimeStep();

  Legs[Shoulder][LF] = robot->getMotor("JShoulderLF");
  Legs[Shoulder][RF] = robot->getMotor("JShoulderRF");
  Legs[Shoulder][LB] = robot->getMotor("JShoulderLB");
  Legs[Shoulder][RB] = robot->getMotor("JShoulderRB");
  Legs[LegUp][LF] = robot->getMotor("JLegUpLF");
  Legs[LegUp][RF] = robot->getMotor("JLegUpRF");
  Legs[LegUp][LB] = robot->getMotor("JLegUpLB");
  Legs[LegUp][RB] = robot->getMotor("JLegUpRB");
  Legs[LegDown][LF] = robot->getMotor("JLegDownLF");
  Legs[LegDown][RF] = robot->getMotor("JLegDownRF");
  Legs[LegDown][LB] = robot->getMotor("JLegDownLB");
  Legs[LegDown][RB] = robot->getMotor("JLegDownRB");
  Legs[Wheel][LF] = robot->getMotor("JWheelLF");
  Legs[Wheel][RF] = robot->getMotor("JWheelRF");
  Legs[Wheel][LB] = robot->getMotor("JWheelLB");
  Legs[Wheel][RB] = robot->getMotor("JWheelRB");

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    
    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}