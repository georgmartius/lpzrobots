#ifndef __CONTROLLEDROBOT_H
#define __CONTROLLEDROBOT_H

#include "abstractrobot.h"
#include "abstractcontroller.h"

/// Glue-object between controller and robot. 
//   Implements wireing of sensors from robot to inputs of the controller and
//   controller outputs to motors. 
class ControlledRobot {
public:
  ControlledRobot(){
    controller = 0;
    robot      = 0;
  }
  /// constructs the object with the given controller and robot
  ControlledRobot(AbstractController* controller, AbstractRobot* robot){
    init(controller, robot);
  }

  /// initializes the object with the given controller and robot
  void init(AbstractController* controller, AbstractRobot* robot){
    this->controller = controller;
    this->robot      = robot;
  }

  void Step() = 0;

  ///
  AbstractController* getController() { return controller;}
  ///
  AbstractRobot* getRobot() { return robot;}

private:
  AbstractController* controller;
  AbstractRobot* robot;
};


#endif


