#ifndef __AGENT_H
#define __AGENT_H

#include "abstractrobot.h"
#include "abstractcontroller.h"

/// Abstract glue-object between controller and robot. 
//   Implements wireing of sensors from robot to inputs of the controller and
//   controller outputs to motors. 
class Agent {
public:
  /// default constructor
  //   should be called from overloaded classes!
  Agent(){
    controller = 0;
    robot      = 0;
  }
  /// constructs the object with the given controller and robot
  //   should be called from overloaded classes!
  Agent(AbstractController* controller, AbstractRobot* robot){
    init(controller, robot);
  }

  /// initializes the object with the given controller and robot
  // should be called from overloaded classes!
  virtual bool init(AbstractController* controller, AbstractRobot* robot){
    this->controller = controller;
    this->robot      = robot;
    if(!controller || !robot) return false;
  }

  /// Performs an step of the agent. 
  //   Must be overloaded in order to implement the appropriate mapping 
  //   of the robot sensors to the controller inputs and so on.
  virtual void step() = 0;

  ///
  AbstractController* getController() { return controller;}
  ///
  AbstractRobot* getRobot() { return robot;}

private:
  AbstractController* controller;
  AbstractRobot* robot;
};


#endif


