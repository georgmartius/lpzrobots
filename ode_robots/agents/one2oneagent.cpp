#include "one2oneagent.h"

void One2oneAgent::init(AbstractController* controller, AbstractRobot* robot){
  sensornumber = robot->getSensorNumber();
  motornumber  = robot->getMotorNumber();
  sensors = malloc(sizeof(sensor) *sensornumber);
  motors   = malloc(sizeof(motor) *motornumber);
}


 
void One2oneAgent::step(){
  if(!controller || !robot || !sensors || !motors) return;
  int len =  robot->getSensors(sensors, sensornumber);
  if(len != sensornumber){
    fprintf(stderr, "%s:%i: Given not enough sensors!\n", __FILE__, __LINE__);
  }
  controller->step(sensors, sensornumber, motors, motornumber);
  robot->setMotors(motors, motornumber);
  //  if()
  plot(sensors, sensornumber, motors, motornumber);
  
}
 
