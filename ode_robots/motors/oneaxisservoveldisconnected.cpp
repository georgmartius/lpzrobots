/*
 * OneAxisServoVelDisconnected.cpp
 *
 *  Created on: 26.03.2014
 *      Author: Johannes Widenka
 */


#include "oneaxisservoveldisconnected.h"

namespace lpzrobots{
  
OneAxisServoVelDisconnected::OneAxisServoVelDisconnected(const OdeHandle& odeHandle, OneAxisJoint* joint, double _min,
    double _max, double power, double damp, double maxVel, double jointLimit):OneAxisServoVel(odeHandle,joint,_min,
        _max,power,damp,maxVel,jointLimit) {
  this->maxVel = maxVel*1.2; //factor arbitrary.. handtuned, not so nice
}


void OneAxisServoVelDisconnected::set(double mcmd){
  mcmd = clip(mcmd,-1.0,+1.0);
  double pos = (mcmd>0?1:-1);	//set direction of movement
  double minspeed =0.00;
  double motorfriction = 0.008; //percentage of maximum torque as friction TODO: give as parameter
  pos = (pos+1)*(max-min)/2 + min;
  pid.setTargetPosition(pos);
  double vel = pid.stepVelocity(joint->getPosition1(), joint->odeHandle.getTime());
  double e   = fabs(2.0*(pid.error)/(max-min)); // distance from set point
  motor.set(0, tanh(mcmd*20)*maxVel);
  // calculate power of servo depending on the damping and distance from set point and
  // sigmoid ramping of power for damping < 1
  //      motor.setPower(((1.0-damp)*tanh(e)+damp) * power);
  double setpower = clip(fabs(mcmd*tanh(e*10)),motorfriction,1.0);
  motor.setPower(setpower*power);


  }
}

