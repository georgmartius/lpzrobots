/*
 * spring.h
 *
 * This is a OneAxisServo that leaves the position coordinate unscaled
 * and unshifted. The 0 position is that of initialization of the two joint
 * parts. It now allows preloading of the spring and more control but also
 * more errors. Use carefully!
 *
 *  Created on: Nov 2, 2010
 *      Author: mab
 */

#ifndef SPRING_H_
#define SPRING_H_


#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/pid.h>
#include <ode_robots/angularmotor.h>
#include <selforg/controller_misc.h>

namespace lpzrobots {

/* This is a OneAxisServo that leaves the position coordinate unscaled
* and unshifted. The 0 position is that of initialization of the two joint
* parts. It now allows preloading of the spring and more control but also
* more errors. Use carefully!
*/
class Spring : public OneAxisServo{
public:
        Spring(OneAxisJoint* joint, double _min, double _max,
                         double power, double damp=0.2, double integration=0.0,
                         double maxVel=10.0, double jointLimit = 1.0)
     : OneAxisServo(joint, _min, _max, power, damp, integration, maxVel, jointLimit, false){
        }
        virtual ~Spring(){}

    /** sets the set point of the servo.
        Position is relative to initial position of the two parts
        connected by the spring
    */
    virtual void set(double pos){
        pid.setTargetPosition(pos);

        double force = pid.stepNoCutoff(joint->getPosition1(), joint->odeHandle.getTime());
        force = clip(force,-10*pid.KP, 10*pid.KP); // limit force to 10*KP
        joint->addForce1(force);
        if(maxVel>0){
            joint->getPart1()->limitLinearVel(maxVel);
            joint->getPart2()->limitLinearVel(maxVel);
        }
    }

    /** returns the position of the slider in ranges [-1, 1] (scaled by min, max)*/
    virtual double get(){
        double pos =  joint->getPosition1();
        return pos;
    }

    //want to allow all kinds of borders,
    virtual void setMinMax(double _min, double _max){
          min=_min;
          max=_max;
          joint->setParam(dParamLoStop, min - fabs(min) * (jointLimit-1));
          joint->setParam(dParamHiStop, max + fabs(max) * (jointLimit-1));
        }

};

}

#endif /* SPRING_H_ */
