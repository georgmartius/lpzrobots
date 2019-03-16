/*
 * OneAxisServoVelDisconnected.h
 *
 *  Created on: 26.03.2014
 *      Author: Johannes Widenka
 */

#ifndef ONEAXISSERVOVELDISCONNECTED_H_
#define ONEAXISSERVOVELDISCONNECTED_H_

#include <ode_robots/oneaxisservo.h>

namespace lpzrobots{

/*
 * This class represents a modified servo. It uses its input as power for its simulated DC motor,
 * simulates friction of the gear
 * and serves as sensor for the joints angle at the same time.
 */

class OneAxisServoVelDisconnected : public lpzrobots::OneAxisServoVel {
  public:
    OneAxisServoVelDisconnected(const OdeHandle& odeHandle,
        OneAxisJoint* joint, double _min, double _max,
        double power, double damp=0.05, double maxVel=20,
        double jointLimit = 1.3);

    //setting motorcommand in means of rotational speed & torque
    void set(double mcmd);
};

}
#endif /* ONEAXISSERVOVELDISCONNECTED_H_ */
