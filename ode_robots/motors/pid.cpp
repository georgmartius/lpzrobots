/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/
#include <ode-dbl/ode.h>
#include <iostream>


#include "pid.h"
using namespace std;

namespace lpzrobots {

  PID::PID ( double KP , double KI , double KD)
    : KP(KP), KI(KI), KD(KD)
  {
    P=D=I=0;

    targetposition = 0;
    derivative     = 0;
    position       = 0;
    lastposition   = 0;
    error          = 0;
    tau            = 1000;
    lasttime       = -1;
    last2position  = 0;
    lasterror      = 0;
    force          = 0;
  }

  void PID::setKP(double KP){
    this->KP = KP;
  }

  void PID::setTargetPosition ( double newpos )
  {
    targetposition = newpos;
  }

  double PID::getTargetPosition ( )
  {
    return targetposition;
  }

  /* This is the old implementation. Do not change in order not to brake all
      the simulations
   */
  double PID::step ( double newsensorval, double time)
  {
    if(lasttime != -1 && time - lasttime > 0 ){
      lastposition = position;
      position = newsensorval;
      double stepsize=time-lasttime;

      lasterror = error;
      error = targetposition - position;
      derivative = (lasterror - error) / stepsize;

      P = error;
      //      I += (1/tau) * (error * KI - I); // I+=error * KI
      I += stepsize * error * KI;
      I = min(0.5,max(-0.5,I)); // limit I to 0.5
      D = -derivative * KD;
      D = min(0.9,max(-0.9,D)); // limit D to 0.9
      force = KP*(P + I + D);
    } else {
      force=0;
    }
    lasttime=time;
    return force;
  }

  // This is the new implementation used by the center and velocity servos
  double PID::stepNoCutoff ( double newsensorval, double time)
  {
    if(lasttime != -1 && time - lasttime > 0 ){
      lastposition = position;
      position = newsensorval;
      double stepsize=time-lasttime;

      lasterror = error;
      error = targetposition - position;
      derivative += ((lasterror - error) / stepsize - derivative)*0.2; // Georg: Who put the 0.2 here!?

      P = error;
      I*= (1-1/tau);
      I += stepsize * error * KI;
      D = -derivative * KD;
      force = KP*(P + I + D);
    } else {
      force=0;
    }
    lasttime=time;
    return force;
  }

  // This is the new implementation used for the velocity servos (velocity control)
  // no I term and velocity is bound such that we cannot overshoot in one step
  double PID::stepVelocity ( double newsensorval, double time)
  {
    // force is here a nominal velocity

    if(lasttime != -1 && time - lasttime > 0 ){
      lastposition = position;
      position = newsensorval;
      double stepsize=time-lasttime;

      lasterror = error;
      error = targetposition - position;

      P = error;
      if(KD!=0.0){
        derivative += ((lasterror - error) / stepsize - derivative);
        D = -derivative * KD;
        force = KP*(P + D);
      } else
        force = KP*P;
      // limit the velocity
      if(stepsize*fabs(force) > fabs(error)){
        force = error/stepsize;
      }
    } else {
      force=0;
    }
    lasttime=time;
    return force;
  }


}
