/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.20  2010-07-05 16:47:34  martius
 *   hashset transition to tr1
 *   new pid function for velocity servos, which work now fine
 *
 *   Revision 1.19  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.18  2009/08/19 16:17:59  martius
 *   derivative is initialized
 *
 *   Revision 1.17  2009/08/12 10:27:32  der
 *   stepNoCutOff uses smoothed derivative and decay in integration term
 *
 *   Revision 1.16  2009/05/11 15:43:22  martius
 *   new velocity controlling servo motors
 *
 *   Revision 1.15  2007/07/03 13:01:21  martius
 *   new pid formulas,
 *   we use clipped sum for integral term
 *   and clipped derivative value
 *
 *   Revision 1.14  2007/04/03 16:28:38  der
 *   derivative computed on error! This is the correct implementation
 *
 *   Revision 1.13  2007/02/12 13:28:20  martius
 *   twoaxisservo and some minor changes
 *
 *   Revision 1.12  2007/02/01 09:27:36  martius
 *   *** empty log message ***
 *
 *   Revision 1.11  2007/01/31 16:24:15  martius
 *   stabalised servos extremly through limiting damping
 *
 *   Revision 1.10  2007/01/26 12:04:15  martius
 *   servos combinied into OneAxisServo
 *
 *   Revision 1.9  2006/07/14 12:23:32  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.8.4.3  2006/02/07 15:51:56  martius
 *   axis, setpower
 *
 *   Revision 1.8.4.2  2006/01/10 14:48:28  martius
 *   indentation
 *
 *   Revision 1.8.4.1  2005/12/20 17:53:42  martius
 *   changed to Joints from joint.h
 *   new servos for universal and hinge2
 *
 *   Revision 1.8  2005/11/09 14:08:48  martius
 *   *** empty log message ***
 *
 *   Revision 1.7  2005/11/09 13:28:24  fhesse
 *   GPL added
 *                                                                * 
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
