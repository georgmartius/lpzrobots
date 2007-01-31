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
 *   Revision 1.11  2007-01-31 16:24:15  martius
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
#include <ode/ode.h>
#include <iostream>

#include "pid.h"

namespace lpzrobots {

  PID::PID ( double KP , double KI , double KD)
  {
    this->KP = KP;
    this->KI = KI;
    this->KD = KD;

    P=D=I=0;

    targetposition = 0;
	
    position = 0;
    lastposition = 0;
    error = 0;
    alpha = 0.95;
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

  double PID::step ( double newsensorval )
  {
    last2position = lastposition;
    lastposition = position;
    position = newsensorval;
	
    return stepWithD(newsensorval, lastposition - position);
  }

  double PID::stepWithD ( double newsensorval, double derivative ){
    position = newsensorval;

    lasterror = error;
    error = targetposition - position;
	
    P = error;
    I += (1-alpha) * (error * KI - I);
    D = -derivative * KD; 
    // limit damping term to the size of P+I (this stabilised it tremendously!
    double PI = fabs(P+I);
    D = std::min(PI, std::max(-PI,D));
    //D = -( 3*position - 4 * lastposition + last2position ) * KD;    
    force = KP*(P + I + D);
    return force;
  }

}
