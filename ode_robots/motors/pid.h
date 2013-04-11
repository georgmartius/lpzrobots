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
#ifndef __PID_H
#define __PID_H

namespace lpzrobots {

  class PID
  {
    //*********************attributes***************
    //private:
  public:

    double position;
    double lastposition;
    double last2position;

    double error;
    double lasterror;
    double derivative;
    double targetposition;

    double KP;
    double KI;
    double KD;
    double tau;

    double P;
    double D;
    double I;

    double force;
    double lasttime;  // last update time (to calc stepsize)

    //*********************methods******************
  public :
    /// KP is used as a general koefficient. KI and KD can be tuned without dependence of KP
    PID ( double KP = 100 , double KI = 2.0 , double KD = 0.3 );

    void setKP(double KP);

    void setTargetPosition ( double newpos );

    double getTargetPosition ();

    /// perform one step of the PID controller with cutoff for large forces
    double step ( double newsensorval, double time);
    /// perform one step of the PID controller without cutoffs used for Center-Servos
    double stepNoCutoff ( double newsensorval, double time);
    /** perform one step of the PID controller for velocity control.
        Meaning the misfit is in position space but the output is
        the nominal velocity. The velocity is also limited. such that
        the maximal velocity cannot be so that the error is overcompenstated
        in one timestep.
     */
    double stepVelocity ( double newsensorval, double time);

  };

}

#endif
