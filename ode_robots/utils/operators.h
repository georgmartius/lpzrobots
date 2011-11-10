/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
+ *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
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
#ifndef __OPERATORS_H
#define __OPERATORS_H

#include "operator.h"
#include "axis.h"

namespace lpzrobots {
  /**
     An Operator for limiting the orientation of the main primitive of a robot.
   */
  class LimitOrientationOperator : public Operator {
  public:
    LimitOrientationOperator(const Axis& robotAxis, const Axis& globalAxis, 
                             double maxAngle, double force)
      : robotAxis(robotAxis), globalAxis(globalAxis), 
        maxAngle(maxAngle), force(force), currentforce(force) {
    }

    virtual ManipAction observe(OdeAgent* agent, GlobalData& global);
  protected:
    Axis robotAxis;
    Axis globalAxis;
    double maxAngle;

    double force;
    double currentforce;
  };


  /**
     An Operator for lifting up a robot from time to time.
   */
  class LiftUpOperator : public Operator {
  public:
    LiftUpOperator(const Axis& globalAxis, 
                   double height, double force, double duration, double interval)
      : globalAxis(globalAxis), height(height), force(force), duration(duration), interval(interval), currentforce(force)
    {
    }

    virtual ManipAction observe(OdeAgent* agent, GlobalData& global);
  protected:
    Axis globalAxis;
    double height;

    double force;
    double duration;
    double interval;
    double currentforce;
  };

}

#endif
