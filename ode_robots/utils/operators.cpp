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

#include "operators.h"
#include "odeagent.h"

namespace lpzrobots {
  
  Operator::ManipAction LimitOrientationOperator::observe(OdeAgent* agent, GlobalData& global){
    OdeRobot* r = agent->getRobot();
    Primitive* p  = r->getMainPrimitive();
    ManipAction rv;
    rv.type=None;
    if(!p) return rv;
    const Axis& rpose = p->toGlobal(robotAxis);
    double angle = rpose.enclosingAngle(globalAxis);
    // printf("test %f \t %f\n", angle, currentforce);
    if(angle>maxAngle){
      // get orthogonal axis
      const Axis& rot = rpose.crossProduct(globalAxis);
      osg::Vec3 torque = rot.vec3();
      torque.normalize();
      p->applyTorque(torque*currentforce*(angle-maxAngle));
      currentforce=currentforce*1.01;
      rv.type = Move;
      rv.pos  = p->getPosition() + rpose.vec3()*0.5;
      return rv;
    }else{
      currentforce=force;
    }
    return rv;
  }

  Operator::ManipAction LiftUpOperator::observe(OdeAgent* agent, GlobalData& global){
    OdeRobot* r = agent->getRobot();
    Primitive* p  = r->getMainPrimitive();
    ManipAction rv;
    rv.type=None;
    if(!p) return rv;
    const Pos& pos = p->getPosition();
    // printf("test %f \t %f\n", angle, currentforce);
    if(pos.z() < height){
      // get orthogonal axis
      osg::Vec3 force(0,0,height-pos.z());
      p->applyForce(force*currentforce);
      currentforce=currentforce*1.01;
      rv.type = Move;
      rv.pos  = p->getPosition() + force*0.5;
      return rv;
    }else{
      currentforce=force;
    }
    return rv;
  }


}
