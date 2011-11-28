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
  
  Operator::ManipType LimitOrientationOperator::observe(OdeAgent* agent, 
                                                        GlobalData& global, 
                                                        ManipDescr& descr){
    OdeRobot* r = agent->getRobot();
    Primitive* p  = r->getMainPrimitive();
    ManipType rv;
    rv=None;
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
      descr.pos  = p->getPosition() + rpose.vec3()*0.5;
      descr.show = true;
      return Move;
    }else{
      currentforce=force;
    }
    return rv;
  }

  Operator::ManipType LiftUpOperator::observe(OdeAgent* agent, GlobalData& global,
                                              ManipDescr& descr){
    OdeRobot* r = agent->getRobot();
    Primitive* p  = r->getMainPrimitive();
    ManipType rv;
    rv = None;
    if(!p) return rv;
    if(conf.intervalMode){      
      /// time in units of interval
      double intTime = global.time/conf.interval;
      // ration of the duration w.r.t interval;
      double durRatio = conf.duration / conf.interval; 
      double timeInInt = intTime - int(intTime); // from 0-1
      if(timeInInt > durRatio){ // do nothing
        currentforce = conf.force;
        return rv;
      }
    }

    const Pos& pos = p->getPosition();
    // printf("test %f \t %f\n", angle, currentforce);
    if(pos.z() < conf.height){     
      double f = 1;
      if(conf.propControl)
        f = conf.height-pos.z();
      p->applyForce(osg::Vec3(0,0,f)*currentforce);
      if(conf.increaseForce){
        currentforce=currentforce*1.01;
      }
      descr.pos  = p->getPosition() + Pos(0,0,conf.visualHeight);
      descr.show = true;
      return Move;
    }else{
      if(conf.increaseForce){
        currentforce=currentforce*0.99;
      }
      if(conf.resetForceIfLifted){
        currentforce=conf.force;
      }
    }
    return rv;
  }


}
