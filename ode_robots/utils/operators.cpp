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
    if(angle>maxAngle || (active && angle>minAngle)){
      // get orthogonal axis
      const Axis& rot = rpose.crossProduct(globalAxis);
      osg::Vec3 torque = rot.vec3();
      torque.normalize();
      p->applyTorque(torque*currentforce*(angle-minAngle+0.1));
      currentforce=currentforce*1.01;
      descr.pos  = p->getPosition() + rpose.vec3()*0.5;
      descr.show = 1;
      active=true;
      return Move;
    }else{
      active=false;
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
      descr.posStart = p->getPosition();
      descr.show = 2;
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


  Operator::ManipType PullToPointOperator::observe(OdeAgent* agent, GlobalData& global,
                                                   ManipDescr& descr){
    OdeRobot* r = agent->getRobot();
    Primitive* p  = r->getMainPrimitive();
    ManipType rv;
    rv=None;
    if(!p) return rv;
    const Pos& pos = p->getPosition();
    Pos vec = point - pos;
    if( (dim & X) == 0) vec.x()=0;
    if( (dim & Y) == 0) vec.y()=0;
    if( (dim & Z) == 0) vec.z()=0;
    if(vec.length() > minDist){
      p->applyForce(vec*force);
      if(damp>0){
        Pos vel(p->getVel());
        if( (dim & X) == 0) vel.x()=0;
        if( (dim & Y) == 0) vel.y()=0;
        if( (dim & Z) == 0) vel.z()=0;
        p->applyForce(vel*(-damp)*force);
      }

      descr.pos  = pos + vec;
      descr.posStart = pos;
      descr.show = showPoint ? 2 : 0;
      return Move;
    }
    return rv;
  }

  void PullToPointOperator::notifyOnChange(const paramkey& key){
    if(key=="point_x"){
      point.x()=px;
    }else     if(key=="point_y"){
      point.y()=py;
    }else     if(key=="point_z"){
      point.z()=pz;
    }
  }


  Operator::ManipType BoxRingOperator::observe(OdeAgent* agent, GlobalData& global,
                                               ManipDescr& descr){
    OdeRobot* r = agent->getRobot();
    Primitive* p  = r->getMainPrimitive();
    ManipType rv;
    rv=None;
    if(!p) return rv;
    const Pos& pos = p->getPosition();
    Pos vec = center - pos;
    if(!sphere) {
      double max = 0;
      int idx=0;
      for(int i=0; i < 3; i++ ){
        if(fabs(vec.ptr()[i]) > fabs(max)){
          max = vec.ptr()[i];
          idx = i;
        }
        vec.ptr()[i] = 0;
      }
      vec.ptr()[idx] = max; // all but the one component is 0;
    }
    if(vec.length() > size - offset){
      p->applyForce(vec*force);
      Pos p(vec);
      p.normalize();
      descr.pos          = pos  - p*offset;
      descr.orientation  = Pose::rotate(osg::Vec3(0,0,1), vec);
      descr.size         = Pos(0.3,0,0.05);
      descr.show = 1;
      return Limit;
    }
    return rv;
  }


}
