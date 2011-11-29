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

#include "gripper.h"

#include "primitive.h"
#include "joint.h"

#include <selforg/stl_adds.h>

namespace lpzrobots {

  Gripper::Gripper(Primitive* own)
    : own(own){
    setCollisionCallback(onCollision, this);
    joint=0;
  }
    

  void Gripper::addGrippables(const std::vector<Primitive*>& ps){
    FOREACHC(std::vector<Primitive*>, ps, p){
      grippables[(*p)->getGeom()] = *p;
    }
  }
  
  void Gripper::removeGrippables(const std::vector<Primitive*>& ps){
    FOREACHC(std::vector<Primitive*>, ps, p){
      grippables.erase((*p)->getGeom());
    }
  }

  void Gripper::removeAllGrippables(){
    grippables.clear();
	
  }
  
  int Gripper::onCollision(dSurfaceParameters& params, GlobalData& globaldata, 
                           void *userdata, 
                           dContact* contacts, int numContacts,
                           dGeomID o1, dGeomID o2, 
                           const Substance& s1, const Substance& s2){
    //  Gripper* g = dynamic_cast<Gripper*>(userdata);
    Gripper* g = (Gripper*)(userdata);
    if(!g) return 1;
    if(g->grippables.find(o2) != g->grippables.end()){ // collision with grippable object
      Primitive* p2 = g->grippables[o2];
      
      // create fixed joint
      g->joint = new FixedJoint(g->own, p2);
      return 0;
    }
    return 1;    
  }


}
