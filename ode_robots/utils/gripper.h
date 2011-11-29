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
#ifndef __GRIPPER_H
#define __GRIPPER_H

#include "substance.h"
#include <selforg/stl_map.h>
#include <vector>

namespace lpzrobots {
  
  class FixedJoint;
  class Primitive;

  class Gripper : public Substance {
  public:
    Gripper(Primitive* own, double gripTime, double restTime);
    
    virtual void addGrippables(const std::vector<Primitive*>& ps);
    virtual void removeGrippables(const std::vector<Primitive*>& ps);
    virtual void removeAllGrippables();

    static int onCollision(dSurfaceParameters& params, GlobalData& globaldata, 
			   void *userdata, 
			   dContact* contacts, int numContacts,
			   dGeomID o1, dGeomID o2, 
			   const Substance& s1, const Substance& s2);

  private:
    Primitive* own;    
    double gripTime;
    double restTime;
    //     bool incOrExc; /// include (false) or exclude (true) grippables;
    HashMap<dGeomID, Primitive*> grippables;
    double gripStartTime;
    FixedJoint* joint;
  };
  
}

#endif
