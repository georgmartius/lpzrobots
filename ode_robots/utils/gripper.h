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
#include "primitive.h"
#include "color.h"
#include <selforg/configurable.h>
#include <selforg/stl_map.h>
#include <vector>

namespace lpzrobots {

  /**
     Configure object for Gripper
   */
  struct GripperConf {
    std::string name; ///< name of gripper for configuration

    double gripDuration; ///< time in seconds for how long the gripper grasps
    /** releaseDuration time in seconds for how long the gripper
        cannot grasp after release */
    double releaseDuration;
    Color  color;
    double size; ///< diameter of the drawn sphere (if 0 nothing is drawn)
    /** sphere is drawn at contact point (true)
        or at center of attached primitive (false)
    */
    bool   drawAtContactPoint;
    /** if true the last grasped object cannot be directly grasped again
     */
    bool   forbitLastPrimitive;
    //     bool incOrExc; ///< include (false) or exclude (true) grippables;
    bool   fixedOrBallJoint; ///< use fixed joint (true) or ball joint (false)
  };

  /**
     A gripper can be attached to a primitive via its substance
     and implements gripping (a fixed joint) on collision with
     specified objects.
     Usage: in your robot, create a Gripper object and attach
      it to the primitive that grips (e.g. hand). Then you
      need make the gripper(s) available to you simulation
      in order to set call the addGrippables from there
      (e.g. with otherrobot->getAllPrimitives()), see Skeleton.
   */
  class Gripper : public Configurable {
  public:
    /**
       @param gripDuration time in seconds for how long the gripper grasps
       @param releaseDuration time in seconds for how long the gripper cannot grasp
        after release
       @param size diameter of the drawn sphere (if 0 nothing is drawn)
       @param drawAtContactPoint sphere is drawn at contact point (true)
              or at center of attached primitive (false)
    */
    Gripper(const GripperConf& conf = getDefaultConf());

    static GripperConf getDefaultConf(){
      GripperConf conf;
      conf.name                = "Gripper";
      conf.gripDuration        = 10;
      conf.releaseDuration     = 1;
      conf.color               = Color(1,1,1);
      conf.size                = 0.2;
      conf.drawAtContactPoint  = true;
      conf.forbitLastPrimitive = true;
      conf.fixedOrBallJoint    = true;
      return conf;
    }

    /// call this to attach the gripper to the given primitive
    bool attach(Primitive* p);

    virtual void addGrippables(const std::vector<Primitive*>& ps);
    virtual void removeGrippables(const std::vector<Primitive*>& ps);
    virtual void removeAllGrippables();

    static int onCollision(dSurfaceParameters& params, GlobalData& globaldata,
                           void *userdata,
                           dContact* contacts, int numContacts,
                           dGeomID o1, dGeomID o2,
                           const Substance& s1, const Substance& s2);

  private:
    GripperConf conf;
    bool   isAttached;

    dGeomID last;
    HashSet<dGeomID> grippables;
    double gripStartTime;
  };

  typedef std::vector<Gripper*> GripperList;

}

#endif
