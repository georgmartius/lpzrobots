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
#ifndef __TMPPRIMITIVE_H
#define __TMPPRIMITIVE_H

#include "tmpobject.h"
#include <ode_robots/osgprimitive.h>

namespace lpzrobots {

  class Primitive;
  class Joint;

  /**
   holding a temporary primitive
   */
  class TmpPrimitive : public TmpObject {
  public:
    /** creates a new item from the given primitives and initializes it.
        The lifetime is set when adding it to globalData
     */
    TmpPrimitive(Primitive* p, char mode, double mass, const Pose& pose,
                 const Color& color);

    /// provided for convenience to supply color as name and alpha independently
    TmpPrimitive(Primitive* p, char mode, double mass, const Pose& pose,
                 const std::string& colorname, float alpha = 1.0);

    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle);
    virtual void deleteObject();
    virtual void update();

  private:
    Primitive* item;
    char mode;
    double mass;
    Pose pose;
    Color color;
    std::string colorname;
    bool useColorName;
    float alpha;
    bool initialized;
  };

  /**
   holding a temporary graphical item
   */
  class TmpDisplayItem : public TmpObject {
  public:
    /** creates a new item from the given primitives and initializes it.
        The lifetime is set when adding it to globalData
     */
    TmpDisplayItem(OSGPrimitive* p, const Pose& pose, const Color& color,
                   OSGPrimitive::Quality quality = OSGPrimitive::Middle);

    /// provided for convenience to supply color as name and alpha independently
    TmpDisplayItem(OSGPrimitive* p, const Pose& pose,
                   const std::string& colorname, float alpha = 1.0,
                   OSGPrimitive::Quality quality = OSGPrimitive::Middle);

    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle);

    virtual void deleteObject();
    virtual void update() {} // nothing to be done here, because they do not move

  private:
    OSGPrimitive* item;
    Pose pose;
    Color color;
    std::string colorname;
    bool useColorName;
    float alpha;
    OSGPrimitive::Quality quality;
    bool initialized;
  };

  /**
   holding a temporary joint
   */
  class TmpJoint : public TmpObject {
  public:
    /** creates a new tmporary object from the given joint and initializes it.
        The lifetime is set when adding it to globalData
     */
    TmpJoint(Joint* p, const Color& color, bool withVisual = true, double visualSize = 0.2,
             bool ignoreColl = true);

    /// provided for convenience to supply color as name and alpha independently
    TmpJoint(Joint* p, const std::string& colorname, float alpha = 1.0,
             bool withVisual = true, double visualSize = 0.2, bool ignoreColl = true);

    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle);

    virtual void deleteObject();
    virtual void update();

  private:
    Joint* joint;
    Color color;
    std::string colorname;
    bool useColorName;
    float alpha;
    bool withVisual;
    double visualSize;
    bool ignoreColl;
    bool initialized;
  };

}

#endif
