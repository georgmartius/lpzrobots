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
#ifndef __AXIS_H
#define __AXIS_H

#include <osg/Vec3>
#include <osg/Vec4>
#include <ode-dbl/ode.h>
#include <iostream>

namespace lpzrobots{

  // class for axis. This is a internally a 4 dimensional vector (homogenenous) with last component = 0 
  //   meaning it is a direction not a point
  class Axis : public osg::Vec4 {
  public:
    Axis () : osg::Vec4() {}
    Axis (float x, float y, float z) : osg::Vec4(x, y, z, 0) {}
    Axis (const osg::Vec4& v) : osg::Vec4(v) { w() =0; }
    Axis (const osg::Vec3& v) : osg::Vec4(v,0) {}
    Axis (const dReal v[3]) : osg::Vec4(v[0], v[1], v[2], 0) {}

    osg::Vec3 vec3() const { return osg::Vec3( x(), y(), z()); }

    float enclosingAngle(const Axis& a) const {
      return acos((*this * a)/(this->length() * a.length()));
    }

    Axis crossProduct(const Axis& a) const {
      return Axis(y()*a.z() - z()*a.y(), z()*a.x() - x()*a.z(), x()*a.y() - y()*a.x());
    }
    
    void print(){
      std::cout << '(' << x() << ',' << y() << ',' << z() << ')' << std::endl;
    }
  };
  
}

#endif
