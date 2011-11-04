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
#ifndef __POS_H
#define __POS_H

#include <iostream>

#include <osg/Vec3>
#include <osg/Vec4>
#include <ode-dbl/ode.h>
#include <selforg/position.h>

namespace lpzrobots{

  class Pos : public osg::Vec3 {
  public:
    Pos () : osg::Vec3 () {};
    Pos (float x, float y, float z) : osg::Vec3(x, y, z) {}
    Pos (const osg::Vec3& v) : osg::Vec3(v) {}
    Pos (const osg::Vec4& v) : osg::Vec3(v.x(),v.y(),v.z()) {}
    Pos (const Position& p) : osg::Vec3(p.x, p.y, p.z) {}
    Pos (const dReal v[3]) : osg::Vec3(v[0], v[1], v[2]) {}

    /// scaling
    Pos operator*(double f) const { return Pos(x()*f,y()*f,z()*f);}
    /// scalar product
    double operator*(const Pos& p) const { return p.x()*x() + p.y()*y() + p.z()*z();}
    /// componentwise  product
    Pos operator&(const Pos& p) const { return Pos(p.x()*x(), p.y()*y(), p.z()*z());}

    Position toPosition(){
      return Position(x(), y(), z());
    }

    void print() const {
      std::cout << '(' << x() << ',' << y() << ',' << z() << ')' << std::endl;
    }
  };
  
}

#endif
