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
#ifndef __PASSIVEBOX_H
#define __PASSIVEBOX_H

#include <stdio.h>
#include <cmath>

#include "primitive.h"
#include "abstractobstacle.h"

namespace lpzrobots {

/**
 *  (Passive) box as obstacle
 */
class PassiveBox : public AbstractObstacle{
  osg::Vec3 dimension;
  double mass;
  int texture;


  Box* box;


 public:

  /**
   * Constructor, if you set mass=0.0, you get a box which cannot be moved
   */
  PassiveBox(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
             const osg::Vec3& dimension = osg::Vec3(1.0, 1.0, 1.0), double mass = 1.0):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), dimension(dimension), mass(mass), texture(0)
  {
    box = new Box(dimension.x(), dimension.y(), dimension.z());
    obst.push_back(box);
    obstacle_exists=false;
  };


  virtual void setPose(const osg::Matrix& pose){
    this->pose = osg::Matrix::translate(0,0,dimension.z()/2) * pose;
    if (!obstacle_exists) {
      create();
    }
    box->setPose(pose);
  };


  virtual Primitive* getMainPrimitive() const { return box; }

 protected:
  virtual void create(){
    box->setTextures(getTextures(0));
    if (mass==0.0) {
      box->init(odeHandle, mass, osgHandle, Primitive::Geom | Primitive::Draw);
    } else {
      box->init(odeHandle, mass, osgHandle);
    }
//     osg::Vec3 pos=pose.getTrans();
//     box->setPosition(pos);
    obstacle_exists=true;
  };

};

}

#endif

