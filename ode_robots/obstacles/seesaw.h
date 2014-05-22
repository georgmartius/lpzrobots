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
#ifndef __SEESAW_H
#define __SEESAW_H

#include <stdio.h>
#include <cmath>

#include "primitive.h"
#include "joint.h"
#include "abstractobstacle.h"

namespace lpzrobots {

/**
 *  Seesaw
 */
class Seesaw : public AbstractObstacle{
  osg::Vec3 dimension;
  double mass;
  int texture;

 public:
  /**
   * Constructor
   * the support of the seesaw is fixed to the world
   * @param mass mass of the bar
   * @param dimension (length of bar, width of bar, height of support)
   */
  Seesaw(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
         const osg::Vec3& dimension = osg::Vec3(4.0, 0.6, 0.3), double mass = 1.0):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), dimension(dimension),
    mass(mass), texture(0) {
    setTexture("Images/wood_sw.jpg");
    obstacle_exists=false;
  };


  virtual void setPose(const osg::Matrix& pose){
    this->pose = osg::Matrix::translate(0,0,dimension.z()/2.0) * pose;
    if (!obstacle_exists) {
      create();
    }
  };

  virtual Primitive* getMainPrimitive() const {
    if(!obst.empty()) return obst[0];
    else return 0;
  }

 protected:
  virtual void create(){
    Box* support;
    support = new Box(dimension.y()*0.9, dimension.y()/2.0, dimension.z());
    support->setTextures(getTextures(0));
    support->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
    support->setPose(pose);
    obst.push_back(support);

    Box* bar;
    bar = new Box(dimension.y(), dimension.x(), dimension.y()/6.0);
    bar->setTextures(getTextures(1));
    bar->init(odeHandle, mass, osgHandle);
    bar->setPose(osg::Matrix::translate(0,0,dimension.z()/2.0) * pose);
    obst.push_back(bar);

    // connect them together
    HingeJoint* joint = new HingeJoint(bar, support, bar->getPosition(),
                                       bar->toGlobal(Axis(1,0,0)));
    joint->init(odeHandle, osgHandle, true, dimension.y()*1.05,false);

    // maybe set stop values
    obstacle_exists=true;
  };

};

}

#endif

