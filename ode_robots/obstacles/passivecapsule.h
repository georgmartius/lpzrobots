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
#ifndef __PASSIVECAPSULE_H
#define __PASSIVECAPSULE_H

#include <stdio.h>
#include <cmath>

#include "primitive.h"
#include "abstractobstacle.h"

namespace lpzrobots {

/**
 *  (Passive) capsule as obstacle
 */
class PassiveCapsule : public AbstractObstacle{
  float radius;
  float height;
  double mass;

  Capsule* capsule;


 public:

  /**
   * Constructor
   */
  PassiveCapsule(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                 float radius=1.0, float height=1.0, double mass = 1.0):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), radius(radius), height(height), mass(mass) {
    capsule = new Capsule(radius,height);
    obst.push_back(capsule);
    obstacle_exists=false;
  };

  ~PassiveCapsule(){
  }

  /**
   * update position of box
   */
  virtual void update(){
    if(capsule) capsule->update();
  };

  virtual void setTexture(const std::string& filename){
    if(capsule) capsule->getOSGPrimitive()->setTexture(filename);
  }

  virtual void setPose(const osg::Matrix& pose){
    this->pose = osg::Matrix::translate(0,0,height*0.5f+radius) * pose;
    if (!obstacle_exists) {
       create();
     }
     capsule->setPose(pose);
  };

  virtual Primitive* getMainPrimitive() const { return capsule; }

 protected:
  virtual void create(){
    capsule->setTextures(getTextures(0));
    if (mass==0.0) {
      capsule->init(odeHandle, mass, osgHandle, Primitive::Geom | Primitive::Draw);
    } else {
      capsule->init(odeHandle, mass, osgHandle);
    }
    obstacle_exists=true;
  };

};

}

#endif

