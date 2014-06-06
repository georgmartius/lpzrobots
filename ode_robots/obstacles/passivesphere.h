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
#ifndef __PASSIVESPHERE_H
#define __PASSIVESPHERE_H

#include <stdio.h>
#include <cmath>

#include "primitive.h"
#include "osgprimitive.h"
#include "abstractobstacle.h"

namespace lpzrobots {

/**
 *  (Passive) sphere as obstacle
 */
class PassiveSphere : public AbstractObstacle{
  double radius;
  double mass;
  int texture;

  Sphere* sphere;

 public:

  /**
   * Constructor
   */
  PassiveSphere(const OdeHandle& odeHandle, const OsgHandle& osgHandle, double radius = 0.3, double mass = 1.0):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), radius(radius), mass(mass), texture(0) {
    sphere = new Sphere(radius);
    obst.push_back(sphere);
    obstacle_exists=false;
  };

  virtual void setTexture(const std::string& filename){
    if(sphere) sphere->getOSGPrimitive()->setTexture(filename);
  }

  virtual void setPose(const osg::Matrix& pose){
    this->pose = osg::Matrix::translate(0,0,radius) * pose;
    if (!obstacle_exists) {
       create();
     }
     sphere->setPose(pose);
  };

  virtual Primitive* getMainPrimitive() const { return sphere; }


 protected:
  virtual void create(){
    sphere->setTextures(getTextures(0));
    if (mass==0.0) {
      sphere->init(odeHandle, mass, osgHandle, Primitive::Geom | Primitive::Draw);
     } else {
      sphere->init(odeHandle, mass, osgHandle);
    }
    obstacle_exists=true;
  };

};

}

#endif
