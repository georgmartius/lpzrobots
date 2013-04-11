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
#ifndef __PASSIVEMESH_H
#define __PASSIVEMESH_H

#include <stdio.h>
#include <cmath>

#include "primitive.h"
#include "osgprimitive.h"
#include "abstractobstacle.h"

namespace lpzrobots {

/**
 *  (Passive) mesh as obstacle
 */
class PassiveMesh : public AbstractObstacle{
  std::string filename;
  float scale;
  double mass;

  Mesh* mesh;

 public:

  /**
   * Constructor
   */
  PassiveMesh(const OdeHandle& odeHandle,
              const OsgHandle& osgHandle,
              const std::string& filename,
              double scale = 1.0, double mass = 1.0):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle),
    filename(filename), scale(scale), mass(mass){
    mesh=0;
    obstacle_exists=false;
  };

  ~PassiveMesh(){
    if(mesh) delete mesh;
  }

  /**
   * update position of mesh
   */
  virtual void update(){
    if(mesh) mesh->update();
  };

/*   virtual void setTexture(const std::string& filename){ */
/*     if(mesh) mesh->getOSGPrimitive()->setTexture(filename); */
/*   } */

  virtual void setPose(const osg::Matrix& pose){
    this->pose = pose;
    if (obstacle_exists){
      destroy();
    }
    create();
  };


  virtual Primitive* getMainPrimitive() const { return mesh; }

 protected:

  virtual void create(){
    mesh = new Mesh(filename,scale);
    mesh->init(odeHandle, mass, osgHandle);
//     osg::Vec3 pos=pose.getTrans();
//     pos[2]+=mesh->getRadius();
    mesh->setPose(pose);
    obstacle_exists=true;
  };


  virtual void destroy(){
    if(mesh) delete mesh;
    obstacle_exists=false;
  };

};

}

#endif
