/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.1.2.1  2006-03-29 15:05:32  martius
 *   obstacle that uses a Mesh / OSGModel for graphics and a bounding shape for ODE
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __MESHOBSTACLE_H
#define __MESHOBSTACLE_H

#include <stdio.h>
#include <math.h>
#include <osg/BoundingSphere>

#include "primitive.h"
#include "abstractobstacle.h"
#include "boundingshape.h"
 
namespace lpzrobots {

class MeshObstacle : public AbstractObstacle {
protected:


  std::string filename;
  float scale;
  OSGMesh* mesh;
  Sphere* bound;
  BoundingShape* boundshape;
  bool obstacle_exists;

public:
  
  MeshObstacle(const OdeHandle& odeHandle, const OsgHandle& osgHandle , 
	       std::string filename, double scale = 1):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), 
    filename(filename), scale(scale) 
  {
    mesh = 0;
    bound = 0;
    boundshape = 0;
    obstacle_exists=false;    
  };

  /**
   * updates the position of the geoms  ( not nessary for static objects)
   */
  virtual void update(){
  };
  
  
  virtual void setPose(const osg::Matrix& pose){
    this->pose = pose;
    if (obstacle_exists){
      destroy();
    }
    create();
  };

  virtual osg::Matrix getPose(){
    return pose;
  }  

 protected:
  virtual void create(){
    
    mesh = new OSGMesh(filename, scale);
    mesh->init(osgHandle);
    mesh->setMatrix(pose);
    const osg::BoundingSphere& bsphere = mesh->getGroup()->getBound(); 

    boundshape = new BoundingShape(filename  + ".bbox" );
    if(!boundshape->init(odeHandle, osgHandle.changeColor(Color(0,1,0,0.2)), 
			 pose, scale, Primitive::Geom | Primitive::Draw)){
      printf("use default bounding box, because bbox file not found\n");
      bound = new Sphere(bsphere.radius()); 
      bound->init(odeHandle, 0, osgHandle.changeColor(Color(1,0,0,0.2)), Primitive::Geom | Primitive::Draw);    
      bound->setPose(osg::Matrix::translate(bsphere.center()));      
    }

    obstacle_exists=true;
  };


  virtual void destroy(){
    if(mesh) delete(mesh);
    if(bound) delete(bound);    
    if(boundshape) delete(boundshape);    
    mesh=0;
    bound=0;
    boundshape=0;
    obstacle_exists=false;
  };

};

}

#endif
