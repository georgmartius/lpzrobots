/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.1.2.1  2006-05-19 08:41:27  robot3
 *   Class AbstractGround contains now basic routines like
 *   creating the groundPlane, setPose and so on
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __ABSTRACTGROUND_H
#define __ABSTRACTGROUND_H

//#include <math.h>
#include <vector>
#include <osg/Matrix>

#include "primitive.h"
#include "abstractobstacle.h"

namespace lpzrobots {

  // abstract class for any playground
  class AbstractGround : public AbstractObstacle {



public:
  

  AbstractGround(const OdeHandle& odeHandle, const OsgHandle& osgHandle, bool createGround=true):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), creategroundPlane(createGround) {
    obstacle_exists=false;
    ground_length=10.0f;
  };
  
  virtual ~AbstractGround(){
    destroy();
    obst.clear();
  }

  virtual void update(){
    for (unsigned int i=0; i<obst.size(); i++){
      if(obst[i]) obst[i]->update();
    }
    if (groundPlane) groundPlane->update();
  };
  

  virtual void setPose(const osg::Matrix& pose){
    this->pose = pose;
    if (obstacle_exists){
      destroy();
    }
    create();
    createGround();
  };

  virtual void createGround(bool create) {
    creategroundPlane=create;
    if (obstacle_exists){
      destroy();
      this->create();
      this->createGround();
    }
  }

protected:

    vector<Primitive*> obst; //obstacles
    Box* groundPlane; // the groundplane
    bool creategroundPlane;
    double ground_length;

    virtual void create()=0;

    virtual void createGround() {
      if (creategroundPlane) {
	// now create the plane in the middle
	groundPlane = new Box(ground_length,ground_length, 0.1f);
	groundPlane->init(odeHandle, 0, osgHandle,
			  Primitive::Geom | Primitive::Draw);
	groundPlane->setPose(osg::Matrix::translate(0.0f,0.0f,-0.05f) * pose);
	groundPlane->getOSGPrimitive()->setColor(Color(1.0f,1.0f,1.0f));
	groundPlane->getOSGPrimitive()->setTexture("Images/greenground.rgb",true,true);
      }
    }


  virtual void destroy(){
    for(unsigned int i=0; i < obst.size(); i++){
      if(obst[i]) delete obst[i];
    }
    if (groundPlane) delete(groundPlane);
    obstacle_exists=false;
  };

};

}

#endif
