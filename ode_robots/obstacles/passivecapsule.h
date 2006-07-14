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
 *   Revision 1.2  2006-07-14 12:23:33  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.4  2006/06/16 22:27:26  martius
 *   getMainPrimtive
 *
 *   Revision 1.1.2.3  2006/05/18 15:40:32  robot3
 *   fixed setPose
 *
 *   Revision 1.1.2.2  2006/05/18 15:36:15  robot3
 *   fixed compiling bug
 *
 *   Revision 1.1.2.1  2006/05/18 13:06:51  robot3
 *   added a passive passive capsule
 *
 *
 *                                                    *
 *                                                                         *
 ***************************************************************************/
#ifndef __PASSIVECAPSULE_H
#define __PASSIVECAPSULE_H

#include <stdio.h>
#include <math.h>

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
    capsule=0;
    obstacle_exists=false;    
  };

  ~PassiveCapsule(){
    if(capsule) delete capsule;
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
    if (obstacle_exists){
      destroy();
    }
    create();
  };

  virtual Primitive* getMainPrimitive() const { return capsule; }

 protected:
  virtual void create(){
    capsule = new Capsule(radius,height);
    capsule->init(odeHandle, mass, osgHandle);
    osg::Vec3 pos=pose.getTrans();
    capsule->setPosition(pos);
        
    obstacle_exists=true;
  };

  virtual void destroy(){
    if(capsule) delete capsule;    
    obstacle_exists=false;
  };

};

}

#endif

