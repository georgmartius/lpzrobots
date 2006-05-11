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
 *   Revision 1.1.2.3  2006-05-11 08:59:15  robot3
 *   -fixed a positioning bug (e.g. for passivesphere)
 *   -some methods moved to abstractobstacle.h for avoiding inconsistencies
 *
 *   Revision 1.1.2.2  2006/03/30 12:34:51  martius
 *   documentation updated
 *
 *   Revision 1.1.2.1  2006/03/29 15:04:39  martius
 *   have pose now
 *
 *                                                    *
 *                                                                         *
 ***************************************************************************/
#ifndef __PASSIVEBOX_H
#define __PASSIVEBOX_H

#include <stdio.h>
#include <math.h>

#include "primitive.h"
#include "abstractobstacle.h"

namespace lpzrobots {

/**
 *  (Passive) box as obstacle
 */
class PassiveBox : public AbstractObstacle{
  double radius;
  double mass;
  int texture;

  Box* box;

  bool obstacle_exists;

 public:
  
  /**
   * Constructor
   */
  PassiveBox(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
	     const osg::Vec3& dimension = osg::Vec3(1.0, 1.0, 1.0), double mass = 1.0):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), dimension(dimension), mass(mass) {       
    box=0;
    obstacle_exists=false;    
  };

  ~PassiveBox(){
    if(box) delete box;
  }

  /**
   * update position of box
   */
  virtual void update(){
    if(box) box->update();
  };

  virtual void setTexture(const std::string& filename){
    if(box) box->getOSGPrimitive()->setTexture(filename);
  }
  
  virtual void setPose(const osg::Matrix& pose){
    this->pose = osg::Matrix::transform(0,0,radius) * pose;
    if (obstacle_exists){
      destroy();
    }
    create();
  };

 protected:
  virtual void create(){
    box = new Box(dimension.x(), dimension.y(), dimension.z());
    box->init(odeHandle, mass, osgHandle);
    osg::Vec3 pos=pose.getTrans();
    box->setPosition(pos);
        
    obstacle_exists=true;
  };


  virtual void destroy(){
    if(box) delete box;    
    obstacle_exists=false;
  };

};

}

#endif
