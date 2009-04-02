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
 *   Revision 1.7  2009-04-02 10:12:25  martius
 *   Texture handling changed
 *
 *   Revision 1.6  2009/01/09 16:52:36  martius
 *   use pose instead of translation only
 *
 *   Revision 1.5  2008/09/16 14:49:46  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.4  2007/12/06 15:59:19  der
 *   if you set the mass=0.0, a passive box without a body is created
 *
 *   Revision 1.3  2007/03/16 11:01:37  martius
 *   abstractobstacle gets mor functionallity
 *   setSubstance
 *
 *   Revision 1.2  2006/07/14 12:23:33  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.6  2006/06/16 22:27:26  martius
 *   getMainPrimtive
 *
 *   Revision 1.1.2.5  2006/05/18 12:54:24  robot3
 *   -fixed not being able to change the color after positioning
 *    the obstacle
 *   -cleared the files up
 *
 *   Revision 1.1.2.4  2006/05/11 12:53:04  robot3
 *   fixed some errors in passivebox.h
 *
 *   Revision 1.1.2.3  2006/05/11 08:59:15  robot3
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
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), dimension(dimension), mass(mass) {       
    box = new Box(dimension.x(), dimension.y(), dimension.z());
    obst.push_back(box);
    obstacle_exists=false;    
  };

  
  virtual void setPose(const osg::Matrix& pose){
    this->pose = osg::Matrix::translate(0,0,dimension.z()/2) * pose;
    if (obstacle_exists){
      destroy();
    }
    create();
  };

  virtual Primitive* getMainPrimitive() const { return box; }

 protected:
  virtual void create(){
    if (mass==0.0) {
      box->init(odeHandle, mass, osgHandle, Primitive::Geom | Primitive::Draw);
    } else {
      box->init(odeHandle, mass, osgHandle);
    }
//     osg::Vec3 pos=pose.getTrans();
//     box->setPosition(pos);
    box->setPose(pose);
    obst.push_back(box);

    obstacle_exists=true;
  };

};

}

#endif

