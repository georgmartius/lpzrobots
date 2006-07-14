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
 *   Revision 1.2  2006-07-14 12:23:33  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.11  2006/06/16 22:27:26  martius
 *   getMainPrimtive
 *
 *   Revision 1.1.2.10  2006/05/28 22:14:56  martius
 *   heightfield included
 *
 *   Revision 1.1.2.9  2006/05/18 12:54:24  robot3
 *   -fixed not being able to change the color after positioning
 *    the obstacle
 *   -cleared the files up
 *
 *   Revision 1.1.2.8  2006/05/11 08:59:15  robot3
 *   -fixed a positioning bug (e.g. for passivesphere)
 *   -some methods moved to abstractobstacle.h for avoiding inconsistencies
 *
 *   Revision 1.1.2.7  2006/03/31 09:59:23  fhesse
 *   in create: z+=radius; added to place sphere on ground
 *
 *   Revision 1.1.2.6  2006/03/30 12:34:51  martius
 *   documentation updated
 *
 *   Revision 1.1.2.5  2006/03/29 15:04:39  martius
 *   have pose now
 *
 *   Revision 1.1.2.4  2006/01/18 16:46:39  martius
 *   mass adjustable
 *
 *   Revision 1.1.2.3  2005/12/15 17:02:16  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.2  2005/12/11 23:35:07  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.1  2005/12/09 16:53:17  martius
 *   camera is working now
 *
 *   Revision 1.5  2005/10/25 19:26:57  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.4  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.3  2005/07/31 22:30:56  martius
 *   textures
 *
 *   Revision 1.2  2005/07/18 14:52:33  martius
 *   world and space are not pointers anymore.
 *
 *   Revision 1.1  2005/07/08 10:00:33  fhesse
 *   initial version
 *                                                    *
 *                                                                         *
 ***************************************************************************/
#ifndef __PASSIVESPHERE_H
#define __PASSIVESPHERE_H

#include <stdio.h>
#include <math.h>

#include "primitive.h"
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
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), radius(radius), mass(mass) {       
    sphere=0;
    obstacle_exists=false;    
  };

  ~PassiveSphere(){
    if(sphere) delete sphere;
  }

  /**
   * update position of sphere
   */
  virtual void update(){
    if(sphere) sphere->update();
  };

  virtual void setTexture(const std::string& filename){
    if(sphere) sphere->getOSGPrimitive()->setTexture(filename);
  }
  
  virtual void setPose(const osg::Matrix& pose){
    this->pose = osg::Matrix::translate(0,0,radius) * pose;
    if (obstacle_exists){
      destroy();
    }
    create();
  };

  virtual Primitive* getMainPrimitive() const { return sphere; }

  
 protected:
  virtual void create(){
    sphere = new Sphere(radius);
    sphere->init(odeHandle, mass, osgHandle);
    osg::Vec3 pos=pose.getTrans();
    sphere->setPosition(pos);
        
    obstacle_exists=true;
  };


  virtual void destroy(){
    if(sphere) delete sphere;    
    obstacle_exists=false;
  };

};

}

#endif
