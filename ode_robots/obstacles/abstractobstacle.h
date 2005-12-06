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
 *   Revision 1.8.4.2  2005-12-06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.8.4.1  2005/11/14 17:37:14  martius
 *   moved to selforg
 *
 *   Revision 1.8  2005/10/25 19:26:56  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.7  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.6  2005/09/13 13:20:12  martius
 *   initialised color
 *
 *   Revision 1.5  2005/08/29 06:32:25  martius
 *   added virtual destructor
 *
 *   Revision 1.4  2005/07/18 14:52:33  martius
 *   world and space are not pointers anymore.
 *
 *   Revision 1.3  2005/06/15 14:22:11  martius
 *   GPL included
 *                                                                 *
 ***************************************************************************/
#ifndef __ABSTRACTOBSTACLE_H
#define __ABSTRACTOBSTACLE_H

#include <ode/ode.h>

#include "odehandle.h"
#include "osghandle.h"

namespace lpzrobots {

/**
 *  Abstract class (interface) for obstacles
 */
class AbstractObstacle{

 public:
  /**
   * Constructor
   * @param odehandle containing world, space and jointgroup which should 
   * be used for creation of obstacles
   */
  AbstractObstacle(const OdeHandle& odeHandle, const OsgHandle& osgHandle)
    : odeHandle(odeHandle), osgHandle(osgHandle) {
  };

  virtual ~AbstractObstacle(){}
  
  /**
   * updates the position if the scenegraph nodes
   */
  virtual void update() = 0;
  
  /**
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  virtual void setPosition(const osg::Vec3& pos) = 0;

  /**
   * gives actual position of the obstacle
   */
  virtual osg::Vec3 getPosition() = 0;
    
  /**
   * sets the obstacle color
   * @param Vec4 color values in RGBA
   */
  virtual void setColor(const Color& color) {
    osgHandle.color = color;
  };

 protected:

  OdeHandle odeHandle;
  OsgHandle osgHandle; 

};

}

#endif
