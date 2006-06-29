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
 *   Revision 1.1.2.1  2006-06-29 16:43:20  robot3
 *   abstract classes have now own .cpp-files
 *
 *   Revision 1.8.4.8  2006/06/16 22:27:26  martius
 *   getMainPrimtive
 *
 *   Revision 1.8.4.7  2006/05/23 13:38:02  robot3
 *   -fixed some creating bugs
 *   -setColor,setTexture and createGround must be
 *    called before setPosition now
 *
 *   Revision 1.8.4.6  2006/05/18 12:54:24  robot3
 *   -fixed not being able to change the color after positioning
 *    the obstacle
 *   -cleared the files up
 *
 *   Revision 1.8.4.5  2006/05/11 08:59:15  robot3
 *   -fixed a positioning bug (e.g. for passivesphere)
 *   -some methods moved to abstractobstacle.h for avoiding inconsistencies
 *
 *   Revision 1.8.4.4  2006/03/30 12:34:51  martius
 *   documentation updated
 *
 *   Revision 1.8.4.3  2006/03/29 15:04:38  martius
 *   have pose now
 *
 *   Revision 1.8.4.2  2005/12/06 10:13:23  martius
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
#include "abstractobstacle.h"

namespace lpzrobots {

  /**
   * Constructor
   * @param odeHandle containing ODE stuff like world, space and jointgroup
   * @param osgHandle containing OSG stuff like scene, color...
   * be used for creation of obstacles
   */
  AbstractObstacle::AbstractObstacle(const OdeHandle& odeHandle, const OsgHandle& osgHandle)
    : odeHandle(odeHandle), osgHandle(osgHandle) {
    // initialize the pose matrix correctly
    pose=osg::Matrix::translate(0,0,0);
    obstacle_exists=false;
  };

  AbstractObstacle::~AbstractObstacle(){}
  
  /**
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  void AbstractObstacle::setPosition(const osg::Vec3& pos) {
    pose.setTrans(pos);
    setPose(pose);
  };

  /**
   * gives actual position of the obstacle
   */
  osg::Vec3 AbstractObstacle::getPosition(){
    return pose.getTrans();
  }

  /**
   * gives actual pose of the obstacle
   */
  osg::Matrix AbstractObstacle::getPose(){ return pose; }


  /**
   * sets the obstacle color
   * @param color values in RGBA
   */
  void AbstractObstacle::setColor(const Color& color) {
    osgHandle.color = color;
    if (obstacle_exists) {
      destroy();
      create();
    }
    obstacle_exists=true;
  };

}
