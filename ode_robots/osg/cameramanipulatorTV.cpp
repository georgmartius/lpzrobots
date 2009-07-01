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
 ***************************************************************************
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.7  2009-07-01 08:55:22  guettler
 *   new method which checks if agent is defined and in global list,
 *   if not, use the first agent of global list
 *   --> all camera manipulators fixed
 *
 *   Revision 1.6  2009/07/01 08:07:59  guettler
 *   bugfix: if agent not in global list,
 *   use the first agent of global list
 *
 *   Revision 1.5  2007/12/13 07:04:53  der
 *   fixed a stupid bug created through under sleep ;)
 *
 *   Revision 1.4  2007/12/12 10:27:31  der
 *   fixed some nullpointer bugs
 *
 *   Revision 1.3  2007/07/03 13:15:17  martius
 *   odehandle.h in cpp files included
 *
 *   Revision 1.2  2006/07/14 12:23:34  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.6  2006/03/19 13:36:52  robot3
 *   race mode now works
 *
 *   Revision 1.1.2.5  2006/03/18 13:54:42  robot3
 *   syntax error fix
 *
 *   Revision 1.1.2.4  2006/03/18 13:49:05  robot3
 *   TV mode works now, the appropiate pan and tilt are calculated
 *
 *   Revision 1.1.2.3  2006/03/06 16:57:53  robot3
 *   minor changes
 *
 *   Revision 1.1.2.2  2006/03/03 12:08:50  robot3
 *   preparations made for new cameramanipulators
 *
 *   Revision 1.1.2.1  2006/02/01 10:24:34  robot3
 *   new camera manipulator added
 *
 *                                                                         *
 ***************************************************************************/

#include <osg/Notify>
#include "cameramanipulatorTV.h"
#include "mathutils.h"
#include "pos.h"
#include "odeagent.h"
#include <stdio.h>


#define square(x) ((x)*(x))

namespace lpzrobots {

  using namespace osg;
  using namespace osgGA;

  CameraManipulatorTV::CameraManipulatorTV(osg::Node* node,GlobalData& global)
    : CameraManipulator(node,global) {}

  CameraManipulatorTV::~CameraManipulatorTV(){}


  void CameraManipulatorTV::calcMovementByAgent() {
    if (!this->isWatchingAgentDefined()) return;
    // the actual position of the agent has to be recognized
    // we use the Position getPosition() from OdeRobot
    Position robPos = watchingAgent->getRobot()->getPosition();
    // desiredEye is the position of the camera
    // calculate the horizontal angle, means pan (view.x)
    if (robPos.x-desiredEye[0]!=0) { // division by zero
      desiredView[0]= atan((desiredEye[0]-robPos.x)/(robPos.y-desiredEye[1]))
        / PI*180.0f+180.0f;
      if (desiredEye[1]-robPos.y<0) // we must switch
                desiredView[0]+=180.0f;
    }
    // calculate the vertical angle
    if (robPos.z-desiredEye[2]!=0) { // division by zero
      // need dz and sqrt(dx^2+dy^2) for calulation
      desiredView[1]=-atan((sqrt(square(desiredEye[0]-robPos.x)+
                                square(desiredEye[1]-robPos.y)))
                          /(robPos.z-desiredEye[2]))
        / PI*180.0f-90.0f;
      if (desiredEye[2]-robPos.z<0) // we must switch
        desiredView[1]+=180.0f;
    }
  }


    void CameraManipulatorTV::setHomeViewByAgent() {
    // the default camera manipulator does not need to change the view
    // normally the desired view should be changed
  }


  void CameraManipulatorTV::setHomeEyeByAgent() {
    // the default camera manipulator does not need to change the eye
    // normally the desired eye should be changed
  }
}
