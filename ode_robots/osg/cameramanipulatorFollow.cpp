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
 *   Revision 1.6  2009-07-30 11:52:53  guettler
 *   new CameraHandle replacing static variables in the CameraManipulators
 *
 *   Revision 1.5  2009/07/01 08:55:22  guettler
 *   new method which checks if agent is defined and in global list,
 *   if not, use the first agent of global list
 *   --> all camera manipulators fixed
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
 *   Revision 1.1.2.4  2006/03/19 13:37:29  robot3
 *   race mode now works
 *
 *   Revision 1.1.2.3  2006/03/19 10:51:32  robot3
 *   follow mode now centers the view on the robot
 *   if the robot is choosed (only once)
 *
 *   Revision 1.1.2.2  2006/03/08 13:17:33  robot3
 *   follow mode now works
 *
 *   Revision 1.1.2.1  2006/03/06 17:00:44  robot3
 *   first dummy follow version
 *
 *                                                                         *
 ***************************************************************************/

#include <osg/Notify>
#include "cameramanipulatorFollow.h"
//#include "mathutils.h"
#include "pos.h"
#include "odeagent.h"

namespace lpzrobots {

  using namespace osg;
  using namespace osgGA;

  CameraManipulatorFollow::CameraManipulatorFollow(osg::Node* node,GlobalData& global, CameraHandle& cameraHandle)
    : CameraManipulator(node,global, cameraHandle) {}

  CameraManipulatorFollow::~CameraManipulatorFollow(){}

  void CameraManipulatorFollow::calcMovementByAgent() {
    if (this->isWatchingAgentDefined() && camHandle.oldPositionOfAgentDefined) {
      // then manipulate desired view and desired eye
      const double* robMove = (camHandle.watchingAgent->getRobot()->getPosition()-camHandle.oldPositionOfAgent).toArray();
      // attach the robSpeed to desired eye
      for (int i=0;i<=2;i++) {
	if (!isNaN(robMove[i])) {
	  camHandle.desiredEye[i]+=robMove[i];}
	else
	  std::cout << "NAN exception!" << std::endl;
      }
    }
  }


  void CameraManipulatorFollow::setHomeViewByAgent() {
    // ok here the camera will center on the robot
    if (!this->isWatchingAgentDefined()) return;
    // the actual position of the agent has to be recognized
    // we use the Position getPosition() from OdeRobot
    Position robPos = camHandle.watchingAgent->getRobot()->getPosition();
    // desiredEye is the position of the camera
    // calculate the horizontal angle, means pan (view.x)
    if (robPos.x-camHandle.desiredEye[0]!=0) { // division by zero
      camHandle.desiredView[0]= atan((camHandle.desiredEye[0]-robPos.x)/(robPos.y-camHandle.desiredEye[1]))
        / PI*180.0f+180.0f;
      if (camHandle.desiredEye[1]-robPos.y<0) // we must switch
        camHandle.desiredView[0]+=180.0f;
    }
    // calculate the vertical angle
    if (robPos.z-camHandle.desiredEye[2]!=0) { // division by zero
      // need dz and sqrt(dx^2+dy^2) for calulation
      camHandle.desiredView[1]=-atan((sqrt(square(camHandle.desiredEye[0]-robPos.x)+
                                square(camHandle.desiredEye[1]-robPos.y)))
                          /(robPos.z-camHandle.desiredEye[2]))
        / PI*180.0f-90.0f;
      if (camHandle.desiredEye[2]-robPos.z<0) // we must switch
        camHandle.desiredView[1]+=180.0f;
    }
  }

}
