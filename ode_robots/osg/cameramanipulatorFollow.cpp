/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
      Position robMove = (camHandle.watchingAgent->getRobot()->getPosition()-camHandle.oldPositionOfAgent);
      // attach the robSpeed to desired eye
      for (int i=0;i<=2;i++) {
        if (!isNaN(robMove.toArray()[i])) {
          camHandle.desiredEye[i]+=robMove.toArray()[i];}
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
