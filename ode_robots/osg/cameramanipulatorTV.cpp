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
 ***************************************************************************
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.3  2006-03-06 16:57:53  robot3
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
//#include "mathutils.h"
#include "pos.h"

namespace lpzrobots {

  using namespace osg;
  using namespace osgGA;

  CameraManipulatorTV::CameraManipulatorTV(osg::Node* node,GlobalData& global)
    : CameraManipulator(node,global) {}

  CameraManipulatorTV::~CameraManipulatorTV(){}

  
  void CameraManipulatorTV::calcMovementByAgent() {
    if (watchingAgent!=NULL) {
      //      std::cout << "an agent is availible!" << std::endl;
      // the actual position of the agent has to be recognized
      // we use the Position getPosition() from OdeRobot

      Position robPos = watchingAgent->getRobot()->getPosition();

      // the desired view of the camera has to be changed
      // for that the angles of "from the desired eye to robPos" are needed
      // we use  double getAngle(const osg::Vec3& a, const osg::Vec3& b); for this
      


      // then manipulate desired view and desired eye
      // the default camera manipulator does not need to change the eye and view
    } else {
      //      std::cout << "no agent choosed!" << std::endl;
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
