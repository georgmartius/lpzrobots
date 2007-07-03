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
 *   Revision 1.3  2007-07-03 13:15:17  martius
 *   odehandle.h in cpp files included
 *
 *   Revision 1.2  2006/07/14 12:23:34  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2006/03/19 13:35:10  robot3
 *   race mode now works
 *
 *
 *                                                                         *
 ***************************************************************************/

#include <osg/Notify>
#include "cameramanipulatorRace.h"
#include "mathutils.h"
#include "pos.h"
#include "odeagent.h"
#include <stdio.h>


#define square(x) ((x)*(x))

namespace lpzrobots {

  using namespace osg;
  using namespace osgGA;

  CameraManipulatorRace::CameraManipulatorRace(osg::Node* node,GlobalData& global)
    : CameraManipulator(node,global) {}

  CameraManipulatorRace::~CameraManipulatorRace(){}


  void CameraManipulatorRace::calcMovementByAgent() {
    if (watchingAgent!=NULL) {
      // manipulate desired eye by the move of the robot
      const double* robMove = (watchingAgent->getRobot()->getPosition()-oldPositionOfAgent).toArray();
      // attach the robSpeed to desired eye
      for (int i=0;i<=2;i++) {
	if (!isNaN(robMove[i])) {
	  desiredEye[i]+=robMove[i];}
	else 
	  std::cout << "NAN exception!" << std::endl;
      }
      // move behind the robot
      // returns the orientation of the robot in matrix style
      matrix::Matrix Orientation= (watchingAgent->getRobot()->getOrientation());
      Orientation.toTranspose();
      // first get the normalized vector of the orientation
      double eVecX[3] = {0,1,0};
      double eVecY[3] = {1,0,0};
      matrix::Matrix normVecX = Orientation * matrix::Matrix(3,1,eVecX);
      matrix::Matrix normVecY = Orientation * matrix::Matrix(3,1,eVecY);
      // then get the distance between robot and camera
      Position robPos = watchingAgent->getRobot()->getPosition();
      double distance = sqrt(square(desiredEye[0]-robPos.x)+
			     square(desiredEye[1]-robPos.y));
      // then new eye = robPos minus normalized vector * distance
      desiredEye[0]=robPos.x + distance *normVecX.val(1,0);
      desiredEye[1]=robPos.y - distance *normVecY.val(1,0);

      // now do center on the robot (manipulate the view)
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
  }
}
