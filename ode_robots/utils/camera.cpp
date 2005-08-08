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
 *   Revision 1.2  2005-08-08 14:11:28  robot1
 *   FUCKTHECODE for 2 weeks
 *
 *   Revision 1.1  2005/08/08 11:06:46  martius
 *   camera is a module for camera movements
 *   includes cleaned
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include "camera.h" 

#include <drawstuff/drawstuff.h>
#include "ode/ode.h" 

CameraType oldcamType;
AbstractRobot& oldRobot;

// camera points, first is position, second the point to view
// double KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
float camPos[3];
//  double KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};
float camView[3];
// the position of robot
double robotPos[3];
// the view of robot
double RobotView[3];
 
// the new positions and view of the camera and the robot
float newCamPos[3];
float newCamView[3];
double newRobotPos[3];
double newRobotView[3];

void getRobotPosAndView(double *robotPos, double *robotView,AbstractRobot& robot) {
	Position pos=vehicle->getPosition();
	RobotPos[0]=pos.x;
	RobotPos[1]=pos.y;
	RobotPos[2]=pos.z;
	// robotView not yet included
}

// has to be called if CameraType or the Robot has changed.
void initCamera(CameraType camType, const AbstractRobot& robot) {
	// getting first the position and view of robot
	getRobotPosAndView(robotPos, robotView, robot);
	// now getting the current angle of the camera
	dsGetViewpoint(CamPos,CamView);
	// setting the new Values
	oldcamType=camType;
	oldRobot=robot;
}


void moveCamera( CameraType camType, const AbstractRobot& robot) {
	// first look if someone is changed
	// only otherwise change the camera position and/or view.
	if (oldcamType!=camType || robot!=oldRobot)
		initCamera(camType,robot);
	else {
		// first get all needed values
		if (CameraType!=Static) {
			// getting first the position and view of robot
			getRobotPosAndView(newRobotPos,newRobotView,robot);
			// now getting the current angle of the camera
			dsGetViewpoint(newCamPos,newCamView);
		}
		// now compute
		// to compute is the new camPos and the new camView.
		// in addition the new robotPos and robotView have to be stored.
		switch (CameraType) {
			case Static:
				break; // do nothing
			case TV:
				break;
			case Following:
				// now adjusting the original position of the camera
				// new values must be stored as old too
				for (int i=0;i<=2;i++) {
					camPos[i]=newCamPos[i]+newRobotPos[i]-robotPos[i];
					camView[i]=newCamView[i]; // no change
					robotPos[i]=newRobotPos[i];
					// robotView[i]=newRobotView[i]; // not used yet
				}
				break;
		}
		// now execute :)
		if (CameraType!=Static) {
			dsSetViewpoint(camPos,camView);
		}
	}
}