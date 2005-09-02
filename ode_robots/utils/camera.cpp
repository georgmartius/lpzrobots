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
 *   Revision 1.8  2005-09-02 17:20:18  martius
 *   advancedTV disabled
 *
 *   Revision 1.7  2005/08/23 11:41:20  robot1
 *   advancedFollowing mode included
 *
 *   Revision 1.6  2005/08/22 12:38:32  robot1
 *   -advancedTV mode implemented, early version
 *   -internal code optimized
 *   -printMode prints now the current camera mode on stdout
 *
 *   Revision 1.5  2005/08/16 10:06:48  robot1
 *   TV mode with horizontal centering implemented.
 *
 *   Revision 1.4  2005/08/12 11:56:46  robot1
 *   tiny bugfixing
 *
 *   Revision 1.3  2005/08/09 11:08:49  robot1
 *   following mode included
 *
 *
 *   Revision 1.1  2005/08/08 11:06:46  martius
 *   camera is a module for camera movements
 *   includes cleaned
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

// #define SQUARE(x) (x)*(x)
 
#include "camera.h" 
#include <stdio.h>

#include <drawstuff/drawstuff.h>
#include "ode/ode.h" 

CameraType oldCamType;
const AbstractRobot* oldRobot;

// camera points, first is position, second the point to view
// double KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
float camPos[3];
//  double KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};
float camView[3]; // special angle coordinates
// the position of robot
double robotPos[3];
// the view of robot
double robotView[3]; // simple direction vector, should be normalized
 
// the new positions and view of the camera and the robot
float newCamPos[3];
float newCamView[3];
double newRobotPos[3]; // special angle coordinates
double newRobotView[3]; // simple direction vector, should be normalized

// for advanced TV mode
float maxAllowedDistance=10000.0f; // Disabled advanvedTV
float robCamDistance;

// this is a bit faster than the compiler definition (x is only calculated once, not twice)
float SQUARE(float x) {
	return (x*x);

}



void getRobotPosAndView(double *pos, double *view,AbstractRobot& robot) {
	Position position=robot.getPosition();
	pos[0]=position.x;
	pos[1]=position.y;
	pos[2]=position.z;
	// view not yet included
}

void printMode(CameraType camType) {
	char* type;
	printf("Current Mode is now: ");
	switch (camType) {
		case Static:
			type="Static";
			break;
		case TV:
			type="TV";
			break;
		case advancedTV:
			type="advancedTV";
			break;
		case Following:
			type="Following";
			break;
		case advancedFollowing:
			type="advancedFollowing";
			break;
		default:
			type="unknown";
	}
	printf("%s\n",type);
}

// has to be called if CameraType or the Robot has changed.
void initCamera(CameraType camType,AbstractRobot& robot) {
	if (oldCamType!=camType)
		printMode(camType);
	// getting first the position and view of robot
	getRobotPosAndView(robotPos, robotView, robot);
	// now getting the current angle of the camera
	dsGetViewpoint(camPos,camView);
	// setting the new Values
	oldCamType=camType;
	oldRobot=&robot;
}

void StaticMode(AbstractRobot& robot) {
	// do nothing
}


void TVMode(AbstractRobot& robot) {
	// now adjusting the original position of the camera
	// new values must be stored as old too
	for (int i=0;i<=2;i++) {
 		if (oldCamType==TV) // because advanced mode uses this routine too
			camPos[i]=newCamPos[i]; // no change
		robotPos[i]=newRobotPos[i];
		// robotView[i]=newRobotView[i]; // not used yet
	}
	camView[2]=newCamView[2]; // no change for z
	if ((newRobotPos[1]-newCamPos[1])!=0) {
					// calculate the horizontal angle
		camView[0]=-atan((newRobotPos[0]-newCamPos[0])/(newRobotPos[1]-newCamPos[1]))
							/3.14159265f*180.0f+270.0f;
		if (newCamPos[1]-newRobotPos[1]<0)
			camView[0]+=180.0f; // we must switch
	} else { // do not compute because of division by zero, take old values
		camView[0]=newCamView[0];
	}
	if ((newRobotPos[2]-newCamPos[2])!=0) {
					// calculate the vertical angle
					// we need dz and sqrt(dx^2+dy^2) for calculation
		camView[1]=atan(
				(sqrt(
				SQUARE(newCamPos[0]-newRobotPos[0]) +
				SQUARE(newCamPos[1]-newRobotPos[1])))
							/(newCamPos[2]-newRobotPos[2]))
							/3.14159265f*180.0f+270.0f;
	} else { // do not compute because of division by zero, take old value
		camView[1]=newCamView[1];
	}
}

void advancedTVMode(AbstractRobot& robot) {
	float n;
	// check wether the camera is too far away
	robCamDistance= (sqrt(
			SQUARE(newCamPos[0]-newRobotPos[0]) +
			SQUARE(newCamPos[1]-newRobotPos[1])));
	if (robCamDistance>maxAllowedDistance) {
		// then the camera must get a new position
		printf("Cam is too far away!\n");
		camPos[2]=newCamPos[2]; // will not be changed
		// first get the direction of the robot
		for (int i=0;i<=1;i++) {
			robotView[i]=newRobotPos[i]-robotPos[i];
		}
		// now get normalizing scalar of robotView, only x and y is needed
		n=sqrt(SQUARE(robotView[0])+SQUARE(robotView[1]));
		if (n>0.0001) { // else no calculation can be made, then do not change the camPos
			for (int i=0;i<=1;i++) {
				robotView[i]*=1/n; // normalizing every dimension
				// now shift the camPos by robCamDistance*1,5*robotView
				// this is an approximation, normally a rotation is needed
				camPos[i]=newCamPos[i]+1.5f*robotView[i]*robCamDistance;
			}
			if ((sqrt(SQUARE(camPos[0]-newRobotPos[0])+SQUARE(camPos[1]-newRobotPos[1])))
						  >maxAllowedDistance) {
				// the distance is not in range, the cam would flip away
				// set default values
				// then add a half of this distance so the cam is not in front of the robot
				// this is a rotation with an angle of 90 degrees
				camPos[0]=newRobotPos[0]-
						(0.25f*robotView[0]+0.125f*robotView[1])*maxAllowedDistance;
				camPos[0]=newRobotPos[0]-
						(0.25f*robotView[1]-0.125f*robotView[0])*maxAllowedDistance;
			}
		} else { // do not change the camPos, take old values
			for (int i=0;i<=1;i++) {
				camPos[i]=newCamPos[i]; // no change
			}
		}
	} else {
		for (int i=0;i<=2;i++) {
			camPos[i]=newCamPos[i]; // no change
		}
	}
	// now do the rest of TV
	TVMode(robot);
}

void FollowingMode(AbstractRobot& robot) {
	// now adjusting the original position of the camera
	// new values must be stored as old too
	for (int i=0;i<=2;i++) {
		robotView[i]=newRobotPos[i]-robotPos[i];
		camPos[i]=newCamPos[i]+newRobotPos[i]-robotPos[i];
		camView[i]=newCamView[i]; // no change
		robotPos[i]=newRobotPos[i];
	}
}

void print3DimFloat(float vec[3]) {
	for (int i=0;i<=2;i++) {
		printf("\t%f",vec[i]);
	}
}


void advancedFollowingMode(AbstractRobot& robot) {
	float n;
	robCamDistance= (sqrt(
			SQUARE(newCamPos[0]-newRobotPos[0]) +
			SQUARE(newCamPos[1]-newRobotPos[1])));
//    	robCamDistance=2.5;
 	printf("distance: %f\n",robCamDistance);
	// get the direction of the robot
	for (int i=0;i<=1;i++) {
		robotView[i]=newRobotPos[i]-robotPos[i];
	}
	// now get normalizing scalar of robotView, only x and y is needed
	n=sqrt(SQUARE(robotView[0])+SQUARE(robotView[1]));
	if (n>0.0001) { // else no calculation can be made, then do not change the camPos
		for (int i=0;i<=1;i++) {
			// now set the camPos to robotPos-robCamDistance*robotView
			// this is an approximation, normally a rotation is needed
			// store it in newCamPos, so TV mode can be called unmodified
			camPos[i]=newRobotPos[i]-robotView[i]*robCamDistance/n+robotView[i];
		}
		camPos[2]=newCamPos[2];
	} else { // else do not change the camPos here
		for (int i=0;i<=2;i++) {
			camPos[i]=newCamPos[i];
		}
	}
	// now do centering on robot, using TV mode
	TVMode(robot);
}

void moveCamera( CameraType camType,AbstractRobot& robot) {
	// first look if someone is changed
	// only otherwise change the camera position and/or view.
	if (oldCamType!=camType || &robot!=oldRobot)
		initCamera(camType,robot);
	else {
		// first get all needed values
		if (camType!=Static) {
			// getting first the position and view of robot
			getRobotPosAndView(newRobotPos,newRobotView,robot);
			// now getting the current angle of the camera
			dsGetViewpoint(newCamPos,newCamView);
		}
		// now compute
		// to compute is the new camPos and the new camView.
		// in addition the new robotPos and robotView have to be stored.
		switch (camType) {
			case Static:
				StaticMode(robot);
				break;
			case advancedTV:
				advancedTVMode(robot);
				break;
			case TV:
				TVMode(robot);
				break;
			case Following:
				FollowingMode(robot);
				break;
			case advancedFollowing:
				advancedFollowingMode(robot);
				break;
		}
		// now execute :)
		if (camType!=Static) {
			dsSetViewpoint(camPos,camView);
		}
	}
}
