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
 *   Revision 1.13  2005-09-27 14:00:29  robot3
 *   printf removed
 *
 *   Revision 1.12  2005/09/27 12:22:52  robot3
 *   now changing camView by mouse movement really works :)
 *
 *   Revision 1.11  2005/09/27 12:11:16  robot3
 *   fixed bug not be able to change the camView through mouse movement
 *
 *   Revision 1.10  2005/09/27 10:49:20  robot3
 *   camera module rewritten, new features:
 *   -everytime there is a smooth move and view
 *   -some hacks are replaced through stable code
 *
 *   Revision 1.9  2005/09/20 10:55:14  robot3
 *   camera module:
 *   -pressing key c now centers on focused robot
 *   -pressing key b now moves 5.0f behind the robot
 *   -fixed a few bugs (nullpointer crashes etc.)
 *
 *   Revision 1.8  2005/09/02 17:20:18  martius
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


 
#include "camera.h" 
#include <stdio.h>

#include <drawstuff/drawstuff.h>
#include "ode/ode.h" 

CameraType oldCamType;
const AbstractRobot* oldRobot;

// all desired values that have to be in the end state
float desiredCamPos[3];
float desiredCamView[3];

// for camera and robot: current (read and calculated) values
float currentCamPos[3];
float currentCamView[3];
double currentRobotPos[3];
double currentRobotView[3];

// for camera and robot: old values from one timestep before
float oldCamPos[3];
float oldCamView[3];
double oldRobotPos[3];
double oldRobotView[3];


// for advanced TV mode
float maxAllowedDistance=10.0f; // enabled advanvedTV ;)
float robCamDistance;



// this is a bit faster than the compiler definition:
// #define SQUARE(x) (x)*(x)
// x is only calculated once, not twice
double SQUARE(double x) {
	return (x*x);

}

// I didn't found the math library so fast, sorry.
float ABS(float x) {
  if (x<0)
    return (x*(-1));
  else
    return x;
}

void followRobotsMove() {
	// now adjusting the desired position of the camera
	for (int i=0;i<=2;i++) {
		desiredCamPos[i]+=currentRobotPos[i]-oldRobotPos[i];
	}
}

// changes only the desiredCamView
void centerViewOnRobot() {
	// no change for z
	if ((currentRobotPos[1]-desiredCamPos[1])!=0) { // division by zero
		// calculate the horizontal angle
		desiredCamView[0]=-atan((currentRobotPos[0]-desiredCamPos[0])/
					(currentRobotPos[1]-desiredCamPos[1]))
							/M_PI*180.0f+270.0f;
		if (desiredCamPos[1]-currentRobotPos[1]<0)
			desiredCamView[0]+=180.0f; // we must switch
	}
	if ((currentRobotPos[2]-desiredCamPos[2])!=0) { // division by zero
		// calculate the vertical angle
		// we need dz and sqrt(dx^2+dy^2) for calculation
		desiredCamView[1]=
			atan(
				(sqrt(
				SQUARE(desiredCamPos[0]-currentRobotPos[0]) +
				SQUARE(desiredCamPos[1]-currentRobotPos[1])))
					/(currentCamPos[2]-currentRobotPos[2]))
					/M_PI*180.0f+270.0f; // TODO: normalize
	}
	// z-angle (rotation) is not adjusted
}



void getRobotPosAndView(double *pos, double *view,AbstractRobot& robot) {
	Position position=robot.getPosition();
	pos[0]=position.x;
	pos[1]=position.y;
	pos[2]=position.z;
	// now the view
	for (int i=0;i<=2;i++) {
		view[i]=pos[i]-oldRobotPos[i];
        }
	// if no view can be detected, use old view!
	if (view[0]==0 && view[1]==0) {
		for (int i=0;i<=2;i++) {
			view[i]=oldRobotView[i];
        	}
	}
}

// prints the current (used) mode on stdout
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
	// getting first the position and view of robot
	getRobotPosAndView(oldRobotPos, oldRobotView, robot);
	// now getting the current angle of the camera
	dsGetViewpoint(oldCamPos,oldCamView);
	if (oldCamType!=camType)
		printMode(camType);
	if (oldRobot!=&robot) // Robot is changed
		// setting current cam values to desired values
		for (int i=0;i<=2;i++) {
			desiredCamPos[i]=oldCamPos[i];
			desiredCamView[i]=oldCamView[i];
		}
	// setting the new Values
	oldCamType=camType;
	oldRobot=&robot;
}

void StaticMode(AbstractRobot& robot) {
	// do nothing
}

void TVMode(AbstractRobot& robot) {
	centerViewOnRobot();
}

void advancedTVMode(AbstractRobot& robot) {
	// check wether the camera is too far away
	robCamDistance= (sqrt(
			SQUARE(currentCamPos[0]-currentRobotPos[0]) +
			SQUARE(currentCamPos[1]-currentRobotPos[1])));
	if (robCamDistance>maxAllowedDistance) {
		// then the camera must get a new position, centering on position of robot
		desiredCamPos[0]=currentRobotPos[0];
		desiredCamPos[1]=currentRobotPos[1];
		desiredCamPos[2]=currentRobotPos[2]+2.0f; // must be higher than robPos...
		//		printf("I am too far away from maximum distance(%f): %f\n",maxAllowedDistance,robCamDistance);
	}
	// now do the rest of TV
	centerViewOnRobot();
}

void FollowingMode(AbstractRobot& robot) {
	followRobotsMove();
}

void print3DimFloat(float vec[3]) {
	for (int i=0;i<=2;i++) {
		printf("\t%f",vec[i]);
	}
}

void moveOnRobot(AbstractRobot& robot) {
	printf("moveOnRobot is now called!\n");
	// now getting the current angle of the camera
	for (int i=0;i<=1;i++) {
		desiredCamPos[i]=currentRobotPos[i];
	}
	desiredCamPos[2]=currentRobotPos[2]+2.0f;
	// center the view on the robot
	centerViewOnRobot();
}

void moveBehindRobot(AbstractRobot& robot) {
	printf("moveBehindRobot is now called!\n");
	robCamDistance=4.0f; // the distance of camera position
	// now get normalizing scalar of currentRobotView, only x and y is needed
	float n=sqrt(SQUARE(currentRobotView[0])+SQUARE(currentRobotView[1]));
	if (n>0) {
		for (int i=0;i<=1;i++)
		  desiredCamPos[i]=currentRobotPos[i]-currentRobotView[i]/n*robCamDistance;
		centerViewOnRobot();
	} else { // we think this is the best
 		moveOnRobot(robot);
	}
}


void advancedFollowingMode(AbstractRobot& robot) {
	float n;
	// first calculate current distance and set the desired position behind the robot
	robCamDistance= (sqrt(
			SQUARE(currentCamPos[0]-currentRobotPos[0]) +
			SQUARE(currentCamPos[1]-currentRobotPos[1])));
	// the direction of the robot is stored in currentRobotView!
	// now get normalizing scalar of currentRobotView, only x and y is needed
	n=sqrt(SQUARE(currentRobotView[0])+SQUARE(currentRobotView[1]));
	if (n>0) { // else no calculation can be made, then do not change the camPos
		for (int i=0;i<=1;i++) {
			// now set the camPos to robotPos-robCamDistance*robotView
			// this is an approximation, normally a rotation is needed
			// store it in newCamPos, so TV mode can be called unmodified
			desiredCamPos[i]=currentRobotPos[i]
			  -currentRobotView[i]*robCamDistance/n+currentRobotView[i];
		}
	}
	// now do centering the view on robot
	centerViewOnRobot();
}


void adjustPosByMouseMove() {
	for (int i=0;i<=2;i++) {
	  desiredCamPos[i]+=currentCamPos[i]-oldCamPos[i];
	  //	  if (oldCamPos[i]-currentCamPos[i]>0) 
	  // desiredCamPos[i]=currentCamPos[i];
	}
}

void adjustViewByMouseMove() {
	for (int i=0;i<=2;i++) {
		desiredCamView[i]-=oldCamView[i]-currentCamView[i];
	}
}

void storeOldValues(float *setCamPos, float *setCamView) {
	for (int i=0;i<=2;i++) {
		oldRobotPos[i]=currentRobotPos[i];
		oldRobotView[i]=currentRobotView[i];
		oldCamPos[i]=setCamPos[i];
		oldCamView[i]=setCamView[i];
	}
}


// wrapper function
void doubleTofloatA(const double *doubleArray,float *floatArray) {
  for (int i=0;i<=2;i++) {
    floatArray[i]=doubleArray[i];
  }
}

// wrapper function
void floatTodoubleA(const float *floatArray,double *doubleArray) {
  for (int i=0;i<=2;i++) {
    doubleArray[i]=floatArray[i];
  }
}


Position smoothMove() {
	double newPos[3];
	for (int i=0;i<=2;i++) {
	  if (ABS(desiredCamPos[i]-currentCamPos[i])<0.1f) {
	    newPos[i]=desiredCamPos[i];
	  } else
	    newPos[i]=desiredCamPos[i]*0.1f+currentCamPos[i]*0.9f;
	}
	return Position(newPos);
}

// normalizes the angle
float normalize360(float angle) {
  float result=angle;
    while (result>=360.0f)
      result-=360.0f;
    while (result<0.0f)
      result+=360.0f;
    return result;
}

// normalizes the angle
double normalize360(double angle) {
  double result=angle;
    while (result>=360.0f)
      result-=360.0f;
    while (result<0.0f)
      result+=360.0f;
    return result;
}

// normalizes the angle of all 3 components
void normalize360vec3(float *angle) {
  for (int i=0;i<=2;i++) {
    angle[i]=normalize360(angle[i]);
  }
}

// normalizes the angle of all 3 components
void normalize360vec3(double *angle) {
  for (int i=0;i<=2;i++) {
    angle[i]=normalize360(angle[i]);
  }
}


Position smoothView() {
	double newView[3];
	normalize360vec3(desiredCamView);
	normalize360vec3(currentCamView);
	for (int i=0;i<=2;i++) {
	  if (ABS(desiredCamView[i]-currentCamView[i])<0.2f) {
	    newView[i]=desiredCamView[i];
	  } else if (ABS(desiredCamView[i]-currentCamView[i])>180.0f) {
	    // we must first transform the angles into linear gradient space (+180 degrees)
	    // then make a backtransformation (-180 degrees)
	    newView[i]=normalize360(desiredCamView[i]+180.0f)*0.1f
	      +normalize360(currentCamView[i]+180.0f)*0.9f-180.0f;
	  normalize360(newView[i]);
	  } else { // use normal sliding
	    newView[i]=desiredCamView[i]*0.1f+currentCamView[i]*0.9f;
	  }
	}
	return Position(newView);
}


void moveCamera( CameraType camType,AbstractRobot& robot) {
	// first look if someone is changed
	// only otherwise change the camera position and/or view.
	if (oldCamType!=camType || &robot!=oldRobot)
		initCamera(camType,robot);
	else {
		// first get all needed values
		// getting first the position and view of robot
		getRobotPosAndView(currentRobotPos,currentRobotView, robot);
		// now getting the current position and angle of the camera
		dsGetViewpoint(currentCamPos,currentCamView);
		// now compute
		// calculate the adjusted new settings from mouse movement
		adjustPosByMouseMove();
		adjustViewByMouseMove();
		// now adjust the desired values through the camera modes
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
		float move[3];
		float view[3];
		doubleTofloatA(smoothMove().toArray(),move);
		doubleTofloatA(smoothView().toArray(),view);
		dsSetViewpoint(move,view);
		// now store the current (set) values to old values
		storeOldValues(move,view);
	}
}

