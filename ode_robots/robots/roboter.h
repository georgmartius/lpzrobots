/********************************************************************************/
/*roboter.h									*/
/*Basic robot-Class, designed for the robot system, designed by the author	*/
/*@author Marcel Kretschmann							*/
/*@version beta									*/
/*										*/
/********************************************************************************/
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
 *   Revision 1.11  2005-11-09 13:24:42  martius
 *   added GPL
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __ROBOTER_H
#define __ROBOTER_H

#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <vector>

using namespace std;

#include "abstractrobot.h"

/******************************************type declaration*****************************************/
typedef struct
{
	double x;
	double y;
	double z;
} raumvektor;

typedef struct
{
   	double istwinkel;
	double istwinkel_alt;
	double sollwinkel;

        double x;
        double y;
        double z;
} Sensor;

enum ausgabemodus { angle , anglerate };

/****************************tool functions********************************/

/**
 *This funktion enables the posibility to directly access the values of a body position.
 *@param basis ODE internal robot-ID
 *@param para 0 = x, 1 = y, 2 = z
 *@author Marcel Kretschmann
 *@version final
 **/
double dBodyGetPositionAll ( dBodyID basis , int para );

/***************************************************************************/

/**
 *This is the universal base class, which provides the basic funktions, each robot should have.
 *But there are the restrictions, the abstract class AbstractRobot defines, which is used as an interface here.
 *@author Marcel Kretschmann
 *@version beta
 **/
class Roboter : public AbstractRobot
{

public:
	vector <Sensor> sensorfeld;

protected:
	//Eigenschaften
	int roboterID;
	
	vector <Object> objektliste;
	vector <dJointID> jointliste;
	vector <dJointID> motorliste;
	
public:

	/**
	 *constructor
	 *@param startRoboterID ID, which should be managed clearly
	 *@param welt pointer to the ODE-simulation world, which contains the whole simulation
	 *@param raum pointer to the ODE-simulation space, which contains all geometrical objects
	 *@param start_contactgroup pointer to the JointGroup, which is used for collision management
	 *@param start_Sensorzahl number of sensors of the robot
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	Roboter ( int startRoboterID , const OdeHandle& odeHandle , int start_Sensoranzahl );
	
	/**
	 *destructor
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual ~Roboter();
	/**
	 *Returns the ID of the robot
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	 virtual int getRoboterID ();
	
	/**
	 *draws all geometrical objects
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual void draw();
	
	/**sets the robot to position pos, sets color to c, and creates robot if necessary
	 *Only sets the color, there is no special place operation for this unspecified robot.
	 *@param pos new position of the robot
	 *@param c desired color for the robot in struct Color
 	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual void place(Position pos, Color *c);
	
	/**
	 *This is the universal collision handling function of all robots. Each robot handles ist own collisions. There is also the posibility that the robot cancels the collision handling, but then it also returns the same value, as if it has handled the collision. So it is possible that there are special parts of the Robot, which could act whithout being influenced by other parts or geometrical objects of the simulation environment.
	 *@param data
	 *@param o1 first geometrical object, which has taken part in the collision
	 *@param o2 second geometrical object, which has taken part in the collision
	 *@return true if the collision was threated  by the robot, false if not
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
   */
  virtual void doInternalStuff(const GlobalData& globalData);
	
	/**
	 *Writes the sensor values to an array in the memory.
	 *@param sensor* pointer to the array
	 *@param sensornumber length of the sensor array
	 *@return number of actually written sensors
 	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual int getSensors(sensor* sensors, int sensornumber);
	
	/**
	 *Reads the actual motor commands from an array, an sets all motors of the robot to this values.
	 *It is an linear allocation.
	 *@param motors pointer to the array, motor values are scaled to [-1,1] 
	 *@param motornumber length of the motor array
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual void setMotors ( const motor* motors, int motornumber );
	
	
	/**
	 *Returns the number of sensors used by the robot.
	 *@return number of sensors
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual int getSensorNumber();
	
	/**
	 *Returns the number of motors used by the robot.
	 *@return number of motors
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual int getMotorNumber();
	
	
	/**
	 *Returns the position of the robot.
	 *@return Position (x,y,z)
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual Position getPosition ( ) = 0;
	
	
	/**
	 *Returns a list with the positionvectors of all segments of the robot
	 *@param poslist list with positionvectors (of all robot segments) (free after use!)
	 *@return length of the list
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual int getSegmentsPosition ( vector<Position> &poslist );
	
	/**
	 *Returns the number of Objects, the robot consists of
	 *@return int number of Objects
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual int getObjektAnzahl ();
	
	/**
	 *Returns the number of joints, linking the robot parts
	 *@return int number of joints
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual int getJointAnzahl ();
	
	/**
	 *Returns the number of motor-joints, linking the robot parts
	 *@return int number of motor-joints
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual int getMotorAnzahl ();
	
	
	/**
	 *Returns a special Object from the list of all Objects, the robot consits off
	 *@param n The position of the Object in the list ob all Objects
	 *@return Object
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual Object getObjektAt ( int n );
	
	/**
	 *Returns one special joint from the list of all joints, which link the robots parts
	 *@param n The position of the joint in the list ob all joints belonging to the robot
	 *@return dJointID
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual dJointID getJointAt ( int n );
	
	/**
	 *Returns one special motor joint from the list of all motors
	 *@param n The position of the motor in the list ob all motors belonging to the robot
	 *@return dJointID
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual dJointID getMotorAt ( int n );
	
	
	/**
	 *Calculates the difference off the actual angle and the last calculated angle of a sensor..
	 *@param motor number of the motor in the motor-list
	 *@param X pointer to a variable, to save the calculated difference
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual void getWinkelDifferenz ( int motor , double* X );
	
	/**
	 *Adds an empy sensor to the sensorlist.
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual void addSensor ();
	
	/**
	 *Removes the last sensor of the sensorlist
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual void delSensor ();
	
	/**
	 *Returns the number of active sensors.
	 *@return number of sensors
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	virtual int getSensorfeldGroesse ();
	
	/**
	 *Updates the sensorarray.
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual void sensoraktualisierung ();
	
	/**
	 *Decides if some collisions of the robot should not be threated by by the collision management.
	 *@param o1 Geometrieobjekt 1, dass an der Kollision beteiligt ist
	 *@param o2 Geometrieobjekt 2, dass an der Kollision beteiligt ist
	 *@return true, if the collision should not be threated, false else
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual bool kollisionsermittlung ( dGeomID o1 , dGeomID o2 );
	
	/**
	 *Prints some internal robot parameters. This only works in some subclasses.
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	virtual void getStatus ();
	
};

#endif
