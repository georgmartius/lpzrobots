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
 *                                                                 *
 ***************************************************************************
 *                                                                         *
 * Spherical Robot inspired by Julius Popp.                                *
 *                                                                         *
 *   $Log$
 *   Revision 1.10.4.2  2005-11-15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.10.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.10  2005/11/09 13:27:07  martius
 *   irsensorrange
 *
 *   Revision 1.9  2005/11/07 17:04:56  martius
 *   irsensorscale added
 *
 *   Revision 1.8  2005/11/04 14:45:18  martius
 *   GPL added
 *
 *                                                                 *
 ***************************************************************************/

#ifndef __SPHEREROBOTARMS_H
#define __SPHEREROBOTARMS_H


using namespace std;

#include "sliderservo.h"
#include "oderobot.h"
#include "raysensorbank.h"

typedef struct {
public:
  double diameter;
  double spheremass;
  double pendulardiameter; /// automatically set
  double pendularmass;
  double pendularrange;
  bool axisZsensor;  
  bool axisXYZsensor;  
  bool motorsensor;  
  bool irAxis1;
  bool irAxis2;
  bool irAxis3;
  bool drawIRs;
  double irsensorscale; /// range of the ir sensors in units of diameter
} SphererobotArmsConf;


/**
 *This is a class, which models a snake like robot. It consists of a number of equal elements, each linked 
 *by a joint. This class is based upon the class roboter by the same author.
 *@author Marcel Kretschmann
 *@version beta
 **/
class SphererobotArms : public OdeRobot
{
public:
  typedef enum objects { Base, Pendular1, Pendular2, Pendular3, Last } ;

private:
  static const int servono=3;

  dSpaceID sphererobot_space;
  SliderServo* servo[servono];
  char* name;
  int texture;
  double transparency;

protected:
  SphererobotArmsConf conf;
  RaySensorBank irSensorBank; // a collection of ir sensors

public:
  Object object[Last];

  /**
   *constructor
   **/ 
  SphererobotArms ( const OdeHandle& odeHandle, 
		    const SphererobotArmsConf& conf, double transparency=0.5 );
  
  virtual ~SphererobotArms();
	
  static SphererobotArmsConf getStandartConf(){
    SphererobotArmsConf c;
    c.diameter     = 1;
    c.spheremass   = 0.1;
    c.pendularmass  = 1.0;
    c.pendularrange  = 0.25; // range of the slider from center in multiple of diameter [-range,range]
    c.axisZsensor = true;
    c.axisXYZsensor = false;  
    c.motorsensor = false;  
    c.irAxis1=false;
    c.irAxis2=false;
    c.irAxis3=false;
    c.drawIRs=true;
    c.irsensorscale=2;
    return c;
  }

  void setTexture(int tex) { texture = tex; }

  /**
   *Draws the geometrical objects of the robot.
   **/
  virtual void draw();
	
  /**Sets the sphere to position pos, sets color to c, and creates sphere if necessary.
   *This overwrides the function place of the class robot.
   *@param pos desired position of the snake in struct Position
   *@param c desired color for the snake in struct Color
   **/
  virtual void place (Position pos, Color *c = 0);
  
  /**
   *This is the collision handling function for snake robots.
   *This overwrides the function collisionCallback of the class robot.
   *@param data
   *@param o1 first geometrical object, which has taken part in the collision
   *@param o2 second geometrical object, which has taken part in the collision
   *@return true if the collision was threated  by the robot, false if not
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
   **/
  virtual int getSensors ( sensor* sensors, int sensornumber );
	
  /**
   *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
   *It is an linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1] 
   *@param motornumber length of the motor array
   **/
  virtual void setMotors ( const motor* motors, int motornumber );
	
  /**
   *Returns the number of motors used by the snake.
   *@return number of motors
   **/
  virtual int getMotorNumber();
  
  /**
   *Returns the number of sensors used by the robot.
   *@return number of sensors
   **/
  virtual int getSensorNumber();
	
 
  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);	

protected:
  virtual Object getMainObject() { return object[Base]; }

};

#endif
