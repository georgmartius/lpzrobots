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
 *   Revision 1.3.4.3  2005-11-16 11:26:52  martius
 *   moved to selforg
 *
 *   Revision 1.3.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.3.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.3  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.2  2005/07/18 14:47:41  martius
 *   world, space, contactgroup are not pointers anymore.
 *
 *   Revision 1.1  2005/06/27 13:15:58  fhesse
 *   from JointTest, for usage in simulation running_through_limit_cycles
 *
 *   Revision 1.3  2005/06/27 09:31:26  fhesse
 *   few things tested, velocity as sensor- and motorvalues still works fine
 *
 *   Revision 1.2  2005/06/24 13:33:40  fhesse
 *   a lot tested and added
 *
 *   Revision 1.1  2005/06/22 15:37:45  fhesse
 *   sensor and motor values are wheel velocities
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __FIXEDSNAKE2ELEMENTS_H
#define __FIXEDSNAKE2ELEMENTS_H

#include "oderobot.h"

class FixedSnake2Elements : public OdeRobot{

public:
  
  FixedSnake2Elements(const OdeHandle& odeHandle);

  virtual ~FixedSnake2Elements(){};

  /**
   * draws the robot
   */
  virtual void draw();

  /** sets the robot to position pos, sets color to c, and creates robot if necessary
      @params pos desired position of the robot in struct Position
      @param c desired color for the robot in struct Color
  */
  virtual void place(Position pos , Color *c = 0);

  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects and the default routine)
   *  else false (collision is passed to other objects and (if not treated) to the default routine).
   */
  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
   */
  virtual void doInternalStuff(const GlobalData& globalData);

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] 
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber);

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber);

  /** returns number of sensors
   */
  virtual int getSensorNumber(){
    return sensorno;
  };

  /** returns number of motors
   */
  virtual int getMotorNumber(){
    return motorno;
  };

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);

protected:
  virtual Object getMainObject() const {
	if(!segments.empty()){
	  return (*segments.begin());
	}else return Object();
  }

  /** creates robot at desired position 
      @param pos struct Position with desired position
  */
  virtual void create(Position pos); 

  /** destroys robot and space
   */
  virtual void destroy();

  /**
   *vereinfachte Objektpositions-Einzelermittlungsfunktion
   *@author Marcel Kretschmann
   *@version
   **/
  virtual double dBodyGetPositionAll ( dBodyID basis , int para );

  /** fix snake in the sky
   */
  void fixInSky();

  /** use Universal Joints for the snake
   */
  void useUniversalJoints();

  /** use Hinge2 Joints for the snake
   */
  void useHinge2Joints();


  void setMotorsHinge2Velocity(const motor* motors);
  void setMotorsUniversalVelocity(const motor* motors);

  double glieder_masse;
  double glieder_durchmesser;
  double glieder_laenge;

  vector<Object> segments;
  vector<dJointID> joints;


  int sensorno;      // number of sensors
  int motorno;       // number of motors
  int segmentsno;    // number of robot segments
  
  Position initial_pos;    // initial position of robot
  double max_force;        // maximal force for motors

  bool created;      // true if robot was created

  int t;
  bool positiv;

  vector <double> old_sensorvalues;
  vector <double> mean_sensorvalues;

  double gamma;

};


#endif
