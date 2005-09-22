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
 *   Revision 1.6  2005-09-22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.5  2005/07/18 14:47:41  martius
 *   world, space, contactgroup are not pointers anymore.
 *
 *   Revision 1.4  2005/06/28 10:12:15  fhesse
 *   friction_factor gamma and tanh in getSensors added
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
#ifndef __JOINTTEST_H
#define __JOINTTEST_H

#include "abstractrobot.h"

class JointTest : public AbstractRobot{

public:
  
  JointTest(const OdeHandle& odeHandle);

  virtual ~JointTest(){};

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

  /** returns position of robot 
      @return position robot position in struct Position  
  */
  virtual Position getPosition();

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);

protected:

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
