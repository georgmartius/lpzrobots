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
 *   Revision 1.3.4.3  2005-11-16 11:26:53  martius
 *   moved to selforg
 *
 *   Revision 1.3.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.3.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.3  2005/09/22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.2  2005/07/18 14:47:41  martius
 *   world, space, contactgroup are not pointers anymore.
 *
 *   Revision 1.1  2005/07/06 16:03:37  martius
 *   dummy robot that connects motors with sensors
 *
 ***************************************************************************/
#ifndef __SHORTCIRCUIT_H
#define __SHORTCIRCUIT_H

#include <ode/common.h>

#include <vector>
using namespace std;
 

/**
 * 
 */
class ShortCircuit : public OdeRobot{
public:
  /**
   * Constructor
   * @param w world in which robot should be created
   * @param s space in which robot should be created
   * @param c contactgroup for collision treatment
   */
  ShortCircuit(const OdeHandle& odeHandle,int sensornumber, int motornumber);

  /// draws the robot
  virtual void draw();

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
  virtual void place(Position pos , Color *c = 0) {}

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
  virtual int getSensorNumber() {return sensorno; }

  /** returns number of motors
  */
  virtual int getMotorNumber() {return motorno; }

  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
  */
  virtual void doInternalStuff(const GlobalData& globalData) {}


  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);

protected:
  virtual Object getMainObject() const  { return Object(); }

 protected:
  int sensorno;      //number of sensors
  int motorno;       // number of motors
  motor* motors;
} ;

#endif
 
