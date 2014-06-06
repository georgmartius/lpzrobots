/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
#ifndef __HURLINGSNAKE_H
#define __HURLINGSNAKE_H

#include "oderobot.h"
#include <selforg/configurable.h>
#include "primitive.h"
#include "joint.h"

namespace lpzrobots {

  /**
   * Hurling snake is a string a beats.
   *
   */
  class HurlingSnake : public OdeRobot{
  public:
    /**
     * Constructor
     */
    HurlingSnake(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const std::string& name);

    ~HurlingSnake();

    /** sets the pose of the vehicle
        @param pose desired 4x4 pose matrix
    */
    virtual void placeIntern(const osg::Matrix& pose);

    /** this function is called in each timestep. It should perform robot-internal checks,
        like sensor resets/update etc.
        @param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData);


    /** returns actual sensorvalues
        @param sensors sensors scaled to [-1,1]
        @param sensornumber length of the sensor array
        @return number of actually written sensors
    */
    virtual int getSensorsIntern(sensor* sensors, int sensornumber);

    /** sets actual motorcommands
        @param motors motors scaled to [-1,1]
        @param motornumber length of the motor array
    */
    virtual void setMotorsIntern(const double* motors, int motornumber);

    /** returns number of sensors
     */
    virtual int getSensorNumberIntern();

    /** returns number of motors
     */
    virtual int getMotorNumberIntern();

    /** returns a vector with the positions of all segments of the robot
        @param poslist vector of positions (of all robot segments)
        @return length of the list
    */
    virtual int getSegmentsPosition(std::vector<Position> &poslist);

    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);

  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return objects[(NUM-1)/2] /*(center)*/; }
    //virtual Primitive* getMainPrimitive() const { return object[NUM-1] /*(head element)*/; }


  private:

    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys robot and space
     */
    virtual void destroy();

    bool created;      // true if robot was created


    Position initial_pos;    // initial position of robot

    int NUM;           /* number of beats */
    double MASS;           /* mass of a beats */
    double RADIUS;   /* sphere radius */

    //    std::list<AngularMotor*> frictionMotors;

    Pos oldp;

    int sensorno;
    int motorno;

    paramval factorForce;
    paramval frictionGround;
    paramval frictionRoll;
    paramval factorSensor;
    parambool placedummy;

  };

}

#endif

