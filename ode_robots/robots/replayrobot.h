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
#ifndef __REPLAYROBOT_H
#define __REPLAYROBOT_H


#include "oderobot.h"
#include <selforg/types.h>
#include <selforg/matrix.h>

namespace lpzrobots {

  /**
   *
   */
  class ReplayRobot : public OdeRobot{
  public:
    ReplayRobot(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const char* filename);

    ~ReplayRobot();



    /** sets the pose of the vehicle
        @param pose desired 4x4 pose matrix
    */
    virtual void placeIntern(const osg::Matrix& pose) {}

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
    virtual int getSensorNumberInternIntern() {return sensorEnd - sensorStart + 1; }

    /** returns number of motors
     */
    virtual int getMotorNumberInternIntern() {return motorEnd - motorStart + 1; }

    /** this function is called in each timestep. It should perform robot-internal checks,
        like space-internal collision detection, sensor resets/update etc.
        @param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData) {}

  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return 0; }

    static bool parseDataFileForHeader(FILE* f, int & sensorstart, int& sensorend,  int& motorstart, int& motorend);
    static bool parseDataLine(matrix::Matrix& data, FILE* f);
    static bool isEmpty(const char* c);
    static bool check4Number(const char* c);


  protected:
    int sensorStart;
    int sensorEnd;
    int motorStart;
    int motorEnd;

    matrix::Matrix sensors;
    const char* filename;
    FILE* f;


  };

}

#endif

