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

#ifndef __SPHEREROBOT_H
#define __SPHEREROBOT_H

#include "oderobot.h"
#include "oneaxisservo.h"

namespace lpzrobots {

  class Primitive;
  class Joint;
  class SliderJoint;

  typedef struct {
  public:
    double diameter;
    double spheremass;
    double pendulardiameter;
    double pendularmass;
    double slidermass;
    double sliderrange;

    double force;      // forcefactor of the servo power (1 is usual)
    double hingeRange; //the angle (in rad) of the hinges that connect pendular with poles
  } SphererobotConf;


  /**
   *This is a class, which models a snake like robot. It consists of a number of equal elements, each linked
   *by a joint. This class is based upon the class roboter by the same author.
   *@author Marcel Kretschmann
   *@version beta
   **/
  class Sphererobot : public OdeRobot
  {
  public:
    /* typedef */ enum objects { Base, Pendular, Pole1Bot, Pole2Bot, Pole3Bot,
                           Pole1Top , Pole2Top, Pole3Top, Last};

  protected:
    const static int sensorno = 9;

    SphererobotConf conf;
    bool created;

  public:
    SliderServo* servo[3];
    Primitive* object[Last];
    SliderJoint* slider[3];
    Joint* joint[6];

  public:

    Sphererobot ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                  const SphererobotConf& conf, const std::string& name );

    virtual ~Sphererobot();

    static SphererobotConf getDefaultConf(){
      SphererobotConf c;
      c.diameter     = 1;
      c.spheremass   = 0.2;
      c.pendulardiameter = 0.2;
      c.pendularmass = 1.0;
      c.slidermass   = 0.005;
      c.sliderrange  = 0.1; // range of the slider from center in multiple of diameter [-range,range]
      c.force        = 1;
      c.hingeRange   = M_PI/180*30;
      return c;
    }

    /// update the subcomponents
    virtual void update();

    /** sets the pose of the vehicle
        @param pose desired 4x4 pose matrix
    */
    virtual void placeIntern(const osg::Matrix& pose);

    /**
     *Writes the sensor values to an array in the memory.
     *@param sensors pointer to the array
     *@param sensornumber length of the sensor array
     *@return number of actually written sensors
     **/
    virtual int getSensorsIntern( double* sensors, int sensornumber );

    /**
     *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
     *It is an linear allocation.
     *@param motors pointer to the array, motor values are scaled to [-1,1]
     *@param motornumber length of the motor array
     **/
    virtual void setMotorsIntern( const double* motors, int motornumber );

    /**
     *Returns the number of motors used by the snake.
     *@return number of motors
     **/
    virtual int getMotorNumberIntern();

    /**
     *Returns the number of sensors used by the robot.
     *@return number of sensors
     **/
    virtual int getSensorNumberIntern();

    /** returns a vector with the positions of all segments of the robot
    */
    virtual int getSegmentsPosition(std::vector<Position> &poslist);

    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return object[Base]; }

  protected:
    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);
    virtual void destroy();



  };

}

#endif
