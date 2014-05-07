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

#ifndef __FORCESSPHERE_H
#define __FORCESSPHERE_H

#include "oderobot.h"
#include "sensor.h"
#include "motor.h"

namespace lpzrobots {

  class Primitive;

  class ForcedSphereConf {
  public:
    ForcedSphereConf();
    ~ForcedSphereConf();
    /// deletes sensors
    void destroy();

    double radius; //< radius of the sphere
    double maxForce; ///< maximal force applied to the sphere
    /// if true, the robot is powered to reach the given speed (force is calculated)
    bool speedDriven;
    double maxSpeed; ///< maximum speed of the robot when in speedDriven mode

    /// bit mask for selecting the dimensions for the forces (see ForcedSphere::Dimensions)
    short drivenDimensions;
    /// whether to use a cylinder as body (like a puck) or the normal sphere
    bool cylinderBody;
    /// list of sensors that are mounted at the robot. (e.g.\ AxisOrientationSensor)
    std::list<Sensor*> sensors;
    /// adds a sensor to the list of sensors
    void addSensor(Sensor* s) { sensors.push_back(s); }
    /// list of motors that are mounted at the robot. (e.g.\ Speaker)
    std::list<Motor*> motors;
    /// adds a motor to the list of motors
    void addMotor(Motor* m) { motors.push_back(m); }

  };

  class ForcedSphere : public OdeRobot
  {
  protected:
    Primitive* object[1];
    bool created;
    ForcedSphereConf conf;

  public:

    enum Dimensions { X = 1, Y = 2, Z = 4 };

    /**
     * constructor
     *
     * use getDefaultConf() to obtain a configuration with default values, which can be altered
     *  to personal needs.
     **/
    ForcedSphere ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                   const ForcedSphereConf& ForcedSphereConf, const std::string& name);

    virtual ~ForcedSphere();

    static ForcedSphereConf getDefaultConf(){
      ForcedSphereConf c;
      c.radius = 1;
      c.maxForce = 1;
      c.drivenDimensions = X | Y;
      c.cylinderBody = false;
      c.speedDriven=false;
      c.maxSpeed = 5;
      return c;
    }

    virtual void update();

    virtual void placeIntern(const osg::Matrix& pose);

    virtual void doInternalStuff(GlobalData& globalData);

    virtual int getSensorsIntern( sensor* sensors, int sensornumber );
    virtual void setMotorsIntern( const double* motors, int motornumber );
    virtual int getMotorNumberIntern();
    virtual int getSensorNumberIntern();

    virtual Primitive* getMainPrimitive() const { return object[0]; }

  protected:

    virtual void create(const osg::Matrix& pose);
    virtual void destroy();


  };

}

#endif
