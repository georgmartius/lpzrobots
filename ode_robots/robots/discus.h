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

#ifndef __DISCUS_H
#define __DISCUS_H

#include <ode_robots/primitive.h>
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/oderobot.h>
#include <ode_robots/sensor.h>
#include <ode_robots/raysensorbank.h>

namespace lpzrobots {

  /// configuration object for the Discus robot.
typedef struct {
public:
  double diameter;
  double relativewidth;
  double stabdiameter;    ///< diameter of capsule relative to diameter of body
  unsigned int  numAxes;  ///< number of axes with moving masses
  double spheremass;
  double pendulardiameter; ///< automatically set
  double pendularmass;
  double motorpowerfactor; ///< power factor for servos w.r.t. pendularmass
  double pendularrange;    ///< fraction of the diameter the pendular masses can move to one side
  double pendularrangeN;   ///< fraction of the diameter the normal pendular masses can move to one side
  bool motorsensor;        ///< motor values as sensors
  bool irAxis1;
  bool irAxis2;
  bool irAxis3;
  bool irRing;            ///< IR sensors in a ring in x,z plane (collides with irAxis1 and irAxis3)
  bool irSide;            ///< 4 IR senors to both side in y direction (collides with irAxis2)
  bool drawIRs;
  double irsensorscale; ///< range of the ir sensors in units of diameter
  double irCharacter;   ///< characteristics of sensor (\f[ x^c \f] where x is the range-distance)
  RaySensor* irSensorTempl;  ///< template for creation of the other ir sensors (if 0 then IRSensor(irCharacter))
  double motor_ir_before_sensors; ///< if true motor sensors and ir sensors are given before additional sensors
  double brake;         ///< if nonzero the robot brakes (deaccelerates actively/magically) (velocity dependend torque)

  /// function that deletes sensors
  void destroy();
  /// list of sensors that are mounted at the robot. (e.g.\ AxisOrientationSensor)
  std::list<Sensor*> sensors;
  /// adds a sensor to the list of sensors
  void addSensor(Sensor* s) { sensors.push_back(s); }
} DiscusConf;

/**
   A spherical robot with 3 internal masses, which can slide on their orthogonal axes.
   This robot was inspired by Julius Popp (http://sphericalrobots.com)
*/
class Discus : public OdeRobot
{
public:
  /// enum for the objects of the robot
  /* typedef */ enum objects { Base, Stabilizer, Pendular1, Pendular2, Pendular3, Last } ;

protected:
  static const int maxservono=3;

  Primitive* object[Last];
  SliderServo* servo[maxservono];
  SliderJoint* joint[maxservono];
  OSGPrimitive* axis[maxservono];

  DiscusConf conf;
  RaySensorBank irSensorBank; ///< a collection of ir sensors
  double transparency;
  bool created;

public:

  /**
   *constructor
   **/
  Discus ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       const DiscusConf& conf, const std::string& name, double transparency=0.5 );

protected:
  /**
   *constructor for children
   **/
  Discus ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       const DiscusConf& conf,
                       const std::string& name, const std::string& revision, double transparency);
  /// initialises some internal variables
  void init();
public:
  virtual ~Discus();


  /// default configuration
  static DiscusConf getDefaultConf(){
    DiscusConf c;
    c.diameter     = 1;
    c.relativewidth= 0.2;
    c.stabdiameter=0.7;
    c.numAxes = 3;
    c.spheremass   = 1;// 0.1
    c.pendularmass  = 0.75;
    c.pendularrange  = 0.20; // range of the slider from center in multiple of diameter [-range,range]
    c.pendularrangeN = 0.25;
    c.motorpowerfactor  = 100;
    c.motorsensor = true;
    c.irAxis1=false;
    c.irAxis2=false;
    c.irAxis3=false;
    c.irRing=false;
    c.irSide=false;
    c.drawIRs=true;
    c.irsensorscale=1.5;
    c.irCharacter=1;
    c.irSensorTempl=0;
    c.motor_ir_before_sensors=false;
    c.brake=0;
   return c;
  }

  virtual void update();

  virtual void placeIntern(const osg::Matrix& pose);

  virtual void doInternalStuff(GlobalData& globalData);

  virtual void sense(GlobalData& globalData) override;

  virtual int getSensorsIntern( sensor* sensors, int sensornumber );

  virtual void setMotorsIntern( const double* motors, int motornumber );

  virtual int getMotorNumberIntern();

  virtual int getSensorNumberIntern();

  virtual Primitive* getMainPrimitive() const { return object[Base]; }

protected:

  virtual void create(const osg::Matrix& pose);
  virtual void destroy();


};

}

#endif
