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

#ifndef __SPHEREROBOT3MASSES_H
#define __SPHEREROBOT3MASSES_H

#include "primitive.h"
#include "joint.h"
#include "oneaxisservo.h"
#include "oderobot.h"
#include "sensor.h"
#include "raysensorbank.h"

namespace lpzrobots {

  /// configuration object for the Sphererobot3Masses robot.
typedef struct {
public:
  double diameter;
  double spheremass;
  double pendulardiameter; ///< automatically set
  double pendularmass;
  double motorpowerfactor; ///< power factor for servos w.r.t. pendularmass
  double pendularrange;    ///< fraction of the diameter the pendular masses can move to one side
  bool motorsensor;        ///< motor values as sensors
  bool irAxis1;
  bool irAxis2;
  bool irAxis3;
  bool irRing;            ///< IR sensors in a ring in x,z plane (collides with irAxis1 and irAxis3)
  bool irSide;            ///< 4 IR senors to both side in y direction (collides with irAxis2)
  RaySensor::rayDrawMode drawIRs;
  double irsensorscale; ///< range of the ir sensors in units of diameter
  double irCharacter;   ///< characteristics of sensor (\f[ x^c \f] where x is the range-distance)
  RaySensor* irSensorTempl;  ///< template for creation of the other ir sensors (if 0 then IRSensor(irCharacter))
  double motor_ir_before_sensors; ///< if true motor sensors and ir sensors are given before additional sensors
  double brake;         ///< if nonzero the robot brakes (deaccelerates actively/magically)
  double axesShift; ///< defines how much the axes are shifted from the center

  /// function that deletes sensors
  void destroy();
  /// list of sensors that are mounted at the robot. (e.g.\ AxisOrientationSensor)
  std::list<Sensor*> sensors;
  /// adds a sensor to the list of sensors
  void addSensor(Sensor* s) { sensors.push_back(s); }
} Sphererobot3MassesConf;

/**
   A spherical robot with 3 internal masses, which can slide on their orthogonal axes.
   This robot was inspired by Julius Popp (http://sphericalrobots.com)
*/
class Sphererobot3Masses : public OdeRobot
{
public:
  /// enum for the objects of the robot
  enum parts { Base, Pendular1, Pendular2, Pendular3, Last } ;

protected:
  static const int servono=3;
  unsigned int numberaxis;

  SliderServo* servo[servono];
  OSGPrimitive* axis[servono];

  Sphererobot3MassesConf conf;
  RaySensorBank irSensorBank; ///< a collection of ir sensors
  double transparency;
  bool created;

public:

  /**
   *constructor
   **/
  Sphererobot3Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       const Sphererobot3MassesConf& conf, const std::string& name, double transparency=0.5 );

protected:
  /**
   *constructor for children
   **/
  Sphererobot3Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       const Sphererobot3MassesConf& conf,
                       const std::string& name, const std::string& revision, double transparency);
  /// initialises some internal variables
  void init();
public:
  virtual ~Sphererobot3Masses();


  /// default configuration
  static Sphererobot3MassesConf getDefaultConf(){
    Sphererobot3MassesConf c;
    c.diameter     = 1;
    c.spheremass   = .3;// 0.1
    c.pendularmass  = 1.0;
    c.pendularrange  = 0.20; // range of the slider from center in multiple of diameter [-range,range]
    c.motorpowerfactor  = 100;
    c.motorsensor = true;
    c.irAxis1=false;
    c.irAxis2=false;
    c.irAxis3=false;
    c.irRing=false;
    c.irSide=false;
    c.drawIRs=RaySensor::drawAll;
    c.irsensorscale=1.5;
    c.irCharacter=1;
    c.irSensorTempl=0;
    c.motor_ir_before_sensors=false;
    c.axesShift=0;
    c.brake=0;
    c.axesShift=0;
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

  /******** CONFIGURABLE ***********/
  virtual void notifyOnChange(const paramkey& key);


protected:

  virtual void create(const osg::Matrix& pose);
  virtual void destroy();


};

}

#endif
