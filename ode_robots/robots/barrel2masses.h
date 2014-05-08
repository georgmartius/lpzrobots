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

#ifndef __BARREL2MASSES_H
#define __BARREL2MASSES_H

#include "primitive.h"
#include "joint.h"
#include "oneaxisservo.h"
#include "oderobot.h"
#include "raysensorbank.h"
#include "sphererobot3masses.h"

namespace lpzrobots {

/*
  parameters for nice rolling modes:

    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
    conf.pendularrange  = 0.3;
    conf.motorsensor=false;
    conf.axisZsensor=true;
    conf.axisXYZsensor=false;
    conf.spheremass   = 1;
    sphere1 = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)),
                                  conf, "Barrel1", 0.2);
    sphere1->placeIntern( osg::Matrix::rotate(M_PI/2, 1,0,0));

    controller = new InvertMotorNStep();
    controller->setParam("steps", 2);
    controller->setParam("adaptrate", 0.0);
    controller->setParam("epsC", 0.03);
    controller->setParam("epsA", 0.05);
    controller->setParam("rootE", 3);
    controller->setParam("logaE", 0);

    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );

*/

/**
   A barrel-like robot with 2 internal masses, which can slide on their orthogonal axes.
   It is the small brother of the Sphererobot3Masses.
   This robot was inspired by Julius Popp (http://sphericalrobots.com)
*/
class Barrel2Masses : public Sphererobot3Masses
{
public:

  /**
   * Constructor. It is configured with the configuration object of Sphererobot3Masses.
   Just two of the 3 axis are used. The worldZaxissensor  and irAxis3 has no meaning here.
   **/
  Barrel2Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       const Sphererobot3MassesConf& conf, const std::string& name, double transparency=0.5 );

  virtual ~Barrel2Masses();

  /** default configuration. It has no sensors.
      Use addSensor(std::make_shared<Sensor>(AxisOrientationSensor(ZProjectionXY)) for example.*/
  static Sphererobot3MassesConf getDefaultConf(){
    Sphererobot3MassesConf c = Sphererobot3Masses::getDefaultConf();
    c.diameter     = 1;
    c.spheremass   = .3;// 0.1
    c.pendularmass  = 1.0;
    c.pendularrange  = 0.25; // range of the slider from center in multiple of diameter [-range,range]
    c.motorpowerfactor = 150;
    c.motorsensor = false;
    c.irRing=true;
    c.irSide=false;
    c.irAxis1=false;
    c.irAxis2=false;
    c.irAxis3=false;
    c.drawIRs=RaySensor::drawAll;
    c.irsensorscale=1.5;
    c.irCharacter=1;
    c.irSensorTempl=0;
    c.motor_ir_before_sensors=false;
    c.brake=0;
    c.axesShift=0;
    return c;
  }

  virtual int getSensorsIntern( sensor* sensors, int sensornumber );

protected:

  /// The cylinder (main body) lies on the ground, that it is rotating about the z-axis
  virtual void create(const osg::Matrix& pose);

};

}

#endif
