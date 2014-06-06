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
#ifndef __IRSENSOR_H
#define __IRSENSOR_H

#include "raysensor.h"

namespace lpzrobots {

  class OSGCylinder;
  class OSGBox;
  class Transform;
  class Ray;


  /** Class for IR sensors.
      IR sensors are based on distance measurements using the ODE geom class Ray.
      The sensor value is obtained by collisions, which are handled by the simulation
      environement. The information of a collision comes to the sensor via the
      collision callback of the substance used for the ray (actually for the transform).
      If no collision is detected the value should be zero, so that a stamp is used.
  */
  class IRSensor : public RaySensor {
  public:
    /**
       @param exponent exponent of the sensor characteritic (default: 1 (linear))
    */
    IRSensor(float exponent = 1, double size = 0.05, float range = 2, rayDrawMode drawMode = drawSensor);

    virtual ~IRSensor();

    // ---- Sensor interface -----
    virtual void init(Primitive* own, Joint* joint = 0) override;

    virtual int getSensorNumber() const { return 1; }

    virtual bool sense(const GlobalData& globaldata) override;

    virtual int get(sensor* sensors, int length) const override;
    virtual std::list<sensor> getList() const override;

    virtual void update() override;

    // ---- Physical Sensor interface ----
    virtual void setPose(const osg::Matrix& pose) override;

    virtual osg::Matrix getPose() override;

    // --------

    double getValue() const;

    virtual void setRange(float range) override;

    virtual void setDrawMode(rayDrawMode drawMode) override;

    /// called by callback
    virtual void setLength(float len, long int time);

    virtual RaySensor* clone() const;

    /// returns the exponent of the sensor characteritic (default: 1 (linear))
    double getExponent () const { return exponent;}

    /// sets the exponent of the sensor characteritic (default: 1 (linear))
    void   setExponent (float exp) { exponent = exp;}

  protected:
    /** describes the sensor characteritic
        An exponential curve is used.
        @see setExponent()
    */
    virtual float characteritic(float len);

  protected:
    float exponent; // exponent of the sensor characteritic
    double size; // size of graphical sensor
    float range; // max length
    rayDrawMode drawMode;

    float len;   // last measured length
    long  time;  // time of last measurement
    float value; // actual sensor value
    float lastvalue; // last value



    OSGCylinder* sensorBody;
    //    OSGBox* sensorRay;

    Transform* transform;
    Ray* ray;
    bool initialised;
  };

}

#endif
