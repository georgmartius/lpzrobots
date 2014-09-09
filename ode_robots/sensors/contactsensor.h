/***************************************************************************
 *   Copyright (C) 2005-2012 LpzRobots development team                    *
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
#ifndef __CONTACTSENSOR_H
#define __CONTACTSENSOR_H

#include <ode_robots/color.h>
#include <ode_robots/physicalsensor.h>

namespace lpzrobots {

  class Sphere;
  class Transform;

  /** Class for a contact sensor.
      The sensor is on if a collision occurs and stores the penetration depth
      (as a crude measure for the colission force). It can be either attached to
      an existing primitive (e.g. a leg) or create its own sensor object. The latter
      is recommended if you want very localized force sensors.
      The information of a collision comes to the sensor via the
      collision callback of the substance used for the primitive.
      The sensor returns the maximum forces since the last sense() call.
  */
  class ContactSensor : public PhysicalSensor{
  public:

    /**
       @param binary if true then the sensor is 0 or 1 (for contact), no force value returned
       @param forcescale scale of the measured collision force (default: 1)
       @param size size of little sphere representing the sensor (if it has an own body) (default: 0.05)
       @param createSphere if true then a little sphere is created otherwise the reference body is directly used
       @param colorObject if true then the object (sphere or reference) is colored according to contact state
       @param contactColor color if contact sensor values is 1. In between it is blend with original color.
        If a channel is negative: use original color and invert those channels that are negative.
        (default: (-1,-1,-1) : invert all channels)
    */
    ContactSensor(bool binary=true, double forcescale = 1, double radius = 0.05,
                  bool createSphere = false, bool colorObject = true,
                  Color contactColor = Color(-1,-1,-1));

    virtual ~ContactSensor();

    /** returns the sensor value in the range >=0;
        0 means nothing no contact
        >0 means contact with another object: size is the force in arbitrary unit
        @see characteritic()
     */
    virtual double get();

    // ---- Sensor interface -----
    virtual void init(Primitive* own, Joint* joint = 0) override;

    virtual int getSensorNumber() const { return 1; }

    virtual bool sense(const GlobalData& globaldata) override;

    virtual int get(sensor* sensors, int length) const override;

    virtual std::list<sensor> getList() const override;

    virtual void update() override ;

    // set measued depth (used internally) and the time (old measures are ignored)
    virtual void setDepth(float depth, long int time);

    Transform* getTransformObject();

  protected:
    bool   binary;              ///< if contact sensor is a switch
    double forcescale;
    double detection;           ///<  currently detected value
    double value;               ///<  actual sensor value
    double lastvalue;           ///< last value
    double size;                ///< size of graphical sensor
    long int lasttimeasked;     // used to make sense return the same number if called two times in one timestep

    Primitive* reference;       ///< primitive to which the sensor is bound
    Sphere* sensorBody;
    Transform* transform;

    bool createSphere;
    bool colorObject;
    Color origColor;
    Color touchColor;
    bool initialised;
  };

}

#endif
