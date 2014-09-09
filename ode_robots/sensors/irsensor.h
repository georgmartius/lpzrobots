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

  /** Class for IR sensors.
      IR sensors are based on distance measurements using the ODE geom class Ray.
      The sensor value is obtained by collisions, which are handled by the simulation
      environement. The information of a collision comes to the sensor via the
      collision callback of the substance used for the ray (actually for the transform).
      If no collision is detected the value should be zero, so that a stamp is used.
      The length measured in this way is modified by the 'characteristic' of the IR sensor
  */
  class IRSensor : public RaySensor {
  public:
    /**
       @param exponent exponent of the sensor characteritic (default: 1 (linear))
       @param size size of sensor in simulation
       @param range maximum range of the IR sensor
       @param drawMode draw mode of the sensor
    */
    IRSensor(double exponent = 1, double size = 0.05, double range = 2,
             rayDrawMode drawMode = drawSensor);

    //Override sense to include characteristic
    virtual bool sense(const GlobalData& globaldata) override;

    //Override to return value given by characteristic
    virtual int get(sensor* sensors, int length) const override;
    virtual std::list<sensor> getList() const override;

    //Directly return value (needed for backward compatibility
    virtual double getValue();

    /// returns the exponent of the sensor characteritic (default: 1 (linear))
    virtual double getExponent () const { return exponent;}

    /// sets the exponent of the sensor characteritic (default: 1 (linear))
    virtual void setExponent (double exp) { exponent = exp;}

  protected:
    /** describes the sensor characteritic
        An exponential curve is used.
        @see setExponent()
    */
    virtual double characteritic(double len);

    double exponent; // exponent of the sensor characteritic

    double value; // actual sensor value
  };

}

#endif
