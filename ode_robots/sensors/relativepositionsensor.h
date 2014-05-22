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
#ifndef __RELATIVEPOSITIONSENSOR_H
#define __RELATIVEPOSITIONSENSOR_H

#include "sensor.h"

namespace lpzrobots {

  /** Class for relative (or absolute) position sensing.
      The sensor values are the normalised relative position to some given object ( setReference() ) or the origin.
  */
  class RelativePositionSensor : public Sensor {
  public:
    /**
       @param maxDistance maximal distance that is expected used for normalisation of sensor value
       @param exponent exponent of the sensor characteritic (default: 1 (linear))
       @param dimensions bit mask for the dimensions to sense. Default: X | Y | Z (all dimensions)
       @see Dimensions
       If exact (relative) positions should be produced, use maxDistance=1 and exponent=1
     */
    RelativePositionSensor(double maxDistance, double exponent, short dimensions = X | Y | Z , bool local_coordinates = false);
    virtual ~RelativePositionSensor() {}

    virtual void init(Primitive* own, Joint* joint = 0);
    virtual int getSensorNumber() const;

    virtual bool sense(const GlobalData& globaldata);
    virtual std::list<sensor> getList() const;

    /**
       Sets the reference object we use for relative position measureing.
       If not set or 0 then the origin is used.
       This can be another robot an obstacle (light source) and such like
       This must be called before first sense() or get() call.
    */
    virtual void setReference(Primitive* ref);

  private:
    double maxDistance;
    double exponent;
    short dimensions;
    Primitive* own;
    Primitive* ref;
    bool local_coords;
  };


}

#endif
