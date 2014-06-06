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
#ifndef __AXISORIENTATIONSENSOR_H
#define __AXISORIENTATIONSENSOR_H

#include "sensor.h"

namespace lpzrobots {

  /** Class for sensing the axis orienation of a primitive (robot)
  */
  class AxisOrientationSensor : public Sensor {
  public:
    /// Sensor mode
    enum Mode { /** Z axis (of robot) in word coordinates (relative to body center)
                    (Dimensions select components of this vector) */
                OnlyZAxis,
                ZProjection, ///< z-component of each axis (Dimension select components of this vector)
                Axis ///< for each dimension one orienation vector, i.e. for X | Y | Z it is a 3x3 rotation matrix
    };

    /**
       @param mode how to measure the axis orientation
       @param dimensions bit mask for the dimensions to sense. Default: X | Y | Z (all dimensions)
       @see Sensor::Dimensions
       @see Mode
     */
    AxisOrientationSensor(Mode mode, short dimensions = X | Y | Z );
    virtual ~AxisOrientationSensor() {}

    virtual void init(Primitive* own, Joint* joint = 0);
    virtual int getSensorNumber() const;

    virtual bool sense(const GlobalData& globaldata);
    virtual std::list<sensor> getList() const;
    virtual int get(sensor* sensors, int length) const;

  private:
    Mode mode;
    short dimensions;
    Primitive* own;
  };


}

#endif
