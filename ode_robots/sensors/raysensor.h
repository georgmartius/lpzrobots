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
#ifndef __RAYSENSOR_H
#define __RAYSENSOR_H

#include <ode-dbl/common.h>
#include "osgforwarddecl.h"
#include "odehandle.h"
#include "osghandle.h"


namespace lpzrobots {
  class Primitive;

/** Abstract class for Ray-based sensors.
    This are sensors which are based on distance measurements using the ODE geom class Ray.
    The sensor value is obtained by collisions.
    However of no collision is detected the sensor needs to ajust its output as well.
    Therefore a reset function is provided.
    See also RaySensorBank, which is an object for managing multiple ray sensors.
 */
class RaySensor {
public:
  enum rayDrawMode { drawNothing, drawRay, drawSensor, drawAll};

  RaySensor() {}
  virtual ~RaySensor(){}

  // should create a copy if this, without initialisation
  virtual RaySensor* clone() const = 0;

  /** providing essential information
      @param odeHandle OdeHandle
      @param osgHandle OsgHandle
      @param body primitive to which the sensor will be attached
      @param pose relative pose in respect to body in which the sensor will be placed
      @param range length of the sensor
      @param drawMode whether to draw nothing, sensor body, ray, or both
   */
  virtual void init(const OdeHandle& odeHandle,
                    const OsgHandle& osgHandle, Primitive* body,
                    const osg::Matrix pose, float range,
                    rayDrawMode drawMode = drawSensor) = 0;

  /** used for reseting the sensor value to a value of maximal distance.
   */
  virtual void reset() = 0;

  /** returns the sensor value (usually in the range [-1,1] )
   */
  virtual double get() = 0;

  /** set the range of the sensor
      @param range new length of the sensor
  */
  virtual void setRange(float range) = 0;

  /** updates the position of the osg nodes
   */
  virtual void update() = 0;

};

}

#endif
