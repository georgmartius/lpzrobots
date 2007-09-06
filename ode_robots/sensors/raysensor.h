/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.9  2007-09-06 18:48:29  martius
 *   clone function (a bit like a factory)
 *
 *   Revision 1.8  2007/08/23 15:39:05  martius
 *   new IR sensor schema which uses substances and callbacks, very nice
 *
 *   Revision 1.7  2006/09/20 12:56:28  martius
 *   setRange
 *
 *   Revision 1.6  2006/08/28 12:18:31  martius
 *   documentation
 *
 *   Revision 1.5  2006/08/08 17:03:27  martius
 *   new sensors model
 *
 *   Revision 1.4  2006/07/14 12:23:43  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.3.4.3  2006/01/31 15:46:16  martius
 *   virtual destructor
 *
 *   Revision 1.3.4.2  2005/12/13 18:11:53  martius
 *   sensors ported, but not yet finished
 *
 *   Revision 1.3.4.1  2005/11/14 17:37:21  martius
 *   moved to selforg
 *
 *   Revision 1.3  2005/09/27 13:59:26  martius
 *   ir sensors are working now
 *
 *   Revision 1.2  2005/09/27 11:03:34  fhesse
 *   sensorbank added
 *
 *   Revision 1.1  2005/09/22 12:56:47  martius
 *   ray based sensors
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __RAYSENSOR_H
#define __RAYSENSOR_H

#include <ode/common.h>
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
