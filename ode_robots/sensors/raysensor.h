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
 *   Revision 1.3.4.2  2005-12-13 18:11:53  martius
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
 */
class RaySensor {
public:  
  enum rayDrawMode { drawNothing, drawRay, drawSensor, drawAll};

  RaySensor() {}
  
  /** providing essential information
   */
  virtual void init(const OdeHandle& odeHandle,
		    const OsgHandle& osgHandle, Primitive* body, 
		    const osg::Matrix pose, double range,
		    rayDrawMode drawMode = drawSensor) = 0;  

  /** used for reseting the sensor value to a value of maximal distance. 
   */
  virtual void reset() = 0;  
  
  /** performs sense action by checking collision with the given object
      @return true for collision handled (sensed) and false for no interaction
   */
  virtual bool sense(dGeomID object) = 0;

  /** returns the sensor value (usually in the range [-1,1] )
   */
  virtual double get() = 0;

  /** updates the position of the osg nodes 
   */
  virtual void update() = 0;
  
  /** returns the geomID of the ray geom (used for optimisation)
   */
  virtual dGeomID getGeomID() =0;

};

}

#endif
