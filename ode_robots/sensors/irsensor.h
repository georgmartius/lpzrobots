/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Rald Der       <ralfder at mis dot mpg dot de>                       *
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
      However of no collision is detected the sensor needs to ajust its output as well. 
      Therefore a reset function is provided.
  */
  class IRSensor : public RaySensor {
  public:  
    /**
       @param exponent exponent of the sensor characteritic (default: 1 (linear))
    */
    IRSensor(float exponent = 1, double size = 0.05);

    virtual ~IRSensor();

    virtual void init(const OdeHandle& odeHandle,
		      const OsgHandle& osgHandle, 
		      Primitive* body, 
		      const osg::Matrix pose, float range,
		      rayDrawMode drawMode = drawSensor);

    virtual void reset();  
 
    /** returns the sensor value in the range [0,1];
	0 means nothing no object in the sensor distance range
	1 means contact with another object
	@see characteritic()
     */
    virtual double get();
    virtual void update();
  
    virtual void setRange(float range);

    virtual void setLength(float len);

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
    float range; // max length
    float len;   // last measured length
    float value; // actual sensor value
    float lastvalue; // last value
    float exponent; // exponent of the sensor characteritic 

    double size; // size of graphical sensor

    OSGCylinder* sensorBody;
    //    OSGBox* sensorRay;
    OsgHandle osgHandle;

    Transform* transform;
    Ray* ray;
    bool initialised;
  };

}

#endif
