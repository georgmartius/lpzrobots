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
 *   Revision 1.15  2010-09-17 10:08:42  martius
 *   ir sensors did not work properly because of bug in osgprimitves. Resolved now
 *   Soundsensor: some comments added (really unfinished stuff)
 *
 *   Revision 1.14  2009/01/20 17:29:52  martius
 *   cvs commit
 *
 *   Revision 1.13  2007/09/06 18:48:29  martius
 *   clone function (a bit like a factory)
 *
 *   Revision 1.12  2007/08/24 11:57:48  martius
 *   some forward declaration
 *
 *   Revision 1.11  2007/08/23 15:39:05  martius
 *   new IR sensor schema which uses substances and callbacks, very nice
 *
 *   Revision 1.10  2006/09/20 12:56:28  martius
 *   setRange
 *
 *   Revision 1.9  2006/09/11 12:01:31  martius
 *   *** empty log message ***
 *
 *   Revision 1.8  2006/08/28 12:18:31  martius
 *   documentation
 *
 *   Revision 1.7  2006/08/08 17:03:27  martius
 *   new sensors model
 *
 *   Revision 1.6  2006/07/14 12:23:43  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.5.4.4  2006/03/30 12:34:59  martius
 *   documentation updated
 *
 *   Revision 1.5.4.3  2006/01/12 15:14:02  martius
 *   some fwd decl.
 *
 *   Revision 1.5.4.2  2005/12/14 12:43:07  martius
 *   moved to osg
 *
 *   Revision 1.5.4.1  2005/12/13 18:11:53  martius
 *   sensors ported, but not yet finished
 *
 *   Revision 1.5  2005/11/09 13:24:20  martius
 *   added exponent
 *
 *   Revision 1.4  2005/11/09 09:13:47  fhesse
 *   geom is only enabled in sense function
 *   there is no external collision detection anymore
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
