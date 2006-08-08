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
 *   Revision 1.7  2006-08-08 17:03:27  martius
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

  /** Class for IR sensors. 
      IR sensors are based on distance measurements using the ODE geom class Ray. 
      The sensor value is obtained by collisions. 
      However of no collision is detected the sensor needs to ajust its output as well. 
      Therefore a reset function is provided.
  */
  class IRSensor : public RaySensor {
  public:  
    /**
       @param exponent exponent of the sensor characteritic (default: 1 (linear))
    */
    IRSensor(double exponent = 1);

    virtual ~IRSensor();

    virtual void init(const OdeHandle& odeHandle,
		      const OsgHandle& osgHandle, 
		      Primitive* body, 
		      const osg::Matrix pose, double range,
		      rayDrawMode drawMode = drawSensor);

    virtual void reset();  
    virtual bool sense(dGeomID object);
  
    virtual double get();
    virtual void update();
  
    virtual dGeomID getGeomID();

    /// returns the exponent of the sensor characteritic (default: 1 (linear))
    double getExponent () const { return exponent;} 

    /// sets the exponent of the sensor characteritic (default: 1 (linear))
    void   setExponent (double exp) { exponent = exp;}

  protected:
    /** describes the sensor characteritic 
	linear curve used here
    */
    virtual double characteritic(double len);

  protected:
    dGeomID transform;
    dGeomID ray;
    double range; // max length
    double len;   // last measured length
    double value; // actual sensor value
    double exponent; // exponent of the sensor characteritic 

    OSGCylinder* sensorBody;
    OSGBox* sensorRay;
    OsgHandle osgHandle;
  
    bool initialised;
  };

}

#endif
