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

namespace lpzrobots {

  class Box;
  class Transform;

  /** Class for a contact sensor. 
      The sensor is on if a collision occurs and stores the penetration depth 
      (as a crude measure for the colission force). It can be either attached to 
      an existing primitive (e.g. a leg) or create its own sensor object. The latter
      is recommended if you want very localized force sensors.
      The information of a collision comes to the sensor via the 
      collision callback of the substance used for the primitive.
      However of no collision is detected the sensor needs to ajust its output as well. 
      Therefore a reset function is provided.
  */
  class ContactSensor {
  public:  

    /**
       @param binary if true then the sensor is 0 or 1 (for contact), no force value returned
       @param forcescale scale of the measured collision force (default: 1)
       @param size size of little box representing the sensor (if it has an own body) (default: 0.05)
    */
    ContactSensor(bool binary=true, double forcescale = 1, double size = 0.05);

    virtual ~ContactSensor();

    virtual void init(const OdeHandle& odeHandle,
		      const OsgHandle& osgHandle, 
		      Primitive* reference, 
                      bool createBox = false,
		      const osg::Matrix pose = osg::Matrix(), 
		      bool colorObject = true);

    virtual void reset();  
 
    /** returns the sensor value in the range >=0;
	0 means nothing no contact
	>0 means contact with another object: size is the force in arbitrary unit
	@see characteritic()
     */
    virtual double get();
    virtual void update();
    
    // set measued depth (used internally) (stores the maximum until next reset)
    virtual void setDepth(float depth);

    
  protected:
    bool   binary;              ///< if contact sensor is a switch
    double forcescale; 
    double value;               ///<  actual sensor value
    double lastvalue;           ///< last value
    double size;                ///< size of graphical sensor

    Primitive* reference;       ///< primitive to which the sensor is bound
    Box* sensorBody;
    Transform* transform;

    bool colorObject;
    Color origColor;
    bool initialised;
  };

}

#endif
