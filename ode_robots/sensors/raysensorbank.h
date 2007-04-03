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
 *   Revision 1.6  2007-04-03 14:12:28  der
 *   getSensorNumber
 *
 *   Revision 1.5  2006/09/20 12:56:28  martius
 *   setRange
 *
 *   Revision 1.4  2006/08/28 12:18:31  martius
 *   documentation
 *
 *   Revision 1.3  2006/07/14 12:23:43  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.2.4.4  2006/03/30 12:34:59  martius
 *   documentation updated
 *
 *   Revision 1.2.4.3  2006/01/12 15:14:57  martius
 *   indentation and clear routine
 *
 *   Revision 1.2.4.2  2005/12/14 12:43:07  martius
 *   moved to osg
 *
 *   Revision 1.2.4.1  2005/12/13 18:11:54  martius
 *   sensors ported, but not yet finished
 *
 *   Revision 1.2  2005/09/27 13:59:26  martius
 *   ir sensors are working now
 *
 *   Revision 1.1  2005/09/27 11:03:34  fhesse
 *   sensorbank added
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __RAYSENSORBANK_H
#define __RAYSENSORBANK_H

#include <vector>
#include "raysensor.h"

namespace lpzrobots {

/** Class for a bank (collection) of ray sensors. 
    Ray sensors can be registered at the bank. Methods for resetting, 
    sensing and reading the sensor values of all sensors are provided.
 */
class RaySensorBank {
public:  
  RaySensorBank();

  virtual ~RaySensorBank();

  /** initialises sensor bank with handles for ode and osg
   */
  virtual void init( const OdeHandle& odeHandle, const OsgHandle& osgHandle ); 

  /** registers a new sensor at the sensor bank. The body and the pose have to be provided.
      @param raysensor RaySensor to add
      @param body body to which the sensor should be connected
      @param pose relative position/orientation
      @param range maximum sense range of the sensor
      @return index of the sensor
   */
  virtual unsigned int registerSensor(RaySensor* raysensor, Primitive* body, 
				      const osg::Matrix& pose, double range,
				      RaySensor::rayDrawMode drawMode);

  /** resets all sensors (used for reseting the sensor value to a value of maximal distance) 
   */
  virtual void reset();  
  
  /** performs sense action of all sensors by checking collision with the given object
      @return true for any collision handled (sensed) and false for no interaction
   */
  virtual bool sense(dGeomID object);

  /** returns the sensor value of the given sensor (usually in the range [-1,1] )
   */
  virtual double get(unsigned int index);

  /** writes sensorvalues in given sensorarray
      @param sensorarray pointer to the sensorarray in which the values should be stored
      @param array_size maximal number of all elements in the sensorarray
      @return number of written sensorvalues
   */
  virtual int get(double* sensorarray, unsigned int array_size);

  /// returns the number of sensors in the bank
  virtual int getSensorNumber();

  /** set the range of the specified sensor (index)
      @param index index of sensor to modify
      @param range new length of the sensor
  */
  virtual void setRange(unsigned int index, double range);

  /** set the range of all sensors 
      @param range new length of the sensors
  */
  virtual void setRange(double range);
  
  /** returns the spaceID of the sensor space
   */
  virtual dSpaceID getSpaceID();

  /** updates the sensor's graphical representation
   */
  virtual void update();
  
  // delete all registered sensors.
  virtual void clear();

protected:
  std::vector<RaySensor*> bank;
  bool initialized;

  OdeHandle odeHandle;
  OsgHandle osgHandle; 
};

}

#endif
