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
 *   Revision 1.2  2005-09-27 13:59:26  martius
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

/** Class for a bank of ray sensors. 
    Ray sensors can be registered at the bank. Methods for resetting, 
    sensing and reading the sensorvalues of all sensors are provided.
 */
class RaySensorBank {
public:  
  RaySensorBank();

  virtual ~RaySensorBank();

  /** gives the space of the parent (usually robot)
   */
  virtual void init(dSpaceID parent_space, RaySensor::rayDrawMode drawmode); 

  /** registers a new sensor at the sensor bank. The body and the pose have to be provided.
      @param range maximum sense range of the sensor
      @return index of the sensor
   */
  virtual unsigned int registerSensor(RaySensor* raysensor, dBodyID body, 
			     const Position& pos, const dMatrix3 rotation, double range);

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
      @param start element in the sensorarray in which the first raysensor should be stored
      @param array_size maximal number of all elements in the sensorarray
      @return number of written sensorvalues
   */
  virtual int get(double* sensorarray, unsigned int array_size);

  /** returns the spaceID of the sensor space
   */
  virtual dSpaceID getSpaceID();

  /** draws all sensors
   */
  virtual void draw();
  


protected:
  std::vector<RaySensor*> bank;
  dSpaceID sensor_space;
  bool initialized;
  RaySensor::rayDrawMode drawMode;
};

#endif
