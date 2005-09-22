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
 *   Revision 1.1  2005-09-22 12:56:47  martius
 *   ray based sensors
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __RAYSENSOR_H
#define __RAYSENSOR_H

#include <ode/common.h>

/** Abstract class for Ray-based sensors. 
    This are sensors which are based on distance measurements using the ODE geom class Ray. 
    The sensor value is obtained by collisions. 
    However of no collision is detected the sensor needs to ajust its output as well. 
    Therefore a reset function is provided.
 */
class RaySensor {
public:  
  RaySensor() {}
  
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

  /** draws the sensor ray
   */
  virtual void draw() = 0;
  
  /** returns the geomID of the ray geom (used for optimisation)
   */
  virtual dGeomID getGeomID() =0;

};

#endif
