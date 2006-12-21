/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2006-12-21 11:41:06  martius
 *   sensor from measuring speed
 *
 *
 *                                                                 *
 ***************************************************************************/

#include <assert.h>

#include "primitive.h"
#include "speedsensor.h"
#include "mathutils.h"

using namespace matrix;

namespace lpzrobots {

  SpeedSensor::SpeedSensor(double maxSpeed, Mode mode /* = Translational */,
			   short dimensions /* = X | Y | Z */ )
    : maxSpeed(maxSpeed), mode(mode), dimensions (dimensions) {
    own=0;
  }
  
  void SpeedSensor::init(Primitive* own){
    this->own = own;
  }

  int SpeedSensor::getSensorNumber() const{
    switch(mode){
    case Translational:
      return (dimensions & X) + ((dimensions & Y) >> 1)  + ((dimensions & Z) >> 2);
      break;
    case Rotational:
      return 3* ( (dimensions & X) + ((dimensions & Y) >> 1)  + ((dimensions & Z) >> 2) );
      break;      
    }
    return 0;
  }
  
  bool SpeedSensor::sense(const GlobalData& globaldata) { return true; }

  std::list<sensor> SpeedSensor::get() const {
    assert(own);
    assert(own->getBody());
    Matrix m;
    switch(mode){
    case Translational:
      m.set(3,1, dBodyGetLinearVel(own->getBody()));
      break;
    case Rotational:
      m = odeRto3x3RotationMatrix(dBodyGetRotation(own->getBody()));       
      break;      
    }
    if(dimensions == X | Y | Z) 
      return m.convertToList(); 
    else
      return selectrows(m,dimensions);
  }

  int SpeedSensor::get(sensor* sensors, int length) const{
    assert(own);
    assert(own->getBody());
    Matrix m;
    switch(mode){
    case Translational:
      m.set(3,1, dBodyGetLinearVel(own->getBody()));
      break;
    case Rotational:
      m = odeRto3x3RotationMatrix(dBodyGetRotation(own->getBody()));       
      break;      
    }
    if(dimensions == X | Y | Z) 
      return m.convertToBuffer(sensors, length); 
    else
      return selectrows(sensors, length, m,dimensions);
  }
  
}
