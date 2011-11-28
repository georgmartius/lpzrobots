/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
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

#include <assert.h>

#include "primitive.h"
#include "relativepositionsensor.h"
#include "mathutils.h"

namespace lpzrobots {

  RelativePositionSensor::RelativePositionSensor(double maxDistance, double exponent, 
						 short dimensions /* = X | Y | Z */ , bool local_coordinates /*= false*/)
    : maxDistance(maxDistance), exponent(exponent), dimensions (dimensions), local_coords(local_coordinates){
    own=0;
    ref=0;
  }
  
  void RelativePositionSensor::init(Primitive* own){
    this->own = own;
  }
  void RelativePositionSensor::setReference(Primitive* ref){
    this->ref = ref;
  }

  int RelativePositionSensor::getSensorNumber() const{
    return (dimensions & X) + ((dimensions & Y) >> 1)  + ((dimensions & Z) >> 2);
  }
  
  std::list<sensor> RelativePositionSensor::get() const {
    assert(ref);    assert(own);
    std::list<sensor> s;
    osg::Vec3 v;
    if (local_coords){
      v = own->toLocal(ref->getPosition());
    }else{
      v = ref->getPosition() - own->getPosition();
    }
    double scale = pow(v.length() / maxDistance, exponent);
    v *= (1/maxDistance)*scale;
    if (dimensions & X) s.push_back(v.x());
    if (dimensions & Y) s.push_back(v.y());
    if (dimensions & Z) s.push_back(v.z());
    return s;
  }
  
  bool RelativePositionSensor::sense(const GlobalData& globaldata){
    return true;
  }
  
  
  int RelativePositionSensor::get(sensor* sensors, int length) const{
    assert(ref);    assert(own);
    int i = 0;
    assert ( length >= getSensorNumber() );
    osg::Vec3 v;
    if (local_coords){
      v = own->toLocal(ref->getPosition());
    }else{
      v = ref->getPosition() - own->getPosition();
    }
    double scale = pow(v.length() / maxDistance, exponent);
    v *= (1/maxDistance)*scale;
    if (dimensions & X) sensors[i++] = v.x();
    if (dimensions & Y) sensors[i++] = v.y();
    if (dimensions & Z) sensors[i++] = v.z();
    return i;
  }

}
