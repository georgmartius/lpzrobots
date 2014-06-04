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
    setBaseInfo(SensorMotorInfo("RelPos").changequantity(SensorMotorInfo::Distance).changemin(0));
#if (__GNUC__ > 4 ) || (__GNUC__ == 4 && __GNUC_MINOR__ > 7)
    setNamingFunc([dimensions](int index) {return dimensions2String(dimensions).substr(index,1);});
#endif
  }


  void RelativePositionSensor::init(Primitive* own, Joint* joint){
    this->own = own;
  }
  void RelativePositionSensor::setReference(Primitive* ref){
    this->ref = ref;
  }

  int RelativePositionSensor::getSensorNumber() const{
    return (dimensions & X) + ((dimensions & Y) >> 1)  + ((dimensions & Z) >> 2);
  }

  std::list<sensor> RelativePositionSensor::getList() const {
    assert(own);
    std::list<sensor> s;
    osg::Vec3 v;
    osg::Vec3 refpos = ref ? ref->getPosition() : osg::Vec3(0,0,0);
    if (local_coords){
      v = own->toLocal(refpos);
    }else{
      v = refpos - own->getPosition();
    }
    // those components that we don't need we set to zero.
    if (!(dimensions & X)) v.x()=0;
    if (!(dimensions & Y)) v.y()=0;
    if (!(dimensions & Z)) v.z()=0;

    double rellen = std::min(1.0 ,v.length() / maxDistance); // between 0 and 1

    double scale = rellen>0 ? pow(rellen, exponent)/rellen : 1; // exponential characteristics divided by linear characteristics
    // nonlinear scaling of the vector, such that
    v *= (scale/maxDistance);
    if (dimensions & X) s.push_back(v.x());
    if (dimensions & Y) s.push_back(v.y());
    if (dimensions & Z) s.push_back(v.z());
    return s;
  }

  bool RelativePositionSensor::sense(const GlobalData& globaldata){
    return true;
  }

}
