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
#include <string>

#include "primitive.h"
#include "speedsensor.h"
#include "mathutils.h"


using namespace matrix;

namespace lpzrobots {

  SpeedSensor::SpeedSensor(double maxSpeed, Mode mode /* = Translational */,
                           short dimensions /* = X | Y | Z */ )
    : maxSpeed(maxSpeed), mode(mode), dimensions (dimensions) {
    own=0;
    std::string name = "Speed";
    switch(mode){
    case Translational:   name += "Translational"; break;
    case TranslationalRel:name += "TranslationalRel"; break;
    case Rotational:      name += "Rotational"; break;
    case RotationalRel:   name += "RotationalRel"; break;
    }
    setBaseInfo(SensorMotorInfo(name).changequantity(SensorMotorInfo::Velocity));
#if (__GNUC__ > 4 ) || (__GNUC__ == 4 && __GNUC_MINOR__ > 7)
    setNamingFunc([dimensions](int index) {return dimensions2String(dimensions).substr(index,1);});
#endif
  }

  void SpeedSensor::init(Primitive* own, Joint* joint){
    this->own = own;
  }

  int SpeedSensor::getSensorNumber() const{
    return (dimensions & X) + ((dimensions & Y) >> 1)  + ((dimensions & Z) >> 2);
  }

  bool SpeedSensor::sense(const GlobalData& globaldata) { return true; }

  std::list<sensor> SpeedSensor::getList() const {
    const Matrix& m = getSenseMatrix()*(1.0/maxSpeed);
    return selectrows(m,dimensions);
  }

  int SpeedSensor::get(sensor* sensors, int length) const{
    const Matrix& m = getSenseMatrix()*(1.0/maxSpeed);
    if(dimensions == (X | Y | Z))
      return m.convertToBuffer(sensors, length);
    else{
      return selectrows(sensors, length, m, dimensions);
    }
  }

  Matrix SpeedSensor::getSenseMatrix() const {
    assert(own);
    assert(own->getBody());
    Matrix local;
    Matrix m;
    switch(mode){
    case Translational:
      m.set(3,1, dBodyGetLinearVel(own->getBody()));
      break;
    case TranslationalRel:
      local = osgMatrix2Matrixlib(own->getPose());
      m.set(4,1, dBodyGetLinearVel(own->getBody()));
      m.val(3,0)=0; // we have a vector and not a point (homogeneous coordinates)
      m=local*m;
      m.reshape(3,1);
      break;
    case Rotational:
      m.set(3,1, dBodyGetAngularVel(own->getBody()));
      break;
    case RotationalRel:
      local = osgMatrix2Matrixlib(own->getPose());
      m.set(4,1, dBodyGetAngularVel(own->getBody()));
      m.val(3,0)=0; // we have a vector and not a point (homogeneous coordinates)
      m=local*m;  // this is m^T= m^T*local^T, ode matrix multiplications are the other way around (left sided)
      m.reshape(3,1);
      break;
    }
    return m;
  }

}
