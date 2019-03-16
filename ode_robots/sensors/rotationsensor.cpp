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

#include "rotationsensor.h"

namespace lpzrobots {

  RPYsensor::RPYsensor(Mode mode, short dimensions)
    : mode(mode), dimensions(dimensions) {
    own = 0;
    setBaseInfo(SensorMotorInfo("AxisOrientation").changequantity(SensorMotorInfo::Position));
  }

  void RPYsensor::init(Primitive* own, Joint* joint){
    this->own = own;
  }

  int RPYsensor::getSensorNumber() const{

    short n = ((dimensions & X) != 0) + ((dimensions & Y) != 0) + ((dimensions & Z) != 0);
    switch (mode) {
    case OnlyZAxis:
    case ZProjection:
      return n;
      break;
    case Axis:
      return 3*n;
      break;
    }
    return 0;
  }

  bool RPYsensor::sense(const GlobalData& globaldata) { return true; }

  std::list<sensor> RPYsensor::getList() const {
    assert(own);
    matrix::Matrix A = odeRto3x3RotationMatrix ( dBodyGetRotation ( own->getBody() ) );

    std::list<sensor> sensor_list;
    sensor_list += std::list<double>( getRoll(A) );
    sensor_list += std::list<double>( getPitch(A) );
    sensor_list += std::list<double>( getYaw(A) );

    return sensor_list;
  }

  int RPYsensor::get(sensor* sensors, int length) const{
    assert(own);
    matrix::Matrix A = odeRto3x3RotationMatrix ( dBodyGetRotation ( own->getBody() ) );

    double rpy[3];
    rpy[0] = atan2( A.val(1,0), A.val(0,0) );
    rpy[1] = atan2(-A.val(2,0), sqrt( pow(A.val(2,1),2) + pow(A.val(2,2),2) ) );
    rpy[2] = atan2( A.val(2,1), A.val(2,2) );

    memcpy ( sensors, rpy, 3*sizeof ( double ) );

    return 0;
}

double RPYsensor::getRoll(matrix::Matrix matrix) const {
	return atan2( matrix.val(1,0), matrix.val(0,0) );
}

double RPYsensor::getPitch(matrix::Matrix matrix) const {
	return atan2(-matrix.val(2,0), sqrt( pow(matrix.val(2,1),2) + pow(matrix.val(2,2),2) ) );
}

double RPYsensor::getYaw(matrix::Matrix matrix) const {
	return atan2( matrix.val(2,1), matrix.val(2,2) );
}

}
