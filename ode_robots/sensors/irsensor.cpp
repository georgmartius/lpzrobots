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
#include "irsensor.h"

namespace lpzrobots {

  IRSensor::IRSensor(double exponent, double size, double range, rayDrawMode drawMode)
  : RaySensor(size, range, drawMode) {

    this->exponent = exponent;
    value = 0;
  }


  bool IRSensor::sense(const GlobalData& globaldata){
    RaySensor::sense(globaldata);
    value = characteritic(len);
    return true;
  }

  double IRSensor::getValue(){
    return value;
  }

  int IRSensor::get(sensor* sensors, int length) const {
    assert(length>0);
    sensors[0]=value;
    return 1;
  }

  std::list<sensor> IRSensor::getList() const {
    return {value};
  }

  double IRSensor::characteritic(double len){
    double v = (range - len)/range;
    return v < 0 ? 0 : pow(v, exponent);
  }

}
