/***************************************************************************
 *   Copyright (C) 2005-2014 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
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
#ifndef __SENSORMOTORINFO_H
#define __SENSORMOTORINFO_H

#include <string>
#include "types.h"


/**
 * Interface for objects, that can be stored and restored to/from a file stream (binary).
*/

class SensorMotorInfo {
public:
  enum Type { Continuous, Discrete, Binary };
  enum Quantity { Position, Velocity, Force, Distance, Other };

  SensorMotorInfo(std::string name=std::string())
    : name(name), min(-1.0), max(1.0), index(0), quantity(Position), type(Continuous)
  {}

  CHANGER( SensorMotorInfo, std::string, name);
  CHANGER( SensorMotorInfo, double, min);
  CHANGER( SensorMotorInfo, double, max);
  CHANGER( SensorMotorInfo, Quantity, quantity);
  CHANGER( SensorMotorInfo, Type, type);


  std::string name;
  double min;
  double max;
  int index; // index within one Sensor
  Quantity quantity;
  Type type;
};

#endif
