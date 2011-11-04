
/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Antonia Siegert (original author)                                  *
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
#ifndef __SENSOR_TYPES_H
#define __SENSOR_TYPES_H


#include <list>
#include <string>

const std::string SENSOR_TILT = "tilt";
const std::string SENSOR_DEFAULT = "def";
const std::string SENSOR_IR = "ir";
const std::string SENSOR_MOTOR_CURRENT = "mc";
const std::string SENSOR_MOTOR_SPEED = "ms";
const std::string SENSOR_COMPASS = "comp";

class SensorTypes
{
  
public:
  
  
  
  
  static std::list<std::string> getAllTypes() {
    if (typeList.empty()) {
      typeList.push_back(SENSOR_TILT);
      typeList.push_back(SENSOR_IR);
      typeList.push_back(SENSOR_DEFAULT);
      typeList.push_back(SENSOR_MOTOR_CURRENT);
      typeList.push_back(SENSOR_MOTOR_SPEED);
      typeList.push_back(SENSOR_COMPASS);
    }
    return typeList;
  };
  
private:
  static std::list<std::string> typeList;
  
};


#endif


