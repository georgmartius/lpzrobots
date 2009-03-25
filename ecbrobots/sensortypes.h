
/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    marcoeckert@web.de                                                   *
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
 *   Revision 1.1  2009-03-25 11:16:49  robot1
 *   neue Version
 *
 *   Revision 1.7  2008/08/15 13:16:58  robot1
 *   add PORT PIN configuration for ECBs
 *
 *   Revision 1.6  2008/08/12 11:45:00  guettler
 *   plug and play update, added some features for the ECBRobotGUI
 *
 *   Revision 1.5  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.4  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.3  2008/04/11 06:15:48  guettler
 *   Inserted convertion from byte to double and backwards for motor and sensor values
 *
 *   Revision 1.2  2008/04/08 09:09:09  martius
 *   fixed globaldata to pointer in classes
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
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


