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
 *   Revision 1.2  2006-07-14 12:24:02  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2005/11/16 11:24:28  martius
 *   moved to selforg
 *
 *   Revision 1.2  2005/10/24 13:32:07  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.1  2005/08/22 17:28:13  martius
 *   a 1 to 1 wiring that supports the selection of some sensors only
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __SELECTIVEONE2ONEWIRING_H
#define __SELECTIVEONE2ONEWIRING_H

#include "one2onewiring.h"

typedef bool (*select_predicate)(unsigned int index, unsigned int len);

bool select_all(unsigned int index, unsigned int len);
bool select_firsthalf(unsigned int index, unsigned int len);


/** 
 *   Implements a selective one to one wireing of robot sensors to inputs of the controller 
 *   and controller outputs to robot motors. 
 */
class SelectiveOne2OneWiring : public One2OneWiring{
public:
  /** constructor
      @param noise NoiseGenerator that is used for adding noise to sensor values  
      @param sel_sensor select_predicate describung what sensors should be selected
  */
  SelectiveOne2OneWiring(NoiseGenerator* noise, select_predicate sel_sensor);
  virtual ~SelectiveOne2OneWiring();

  /** initializes the number of sensors and motors on robot side, calculate
      number of sensors and motors on controller side
  */
  virtual bool init(int robotsensornumber, int robotmotornumber);

  /// Realizes one to one wiring from robot sensors to controller sensors. 
  //   @param rsensors pointer to array of sensorvalues from robot 
  //   @param rsensornumber number of sensors from robot
  //   @param csensors pointer to array of sensorvalues for controller  
  //   @param csensornumber number of sensors to controller
  //   @param noise size of the noise added to the sensors
  virtual bool wireSensors(const sensor* rsensors, int rsensornumber, 
			   sensor* csensors, int csensornumber,
			   double noise);

protected:  
  select_predicate sel_sensor;

};

#endif
