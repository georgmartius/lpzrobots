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
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2005-11-22 15:50:00  robot3
 *   testing
 *
 *   Revision 1.2  2005/11/22 14:13:31  robot3
 *   call of raceground functions inserted
 *
 *   Revision 1.1  2005/11/22 10:21:48  martius
 *   sensor for raceground position
 *
 *                                                                         *
 ***************************************************************************/

#include "racegroundsensor.h"
#include "raceground.h"
using namespace std;

RaceGroundSensor::RaceGroundSensor() { 
  robot = 0;
  segmentPosition = 0;
  widthPosition  = 0;
}

/** initialises sensor with body of robot
    @return number of sensor values returned by get
*/
int RaceGroundSensor::init(dBodyID body){
  robot = body;
  return 2;
}
  
/** performs sense action 
    @return false if raceground object was not found
*/
bool RaceGroundSensor::sense(const GlobalData& global){
  if(!robot) return false;
  bool found = false;
  Position robotPos = Position(dBodyGetPosition(robot));
  for(ObstacleList::const_iterator i = global.obstacles.begin(); 
      i!= global.obstacles.end(); i++){
    RaceGround* rg = dynamic_cast<RaceGround*>(*i);
    if (rg!=0) {
      // Todo add stuff here
      segmentPosition = rg->getSegmentNumberOfRobot(robotPos);
      widthPosition = rg->getWidthOfRobot(robotPos);
      found=true;
    }    
  }
  return found;
}

/** returns a list of sensor values (usually in the range [0,1] )
 */
list<double> RaceGroundSensor::get(){
  list<double> values;
  values += segmentPosition;
  values += widthPosition;
  return values;
}
