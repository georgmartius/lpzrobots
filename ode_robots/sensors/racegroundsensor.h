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
 *   Revision 1.1  2005-11-22 10:22:35  martius
 *   sensor for raceground position
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __RACEGROUNDSENSOR_H
#define __RACEGROUNDSENSOR_H

#include <list>
#include <ode/common.h>
#include "raceground.h"
#include "globaldata.h"

/** Class for Raceground-sensor, which determines the position of the car on the 
    raceground, see obstacles/raceground.h
 */
class RaceGroundSensor {
public:  
  RaceGroundSensor();
  virtual ~RaceGroundSensor() {}
  
  /** initialises sensor with body of robot
      @return number of sensor values returned by get
   */
  virtual int init(dBodyID body);  
  
  /** performs sense action 
      @return false if raceground object was not found
   */
  virtual bool sense(const GlobalData& globaldata);

  /** returns a list of sensor values (usually in the range [0,1] )
   */
  virtual std::list<double> get();

protected:
  dBodyID robot;
  double position;
  double segment;
};

#endif
