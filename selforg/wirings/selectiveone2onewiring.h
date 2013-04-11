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
#ifndef __SELECTIVEONE2ONEWIRING_H
#define __SELECTIVEONE2ONEWIRING_H

#include "one2onewiring.h"
#include <functional>

/** predicate to select sensors.
    First parameter is the index
    and the second parameter is the length (or number of sensors).
*/
struct select_predicate : public std::binary_function< int,  int, bool> {
  virtual ~select_predicate(){}
  virtual bool operator()( int index,  int len) { return true; }
};

struct select_all : public  select_predicate { };

struct select_firsthalf : public  select_predicate {
  virtual ~select_firsthalf(){}
  virtual bool operator()( int index,  int len) { return index < len/2; }
};

/// select sensors in the range \f[ [from, to] \f] (inclusively)
struct select_from_to : public  select_predicate {
  virtual ~select_from_to(){}
  select_from_to( int from,  int to) : from(from), to(to) {}
  virtual bool operator()( int index,  int len) { return (index >= from) && (index <= to); }
  int from;
  int to;
};

/**
 *   Implements a selective one to one wiring of robot sensors to inputs of the controller
 *   and controller outputs to robot motors.
 */
class SelectiveOne2OneWiring : public One2OneWiring{
public:
  /** constructor
      @param noise NoiseGenerator that is used for adding noise to sensor values
      @param sel_sensor binary predicate taking the index and the length (number of sensors)
             and decides which sensor to select
  */
  SelectiveOne2OneWiring(NoiseGenerator* noise, select_predicate* sel_sensor, int plotMode = Controller, const std::string& name = "SelectiveOne2OneWiring");
  virtual ~SelectiveOne2OneWiring();

protected:
  virtual bool initIntern();

  virtual bool wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                 sensor* csensors, int csensornumber,
                                 double noise);

protected:
  select_predicate* sel_sensor;

};

#endif
