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
#ifndef __WIRINGSEQUENCE_H
#define __WIRINGSEQUENCE_H

#include "abstractwiring.h"
#include <vector>

/** Implements a sequence of wirings
 */
class WiringSequence :public AbstractWiring{
public:

  /** constructor: The wirings given in the list
      are applied in the sequence. For the sensors in normal order and
      for the motors in reverse order*/
  WiringSequence(std::list<AbstractWiring*>);
  /** constructor provided for convinience, essentially calls addWiring(w1);addWiring(w2)
   */
  WiringSequence(AbstractWiring* w1, AbstractWiring* w2);

  virtual ~WiringSequence();

protected:
  virtual bool initIntern();

  virtual bool wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                 sensor* csensors, int csensornumber,
                                 double noise);

  virtual bool wireMotorsIntern(motor* rmotors, int rmotornumber,
                                const motor* cmotors, int cmotornumber);


public:

  /** adds a wiring to the list of wirings*/
  virtual void addWiring(AbstractWiring* wiring);


protected:
  std::vector<AbstractWiring*> wirings;
  bool initialised;

};

#endif
