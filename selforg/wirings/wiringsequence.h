/***************************************************************************
 *   Copyright (C) 2007 by Robot Group Leipzig                             *
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
 *   Revision 1.1  2007-11-29 19:18:33  martius
 *   sequence of wirings
 *
 *   Revision 1.1  2007/11/28 10:30:56  martius
 *   wiring with feedback connections
 *
 *                                            *
 *                                                                         *
 ***************************************************************************/
#ifndef __WIRINGSEQUENCE_H
#define __WIRINGSEQUENCE_H

#include "abstractwiring.h"

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

  virtual bool init(int robotsensornumber, int robotmotornumber);

  virtual bool wireSensors(const sensor* rsensors, int rsensornumber, 
			   sensor* csensors, int csensornumber,
			   double noise);

  virtual bool wireMotors(motor* rmotors, int rmotornumber,
			  const motor* cmotors, int cmotornumber);

  /** adds a wiring to the list of wirings*/
  virtual void addWiring(AbstractWiring* wiring);

protected:  
  std::vector<AbstractWiring*> wirings;
  bool initialised;

};

#endif
