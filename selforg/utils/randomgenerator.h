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
 *   Random generator with internal state used for multitheading envs.     *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-07-15 08:33:58  guettler
 *   workaround for bug: drand48_r overwrites too much data
 *   - buffer increased (thanks to Joern Hoffmann)
 *
 *   Revision 1.2  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.1  2008/04/15 17:03:34  martius
 *   random generator with state
 *
 ***************************************************************************/
#ifndef __RANDOMGENERATOR_H
#define __RANDOMGENERATOR_H

#include <stdlib.h>

typedef struct _RandGen {
  _RandGen(){
    init(::rand());
  }
  void init(long int seedval){
    srand48_r(seedval, buffer);
  }
  double rand(){
    double r;
    drand48_r(buffer,&r);
    return r;
  }
  // allocate more memory than needed, because there is a bug
  // with 64bit compilers: dran48_r overwrites too much data!
  struct drand48_data buffer[2];
} RandGen;

#endif
