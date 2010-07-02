/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    joergweide84@aol.com (robot12)                                       *
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
 *   Revision 1.11  2010-07-02 15:57:25  martius
 *   wirings have new initIntern signature -> less errors can be made
 *   abstractwiring generates the noise of given length
 *
 *   Revision 1.10  2009/10/29 15:23:55  martius
 *   use GNU_SOURCE
 *
 *   Revision 1.9  2009/10/29 14:51:14  fhesse
 *   *** empty log message ***
 *
 *   Revision 1.8  2009/10/29 14:23:36  martius
 *   random gen also for mac (non-gnu)
 *
 *   Revision 1.7  2009/07/23 13:25:12  guettler
 *   cosmetic
 *
 *   Revision 1.6  2009/07/23 13:24:03  guettler
 *   comments corrected (spelling, grammar)
 *
 *   Revision 1.5  2009/07/22 13:21:14  robot12
 *   bugfix in randomgenerator. so it should be work on a 64bit system.
 *   see comment in line 61 for the union around the buffer.
 *
 *   Revision 1.4  2009/07/21 08:47:33  robot12
 *   add some comments
 *
 *   Revision 1.3  2009/07/15 08:33:58  guettler
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
#ifndef _GNU_SOURCE
#include "mac_drand48r.h"
#endif


/// random generator with 48bit integer arithmentic
typedef struct _RandGen {
  _RandGen(){
    init(::rand());
  }
  void init(long int seedval){
    srand48_r(seedval, &buffer);
  }
  /// returns a value in [0,1)
  double rand(){
    double r;
    drand48_r(&buffer,&r);
    return r;
  }
  // See drand48_data structure:
  //  struct drand48_data
  //  {
  //    unsigned short int __x[3];	/* Current state.  */
  //    unsigned short int __old_x[3]; /* Old state.  */
  //    unsigned short int __c;	/* Additive const. in congruential formula.  */
  //    unsigned short int __init;	/* Flag for initializing.  */
  //    unsigned long long int __a;	/* Factor in congruential formula.  */
  //  };
  //
  // The function drand48_r writes too much data into __x. 
  // I think it writes 16 bytes in the 8 byte sized array.
  // Therefore this destroys the call stack. 
  // With a union and a char array of 24 Bytes we can force him
  // to write all elements in the structure to be aligned in memory 
  // which is not ensured if you have a struct.
  // So the 16 bytes are written into __x and __old_x, the call stack is save.
  // This isn't the best way, but everything which isn't predictable
  // is in a random generator perfect. :o)
  union {
	  struct drand48_data buffer;
	  char dummy[24];
  };
} RandGen;




#endif
