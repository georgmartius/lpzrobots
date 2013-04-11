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
  //    unsigned short int __x[3];        /* Current state.  */
  //    unsigned short int __old_x[3]; /* Old state.  */
  //    unsigned short int __c;        /* Additive const. in congruential formula.  */
  //    unsigned short int __init;        /* Flag for initializing.  */
  //    unsigned long long int __a;        /* Factor in congruential formula.  */
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
