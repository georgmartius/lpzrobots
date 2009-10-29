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
 *   This is a copy of the stdlib version                                  *
 *                                                                         *
 *   $Log$
 *   Revision 1.8  2009-10-29 15:26:14  martius
 *   typo
 *
 *   Revision 1.7  2009/10/29 15:24:20  martius
 *   removed implementation, just declaration
 *
 *   Revision 1.6  2009/10/29 15:20:03  fhesse
 *   *** empty log message ***
 *
 *   Revision 1.5  2009/10/29 14:51:14  fhesse
 *   *** empty log message ***
 *
 *   Revision 1.4  2009/10/29 14:44:41  martius
 *   got ieee754 double back
 *
 *   Revision 1.3  2009/10/29 14:37:19  martius
 *   reduced to Little Endian
 *
 *   Revision 1.2  2009/10/29 14:33:24  martius
 *   hard coded little endian
 *
 *   Revision 1.1  2009/10/29 14:23:26  martius
 *   random gen also for mac (non-gnu)
 *
 *
 ***************************************************************************/
#ifndef __MACDRAND48_R_H
#define __MACDRAND48_R_H

#include <stdlib.h>

#include <limits.h>
#include <mach/mach.h>


union ieee754_double
  {
    double d;

    /* This is the IEEE 754 double-precision format.  */
    struct
      {
	/* Together these comprise the mantissa.  */
	unsigned int mantissa1:32;
	unsigned int mantissa0:20;
	unsigned int exponent:11;
	unsigned int negative:1;
      } ieee;

    /* This format makes it easier to see if a NaN is a signalling NaN.  */
    struct
      {
	/* Together these comprise the mantissa.  */
	unsigned int mantissa1:32;
	unsigned int mantissa0:19;
	unsigned int quiet_nan:1;
	unsigned int exponent:11;
	unsigned int negative:1;
      } ieee_nan;
  };

#define IEEE754_DOUBLE_BIAS	0x3ff /* Added to exponent.  */


struct drand48_data {
  unsigned short int __x[3];	/* Current state.  */
  unsigned short int __old_x[3]; /* Old state.  */
  unsigned short int __c;	/* Additive const. in congruential formula.  */
  unsigned short int __init;	/* Flag for initializing.  */
  unsigned long long int __a;	/* Factor in congruential formula.  */
};


int srand48_r (long int seedval, struct drand48_data *buffer);

int drand48_r ( struct drand48_data *buffer, double *result);


#endif
