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
 *   Revision 1.5  2005-12-12 13:45:32  martius
 *   special inverse for 4x4 matrices in Pose form (can have diagonal zeros)
 *
 *   Revision 1.4  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef __POSITION_H
#define __POSITION_H

#include <math.h>
#include <iostream>

class Position
{
public:
  Position(){x=y=z=0;}
  Position(double _x, double _y, double _z){ x=_x; y=_y; z=_z; }
  ///  p MUST have a size of at least 3 
  Position(const double* p){ x=p[0]; y=p[1]; z=p[2]; } 
  const double* toArray(){ array[0]=x;array[1]=y; array[2]=z; return array; } 
  Position operator+(const Position& sum) const 
           { Position rv(x+sum.x, y+sum.y, z+sum.z); return rv; }
  Position operator-(const Position& sum) const
           { Position rv(x-sum.x, y-sum.y, z-sum.z); return rv; }
  Position operator*(double f) const { Position rv(x*f, y*f, z*f); return rv; }

  double length() { return sqrt(x*x+y*y+z*z);  }
  void print() { std::cout << '(' << x << ',' << y << ',' << z << ')' << std::endl;};

  double x;
  double y;
  double z;
private:
  double array[3];
};


#endif
