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
#ifndef __POSITION_H
#define __POSITION_H

#include <iostream>
#include <cmath>

class Position
{
public:
  Position(){x=y=z=0; array[0]=array[1]=array[2]=0;}
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

  double x;
  double y;
  double z;
  
  void print(){
    std::cout << '(' << x << ',' << y << ',' << z << ')' << std::endl;
  }
  
private:
  double array[3];
};

#endif
