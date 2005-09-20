#ifndef __POSITION_H
#define __POSITION_H

typedef struct Position
{
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

  double x;
  double y;
  double z;
  double array[3];
} Position;

#endif
