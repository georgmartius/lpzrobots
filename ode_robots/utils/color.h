#ifndef __COLOR_H
#define __COLOR_H

typedef struct Color
{
  Color() {r=g=b=0;};
  Color(double _r, double _g, double _b){ r=_r; g=_g; b=_b; }
  double r;
  double g;
  double b;
} Color;

#endif
