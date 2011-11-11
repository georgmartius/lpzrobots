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
#ifndef __COLOR_H
#define __COLOR_H

#include <osg/Vec4>
#include <iostream>

namespace lpzrobots{

class Color : public osg::Vec4
{
public:
  Color() {};
  Color(const osg::Vec4& color) 
    : osg::Vec4(color)  {};
  Color(float r, float g, float b)
    : osg::Vec4(r, g, b, 1.0){} 
  Color(float r, float g, float b, float a)
    : osg::Vec4(r, g, b, a){} 

  static Color rgb255(unsigned char r, unsigned char g, unsigned char b, 
                      unsigned char a = 255){
    return Color(((float)r)/255.0, ((float)g)/255.0, ((float)b)/255.0, ((float)a)/255.0); 
  }

  void print(std::ostream& out) const {
    out << '(' << r() << ',' << g() << ',' << b() << ',' << a() << ')';
  }

  friend std::ostream& operator<<(std::ostream& out, const Color& col) {
    col.print(out);
    return out;
  }

/*   float r() const { return x(); } */
/*   float& r() { return x(); } */
/*   float g() const { return y(); } */
/*   float& g() { return y(); } */
/*   float b() const { return z(); } */
/*   float& b() { return z(); } */
/*   float a() const { return w(); } */
/*   float& a() { return w(); } */
  float alpha() const { return w(); }
  float& alpha() { return w(); }
};
  
}

#endif
