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
 *   Revision 1.2.4.3  2006-02-08 16:16:23  martius
 *   no namespace using
 *
 *   Revision 1.2.4.2  2006/01/12 14:37:39  martius
 *   access functions
 *
 *   Revision 1.2.4.1  2005/12/06 10:13:26  martius
 *   openscenegraph integration started
 *
 *   Revision 1.2  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef __COLOR_H
#define __COLOR_H

#include<osg/Vec4>

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
