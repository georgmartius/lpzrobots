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
#ifndef __OSGHEIGHTFIELD_H
#define __OSGHEIGHTFIELD_H


#include "osgprimitive.h"
#include <osg/Shape>

namespace lpzrobots {

  /**
     Graphical HeightField
  */
  class OSGHeightField : public OSGPrimitive {
  public:

    /** height coding using in the read in bitmap.
     * Red: just the red channel is used;
     * Sum: the sum of all channels is used;
     * HighMidLow: Blue is least significant, Green is medium significant and Red is most significant
     */
    enum CodingMode {Red, Sum, LowMidHigh};


    OSGHeightField(osg::HeightField* heightfield,float x_size, float y_size);
    OSGHeightField(const std::string& filename, float x_size, float y_size, float height);

    virtual void setMatrix(const osg::Matrix& matrix);
    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);

    virtual const osg::HeightField* getHeightField() const { return field; }

    /** loads an ppm image and returns the height field using the given coding and the height
        (maximal height of the heightfield)
    */
    static osg::HeightField* loadFromPPM(const std::string& filename, double height,
                                         CodingMode codingMode=Red);
    /// return the height using the given coding mode. The data pointer points to RGB data point
    static double coding(CodingMode mode, const unsigned char* data);

  protected:
    osg::HeightField* field;
    float x_size;
    float y_size;
  };



}

#endif
