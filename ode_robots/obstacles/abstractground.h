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
#ifndef __ABSTRACTGROUND_H
#define __ABSTRACTGROUND_H

#include <list>
#include "abstractobstacle.h"
#include <selforg/position.h>

namespace lpzrobots {

  class Primitive;

  // abstract class for any playground
  class AbstractGround : public AbstractObstacle {

public:

    AbstractGround(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                   bool createGround, double groundLength, double groundWidth, double wallThickness);

    virtual ~AbstractGround();


    virtual void setPose(const osg::Matrix& pose);

    virtual void createGround(bool create);

    virtual Primitive* getMainPrimitive() const;

    virtual void changeGeometry(double length, double width, double height, double factorxy);

    /// prints the contour of the boxes into the file
    virtual void printContours(FILE* f);

    /**
     * assigns the texture to the object
     */
    virtual void setGroundTexture(const std::string& filename);

    /**
     * sets the ground color
     * should be called before setPosition()
     * @param color values in RGBA
     */
    virtual void setGroundColor(const Color& color);

    /**
     * sets the substance of the ground.
     * @param substance description of the substance
     */
    virtual void setGroundSubstance(const Substance& substance);


    /**
     * returns the corner points of the groundplane
     * @return list of the cornerpoints
     */
    virtual std::list<Position> getCornerPointsXY();

    /// size in x dimension
    virtual double getGroundLength(){ return groundLength; }
    /// size in y dimension
    virtual double getGroundWidth(){ return groundWidth; }

    virtual double getGroundThickness(){ return groundThickness; }

    virtual void   setGroundThickness(double thickness);

  protected:

    Primitive* groundPlane; // the groundplane
    bool creategroundPlane;
    double groundLength;
    double groundWidth;
    double wallThickness;
    double groundThickness;
    Substance groundSubstance;
    Color groundColor;
    std::string groundTextureFileName;

    virtual void createGround();

  };

}

#endif
