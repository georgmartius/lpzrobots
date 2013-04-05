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
#ifndef __TERRAINGROUND_H
#define __TERRAINGROUND_H

#include <stdio.h>
#include <cmath>

#include "heightfieldprimitive.h"
#include "abstractobstacle.h"

namespace lpzrobots {

  /** Class provides an terrain based on HeightFields.
      Can be loaded from image or from HeightFieldFiles
  */
  class TerrainGround : public AbstractObstacle {
  public:


    /** Constructor
        @param odeHandle
        @param osgHandle
        @param filename name of the file to load.
        If ending is .ppm then it is considered as a bitmap height file.
        The coding mode is used to decode the heights.
        Otherwise it is consider to be a OSG HeightFieldFile
        @param texture image filename for the texture
        @param x_size size in x direction in world coordinates
        @param y_size size in y direction in world coordinates
        @param height height in world coordinates
        @param coding Coding mode, see OSGHeightField (this is an own class, not in OSG)
    */
    TerrainGround(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
               const std::string& filename, const std::string& texture,
               double x_size, double y_size, double height,
               OSGHeightField::CodingMode coding = OSGHeightField::Red);

    virtual ~TerrainGround(){}

    /**
     * updates the position of the geoms  ( not nessary for static objects)
     */
    virtual void update(){ };

    virtual void setPose(const osg::Matrix& pose);


    virtual Primitive* getMainPrimitive() const { return 0; }

  protected:
    virtual void create();
    virtual void destroy();

  protected:
    std::string filename;
    std::string texture;
    HeightField* heightfield;
    double x_size;
    double y_size;
    double height;
    OSGHeightField::CodingMode coding;
  };
}

#endif
