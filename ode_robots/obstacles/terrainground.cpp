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

#include <stdio.h>
// #include <cmath>
// #include <osg/Geode>
// #include <osg/Geometry>
// #include <osg/Texture2D>
// #include <osg/TexEnv>
// #include <osg/StateSet>

#include <osgDB/ReadFile>

#include "terrainground.h"

namespace lpzrobots {

  using namespace osg;

  TerrainGround::TerrainGround(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                         const std::string& filename, const std::string& texture,
                         double x_size, double y_size, double height,
                         OSGHeightField::CodingMode coding)
    :  AbstractObstacle::AbstractObstacle(odeHandle, osgHandle),
       filename(filename), texture(texture), heightfield(0),
       x_size(x_size), y_size(y_size), height(height), coding(coding){
    obstacle_exists=false;
  };


  void TerrainGround::setPose(const osg::Matrix& pose){
    this->pose = pose;
    if (obstacle_exists){
      destroy();
    }
    create();
  }

  void TerrainGround::create(){
    if(strstr(filename.c_str(),".ppm")){
      heightfield = new HeightField(OSGHeightField::loadFromPPM(filename,height, coding),
                                    x_size, y_size);
    }else{
      heightfield = new HeightField(filename, x_size, y_size, height);
    }
    if(!texture.empty())
      heightfield->setTexture(texture);
    heightfield->setPose(pose);
    heightfield->init(odeHandle, 0, osgHandle);
    obst.push_back(heightfield);
    obstacle_exists=true;
  }


  void TerrainGround::destroy(){
    if(heightfield) delete(heightfield);
    obstacle_exists=false;

  }

}

