/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.5  2010-03-09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.4  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.3  2008/09/16 14:49:46  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.2  2006/07/14 12:23:32  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.2  2006/05/28 22:29:46  martius
 *   some cleanup
 *
 *   Revision 1.1.2.1  2006/05/28 22:14:56  martius
 *   heightfield included
 *
 *
 *                                                                 *
 ***************************************************************************/

#include <stdio.h>
#include <cmath>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/StateSet>

#include <osgDB/ReadFile>


#include <ode-dbl/ode.h>
#include <ode-dbl/common.h>

#include "meshground.h"
 
namespace lpzrobots {

  using namespace osg;

  MeshGround::MeshGround(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
			 const std::string& filename, const std::string& texture, 
			 double x_size, double y_size, double height,
			 OSGHeightField::CodingMode coding)
    :  AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), 
       filename(filename), texture(texture), 
       x_size(x_size), y_size(y_size), height(height), coding(coding){
    obstacle_exists=false;    
  };

  
void MeshGround::setPose(const osg::Matrix& pose){
  this->pose = pose;
  if (obstacle_exists){
    destroy();
  }
  create();
}

void MeshGround::create(){    
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
  
  obstacle_exists=true;
}
  

void MeshGround::destroy(){
  if(heightfield) delete(heightfield);
  obstacle_exists=false;

}

}

