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
 *   Revision 1.20  2010-01-07 14:05:23  der
 *   adapted to new wall texture
 *
 *   Revision 1.19  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.18  2008/02/07 14:25:29  der
 *   added setWallTexture for PlayGround
 *
 *   Revision 1.17  2007/08/28 09:25:13  martius
 *   include primitive.h
 *
 *   Revision 1.16  2007/08/24 11:53:10  martius
 *   Change geometry
 *
 *   Revision 1.15  2007/07/03 13:06:41  martius
 *   groundplane thick
 *
 *   Revision 1.14  2007/03/16 11:01:37  martius
 *   abstractobstacle gets mor functionallity
 *   setSubstance
 *
 *   Revision 1.13  2006/09/21 22:09:12  martius
 *   *** empty log message ***
 *
 *   Revision 1.12  2006/08/11 15:41:04  martius
 *   playgrounds handle non-quadratic ground planes
 *
 *   Revision 1.11  2006/07/14 12:23:33  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.10.4.16  2006/07/14 11:11:53  martius
 *   revert to 1.10.4.12
 *
 *   Revision 1.10.4.12  2006/06/29 16:39:55  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.10.4.11  2006/05/23 13:37:34  robot3
 *   -fixed some creating bugs
 *   -setColor,setTexture and createGround must be
 *    called before setPosition now
 *
 *   Revision 1.10.4.10  2006/05/19 08:42:54  robot3
 *   -some code moved to abstractground.h
 *   -it's now possible creating a playground without a groundplane
 *
 *   Revision 1.10.4.9  2006/05/18 14:32:12  robot3
 *   walls are now textured with a wood texture
 *
 *   Revision 1.10.4.8  2006/05/18 12:54:24  robot3
 *   -fixed not being able to change the color after positioning
 *    the obstacle
 *   -cleared the files up
 *
 *   Revision 1.10.4.7  2006/05/18 09:43:24  robot3
 *   using existing texture image in cvs for the groundplane now
 *
 *   Revision 1.10.4.6  2006/05/18 07:42:36  robot3
 *   Grounds have now a groundPlane for shadowing issues
 *   osgprimitive.cpp contains a bug that repeating textures (wrapping)
 *   don't work, needs to be fixed
 *
 *   Revision 1.10.4.5  2006/05/11 08:59:15  robot3
 *   -fixed a positioning bug (e.g. for passivesphere)
 *   -some methods moved to abstractobstacle.h for avoiding inconsistencies
 *
 *   Revision 1.10.4.4  2006/03/29 15:04:39  martius
 *   have pose now
 *
 *   Revision 1.10.4.3  2006/01/10 20:27:15  martius
 *   protected members
 *
 *   Revision 1.10.4.2  2006/01/10 17:17:33  martius
 *   new mode for primitives
 *
 *   Revision 1.10.4.1  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.10  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.9  2005/09/13 13:19:57  martius
 *   no texture
 *
 *   Revision 1.8  2005/08/02 14:09:06  fhesse
 *   factor between length in x and y direction
 *   added to constructor
 *
 *   Revision 1.7  2005/07/29 14:27:59  martius
 *   color set to some red
 *
 *   Revision 1.6  2005/07/18 14:52:33  martius
 *   world and space are not pointers anymore.
 *
 *   Revision 1.5  2005/07/07 10:24:23  martius
 *   avoid internal collisions
 *
 *   Revision 1.4  2005/06/15 14:22:11  martius
 *   GPL included
 *                                                                 *
 ***************************************************************************/
#ifndef __PLAYGROUND_H
#define __PLAYGROUND_H

#include "mathutils.h"
#include "abstractground.h"
#include "primitive.h"
 
namespace lpzrobots {

  class Playground : public AbstractGround {

  protected:

    double length, width, height;
    double factorlength2;

  public:
  
    Playground(const OdeHandle& odeHandle, const OsgHandle& osgHandle , 
	       const osg::Vec3& dimension = osg::Vec3(7.0, 0.2, 0.5) ,
	       double factorxy = 1, bool createGround=true)
      : AbstractGround(odeHandle, osgHandle, createGround, dimension.x(), dimension.x()*factorxy, dimension.y()) {

      length=dimension.x();
      width=dimension.y();
      height=dimension.z();
      factorlength2=factorxy;
    };

    virtual void changeGeometry(double length, double width, double height, double factorxy){
      AbstractGround::changeGeometry(length, width, height, factorxy);
      this->length = length;
      this->width  = width;
      this->height  = height;
      this->factorlength2  = factorxy;
      if (obstacle_exists) {
	destroy();
	create();
      }      
    }

    virtual void setWallSubstance(const Substance& substance) {
      if (obstacle_exists) {
	FOREACH(std::vector<Primitive*>,obst,wall) {
	  (*wall)->substance=substance;
	}
      } else
	std::cerr << "PlayGround::setWallSubstance() - PlayGround gound not created!\n";

    }


  protected:
    virtual void create(){
      createGround();
      
      Box* box;
      osg::Vec3 offset(- (length/2 + width/2),
		       0,
		       height/2+0.01f/*reduces graphic errors and ode collisions*/);
      box = new Box( width , (length * factorlength2) + 2 * width , height);
      box->setTexture(wallTextureFileName, -2, -4);
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      box->setPose(osg::Matrix::translate(offset) * pose);
      obst.push_back(box);

      offset.x() = length/2 + width/2;
      box = new Box( width , (length * factorlength2) + 2 * width , height);
      box->setTexture(wallTextureFileName, -2, -4);
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      box->setPose(osg::Matrix::translate(offset) * pose);
      obst.push_back(box);

      offset.x() = 0;
      offset.y() = -( (length*factorlength2)/2 +width/2);
      box = new Box( length, width, height);
      box->setTexture(wallTextureFileName, -2, -4);
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      box->setPose(osg::Matrix::translate(offset) * pose);
      obst.push_back(box);

      offset.y() = (length*factorlength2)/2 +width/2;
      box = new Box( length, width, height);
      box->setTexture(wallTextureFileName, -2, -4);
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      box->setPose(osg::Matrix::translate(offset) * pose);
      obst.push_back(box);
    
      obstacle_exists=true;
    };

  };

}

#endif
