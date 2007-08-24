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
 *   Revision 1.9  2007-08-24 11:53:10  martius
 *   Change geometry
 *
 *   Revision 1.8  2007/07/03 13:06:41  martius
 *   groundplane thick
 *
 *   Revision 1.7  2007/05/08 10:18:15  der
 *   added a function for starting the measure after a given time.
 *   made some tests
 *
 *   Revision 1.6  2007/04/05 15:10:14  martius
 *   extra ground substance
 *
 *   Revision 1.5  2007/03/16 11:01:37  martius
 *   abstractobstacle gets mor functionallity
 *   setSubstance
 *
 *   Revision 1.4  2006/11/17 13:42:26  martius
 *   Recreation in setColor and ...
 *   removed error Message
 *
 *   Revision 1.3  2006/08/11 15:41:04  martius
 *   playgrounds handle non-quadratic ground planes
 *
 *   Revision 1.2  2006/07/14 12:23:32  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.4  2006/07/14 11:19:49  martius
 *   revert to 1.1.2.1
 *
 *   Revision 1.1.2.1  2006/06/29 16:43:20  robot3
 *   abstract classes have now own .cpp-files
 *
 *   Revision 1.1.2.6  2006/06/26 08:25:03  robot3
 *   fixed ground texture bug
 *
 *   Revision 1.1.2.5  2006/06/22 12:25:45  der
 *   added setGroundTexture and setGroundColor
 *
 *   Revision 1.1.2.4  2006/06/16 22:27:26  martius
 *   getMainPrimtive
 *
 *   Revision 1.1.2.3  2006/05/23 14:13:41  der
 *   fixed initialization bug
 *
 *   Revision 1.1.2.2  2006/05/23 13:37:45  robot3
 *   -fixed some creating bugs
 *   -setColor,setTexture and createGround must be
 *    called before setPosition now
 *
 *   Revision 1.1.2.1  2006/05/19 08:41:27  robot3
 *   Class AbstractGround contains now basic routines like
 *   creating the groundPlane, setPose and so on
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#include "abstractground.h"

#include "primitive.h"
#include "osgprimitive.h"
#include <iostream>


namespace lpzrobots {

  AbstractGround::AbstractGround(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
				 bool createGround, double groundLength, double groundWidth, double wallThickness)
    : AbstractObstacle(odeHandle, osgHandle), 
      creategroundPlane(createGround), groundLength(groundLength), groundWidth(groundWidth), 
      wallThickness(wallThickness) {
    groundPlane=0;
    wallTextureFileName="Images/wall.rgb";
    groundTextureFileName="Images/greenground.rgb";
    groundColor=Color(1.0f,1.0f,1.0f);
  };
  
  AbstractGround::~AbstractGround(){
    destroy();
  }
  

  void AbstractGround::setPose(const osg::Matrix& pose){
    this->pose = pose;
    if(obstacle_exists){
      destroy();     
    }
    create();
  };

  void AbstractGround::createGround(bool create) {
    creategroundPlane=create;
    if (obstacle_exists) {
      std::cout << "ERROR: createGround(bool create)  has no effect AFTER setPosition(osg::Vec3) !!!" 
		<< std::endl;
      std::cout << "Program terminated. Please correct this error in main.cpp first." << std::endl;
      exit(-1);
    }
  }

  void AbstractGround::changeGeometry(double length, double width, double height, double factorxy){
    this->groundLength = length;
    this->groundWidth = length*factorxy;
    this->wallThickness = wallThickness;
  }

  void AbstractGround::setTexture(const std::string& filename){
    wallTextureFileName=filename;
    if (obstacle_exists) {
      destroy();
      create();
    }
  }

  Primitive* AbstractGround::getMainPrimitive() const { 
    if(groundPlane)
      return groundPlane; 
    else return obst[0];
  }

  void AbstractGround::setGroundTexture(const std::string& filename){
    groundTextureFileName=filename;
    if (obstacle_exists) {
      std::cout << "ERROR: "
		<< "setGroundTexture(const std::sting& filename) has no effect AFTER setPosition(osg::Vec3) !!!"
		<< std::endl;
      std::cout << "Program terminated. Please correct this error in main.cpp first." << std::endl;
      exit(-1);
    }
  }

  /**
   * sets the ground color
   * should be called before setPosition()
   * @param color values in RGBA
   */
  void AbstractGround::setGroundColor(const Color& color) {
    groundColor = color;
    if (obstacle_exists) {
      std::cout << "ERROR: "
		<< "setGroundColor(const Color& color) has no effect AFTER setPosition(osg::Vec3) !!!"
		<< std::endl;
      std::cout << "Program terminated. Please correct this error in main.cpp first." << std::endl;
      exit(-1);
    }
  };

  void AbstractGround::setGroundSubstance(const Substance& substance){
    if(creategroundPlane && groundPlane)
      groundPlane->substance=substance;
    else std::cerr << "AbstractGround::setGroundSubstance() gound not created or no ground used!\n";
  }


  void AbstractGround::createGround() {
    if (creategroundPlane) {
      // now create the plane in the middle
      groundPlane = new Box(groundLength+1.95*wallThickness, groundWidth+1.95*wallThickness, 0.50f);
      groundPlane->init(odeHandle, 0, osgHandle.changeColor(groundColor),
			Primitive::Geom | Primitive::Draw);
      groundPlane->setPose(osg::Matrix::translate(0.0f,0.0f,-0.25f) * pose);
      groundPlane->setTexture(groundTextureFileName,true,true);
      obst.push_back(groundPlane);
    }
  }
  
}
