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
 *   Revision 1.18  2010-03-08 16:07:45  guettler
 *   fixed display bug that abstractground is not on top of world plane
 *
 *   Revision 1.17  2009/04/02 10:12:25  martius
 *   Texture handling changed
 *
 *   Revision 1.16  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.15  2008/01/29 16:19:30  der
 *   increased groundbox thickness for avoiding collision bugs
 *
 *   Revision 1.14  2007/12/06 10:02:49  der
 *   abstractground: returns now cornerpoints
 *   abstractobstacle: is now trackable
 *   hudstatistics: supports now AbstractmMeasure
 *
 *   Revision 1.13  2007/10/15 13:16:57  martius
 *   changed thickness again, it was mistakingly set to 0.5
 *
 *   Revision 1.12  2007/09/27 12:47:17  der
 *   fixed abstractground placement bug (produces shadow artifacts)
 *
 *   Revision 1.11  2007/09/06 18:46:41  martius
 *   printContours
 *
 *   Revision 1.10  2007/08/27 12:27:35  martius
 *   *** empty log message ***
 *
 *   Revision 1.9  2007/08/24 11:53:10  martius
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
#include "pos.h"
#include "osgprimitive.h"
#include <selforg/stl_adds.h>
#include <assert.h>
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

  void AbstractGround::setTexture(const std::string& filename, double repeatOnX, double repeatOnY){
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
      groundPlane = new Box(groundLength+1.95*wallThickness, groundWidth+1.95*wallThickness, 10.0f);
      groundPlane->setTexture(groundTextureFileName,-5,-5);
      groundPlane->init(odeHandle, 0, osgHandle.changeColor(groundColor),
			Primitive::Geom | Primitive::Draw);
      groundPlane->setPose(osg::Matrix::translate(0.0f,0.0f,-5.0f+0.002f) * pose);
      obst.push_back(groundPlane);
    }
  }


  void printCornerPointsXY(Box* box, FILE* f){
    OSGBox* obox = (OSGBox*)box->getOSGPrimitive();
    std::list<Pos> ps;
    Pos dim = obox->getDim();
    ps.push_back(Pos(dim.x()*  0.5, dim.y()*  0.5,0));
    ps.push_back(Pos(dim.x()*  0.5, dim.y()* -0.5,0));
    ps.push_back(Pos(dim.x()* -0.5, dim.y()* -0.5,0));
    ps.push_back(Pos(dim.x()* -0.5, dim.y()*  0.5,0));
//     for(int i=0; i<8; i++){
//       ps.push_back(Pos(dim.x()*( (i&4) ? 0.5: -0.5),dim.y()*( (i&2) ? 0.5: -0.5),dim.z()*( (i&1) ? 0.5: -0.5)));
//     }
    // transform them into global coords
    FOREACH(std::list<Pos>, ps, p){
      *p = (*p) * box->getPose();
    }
    FOREACHC(std::list<Pos>, ps, p){
      fprintf(f,"%f %f\n",p->x(),p->y());
    }
    fprintf(f,"%f %f\n",ps.begin()->x(),ps.begin()->y());    
  }


  void AbstractGround::printContours(FILE* f){
    assert(f);
    FOREACH(std::vector<Primitive*>, obst, o){
      Box* b= dynamic_cast<Box*>(*o);
      if(b){
	printCornerPointsXY(b, f);
	fprintf(f, "\n\n");
      }
    }
  }

  std::list<Position> AbstractGround::getCornerPointsXY() {
    OSGBox* obox = (OSGBox*)groundPlane->getOSGPrimitive();
    std::list<Pos> ps;
    Pos dim =obox->getDim();
    ps.push_back(Pos(dim.x()*  0.5, dim.y()*  0.5,0));
    ps.push_back(Pos(dim.x()*  0.5, dim.y()* -0.5,0));
    ps.push_back(Pos(dim.x()* -0.5, dim.y()* -0.5,0));
    ps.push_back(Pos(dim.x()* -0.5, dim.y()*  0.5,0));
    // transform them into global coords
    FOREACH(std::list<Pos>, ps, p){
      *p = (*p) * groundPlane->getPose();
    }
    std::list<Position> posList;
    FOREACH(std::list<Pos>, ps, p){
      posList.push_back((*p).toPosition());
    }
    return posList;
}



  
}
