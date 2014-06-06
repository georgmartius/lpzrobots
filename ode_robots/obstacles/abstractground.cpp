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
#include "abstractground.h"

#include "primitive.h"
#include "pos.h"
#include "osgprimitive.h"
#include <selforg/stl_adds.h>
#include <assert.h>
#include <iostream>


namespace lpzrobots {

  AbstractGround::AbstractGround(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                                 bool createGround, double groundLength,
                                 double groundWidth, double wallThickness)
    : AbstractObstacle(odeHandle, osgHandle),
      creategroundPlane(createGround), groundLength(groundLength),
      groundWidth(groundWidth), wallThickness(wallThickness),
      groundSubstance(odeHandle.substance) {
    groundPlane=0;
    groundThickness = 0.1;
    setTexture(0,0,TextureDescr("Images/wall.jpg",-1.5,-3)); // was: wall.rgb
    groundTextureFileName="Images/whiteground.jpg";
    groundColor=osgHandle.colorSchema()->color("arenaground");
    setColor(osgHandle.colorSchema()->color("wall"));
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
    groundSubstance = substance;
    if(creategroundPlane && groundPlane)
      groundPlane->setSubstance(groundSubstance);
    // else std::cerr << "AbstractGround::setGroundSubstance() ground not created or no ground used!\n";
  }

  void AbstractGround::setGroundThickness(double thickness) {
    assert(groundPlane==0 || "call setGroundThickness before creation of playground!");
    groundThickness = thickness;
  }

  void AbstractGround::createGround() {
    if (creategroundPlane) {
      // now create the plane in the middle
      groundPlane = new Box(groundLength+1.95*wallThickness,
                            groundWidth+1.95*wallThickness, groundThickness + 1.0);
      groundPlane->setTexture(TextureDescr(groundTextureFileName,-5,-5));
      groundPlane->init(odeHandle, 0, osgHandle.changeColor(groundColor),
                        Primitive::Geom | Primitive::Draw);
      groundPlane->setSubstance(groundSubstance);
      groundPlane->setPose(osg::Matrix::translate(0.0f,0.0f,groundThickness/2.0-0.5) * pose);
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
