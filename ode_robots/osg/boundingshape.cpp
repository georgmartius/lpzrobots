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

#include "boundingshape.h"
#include "mathutils.h"
#include <selforg/stl_adds.h>

#include <iostream>
#include <osgDB/FileUtils>

using namespace std;

namespace lpzrobots {

  /**
     class for reading bounding shape description files (.bbox) and to create appropriate geoms
     File Format: Lines wise, every line stands for one primitive. Possible lines are:
     sphere radius (x,y,z)
     cylinder radius height (x,y,z) (alpha, beta, gamma)
     capsule radius height (x,y,z) (alpha, beta, gamma)
     box length width height (x,y,z) (alpha, beta, gamma)
     (x,y,z) is the position vector and (alpha, beta, gamma) are
     the rotation angles about x,y,z axis respectively
  */

  BoundingShape::BoundingShape(const std::string& filename, Mesh* parent)
    : filename(filename), active(false), parent(parent), attachedToParentBody(true) {
    parent->setBoundingShape(this);
  }

  BoundingShape::~BoundingShape(){
    // TODO: destroy created Primitives and Transforms
  }

  void BoundingShape::setPose(const osg::Matrix& pose) {
    // update only if not attached to parent Body
    if (attachedToParentBody)
      return;
    vector<osg::Matrix>::const_iterator poseIt = boundingPrimitivePoseList.begin();
    FOREACH(vector<Primitive*>, boundingPrimitiveList, primIt) {
      if (*primIt && poseIt!=boundingPrimitivePoseList.end()) {
        const osg::Matrix primPose = (*poseIt);
        (*primIt)->setPose(pose * primPose);
      }
      ++poseIt;
    }
  }

  bool BoundingShape::readBBoxFile(std::string& filename,
                                   const OdeHandle& _odeHandle, const OsgHandle& osgHandle,
                                   double scale, char mode){
    odeHandle = OdeHandle(_odeHandle);
    parentSpace = odeHandle.space;
    odeHandle.createNewSimpleSpace(parentSpace,true);
    std::string filenamepath = osgDB::findDataFile(filename);
    FILE* f = fopen(filenamepath.c_str(),"r");
    if(!f) return false;
    char buffer[128];
    float r,h,x,y,z,l,w;
    double a,b,c;
    while(fgets(buffer,128,f)){
      if(strstr(buffer,"sphere")==buffer){
        if(sscanf(buffer+7, "%g (%g,%g,%g)", &r, &x, &y, &z)==4){
          Sphere* s = new Sphere(r * scale);
          Primitive* Trans = new Transform(parent,s,osgRotate(a*M_PI/180.0f,b*M_PI/180.0f,c*M_PI/180.0f)
                                           *osg::Matrix::translate(scale*x,scale*y,scale*z));
          Trans->init(odeHandle, 0, osgHandle,mode);
          active=true;
        } else{ fprintf(stderr, "Syntax error : %s", buffer); }
      } else if(strstr(buffer,"capsule")==buffer){
        if(sscanf(buffer+7, "%g %g (%g,%g,%g) (%lg,%lg,%lg)", &r, &h, &x, &y, &z, &a, &b, &c)==8){
          Capsule* ca = new Capsule(r * scale,h * scale);
          Primitive* Trans = new Transform(parent,ca,osgRotate(a*M_PI/180.0f,b*M_PI/180.0f,c*M_PI/180.0f)
                                           *osg::Matrix::translate(scale*x,scale*y,scale*z));
          Trans->init(odeHandle, 0, osgHandle,mode);
          active=true;
        } else{ fprintf(stderr, "Syntax error : %s", buffer); }
      } else if(strstr(buffer,"cylinder")==buffer){
        if(sscanf(buffer+8, "%g %g (%g,%g,%g) (%lg,%lg,%lg)", &r, &h, &x, &y, &z, &a, &b, &c)==8){
          Cylinder* cy = new Cylinder(r * scale,h * scale);
          Primitive* Trans = new Transform(parent,cy,osgRotate(a*M_PI/180.0f,b*M_PI/180.0f,c*M_PI/180.0f)
                                           *osg::Matrix::translate(scale*x,scale*y,scale*z));
          Trans->init(odeHandle, 0, osgHandle,mode);
          active=true;
        } else{ fprintf(stderr, "Syntax error : %s", buffer); }
      } else if(strstr(buffer,"box")==buffer){
        if(sscanf(buffer+3, "%g %g %g (%g,%g,%g) (%lg,%lg,%lg)", &l, &w, &h, &x, &y, &z, &a, &b, &c)==9){
          Box* box = new Box(l * scale,w * scale, h * scale);
          Primitive* Trans = new Transform(parent,box,osgRotate(a*M_PI/180.0f,b*M_PI/180.0f,c*M_PI/180.0f)
                                           *osg::Matrix::translate(scale*x,scale*y,scale*z));
          Trans->init(odeHandle, 0, osgHandle,mode);
          active=true;
        } else{ fprintf(stderr, "Syntax error : %s", buffer); }
      }else {
        fprintf(stderr, "Unknown Line: %s", buffer);
      }
    }
    fclose(f);
    return true;
  }

  bool BoundingShape::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                           double scale, char mode){
    return readBBoxFile(filename, odeHandle, osgHandle, scale, mode);
  }


  bool BoundingShape::isActive(){
    return active;
  }

}
