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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2006-07-14 12:23:33  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.4  2006/06/27 14:14:29  robot3
 *   -optimized mesh and boundingshape code
 *   -other changes
 *
 *   Revision 1.1.2.3  2006/06/26 21:52:38  robot3
 *   Mesh works now with bbox file
 *
 *   Revision 1.1.2.2  2006/06/23 08:54:53  robot3
 *   made some changes on primitive Mesh (including boundingshape)
 *
 *   Revision 1.1.2.1  2006/06/22 11:33:30  robot3
 *   moved boundingshape implementation to .cpp-file
 *
 *   Revision 1.1.2.3  2006/05/29 22:22:07  martius
 *   added std includes
 *
 *   Revision 1.1.2.2  2006/03/29 15:03:19  martius
 *   format documented
 *
 *   Revision 1.1.2.1  2006/03/29 14:51:45  martius
 *   class for reading bounding shape description files and creates the appropriate geoms
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/

#include "boundingshape.h"
#include "mathutils.h"


#include <iostream>

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

  BoundingShape::BoundingShape(const std::string& filename, Primitive* parent) : filename(filename), parent(parent) {
    active=false;
  }
  
  BoundingShape::~BoundingShape(){}

  bool BoundingShape::readBBoxFile(std::string& filename,
				   const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
				   double scale, char mode){
    // TODO: use search path (maybe try to use some osgDB function)
    FILE* f = fopen(filename.c_str(),"r");
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
