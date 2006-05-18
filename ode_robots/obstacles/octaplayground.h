/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   Revision 1.4.4.8  2006-05-18 14:38:28  robot3
 *   wall uses wall texture now
 *
 *   Revision 1.4.4.7  2006/05/18 12:54:24  robot3
 *   -fixed not being able to change the color after positioning
 *    the obstacle
 *   -cleared the files up
 *
 *   Revision 1.4.4.6  2006/05/18 12:00:57  robot3
 *   removed unused variables
 *
 *   Revision 1.4.4.5  2006/05/18 09:40:03  robot3
 *   using existing texture image in cvs for the groundplane now
 *
 *   Revision 1.4.4.4  2006/05/18 07:42:36  robot3
 *   Grounds have now a groundPlane for shadowing issues
 *   osgprimitive.cpp contains a bug that repeating textures (wrapping)
 *   don't work, needs to be fixed
 *
 *   Revision 1.4.4.3  2006/05/11 08:59:15  robot3
 *   -fixed a positioning bug (e.g. for passivesphere)
 *   -some methods moved to abstractobstacle.h for avoiding inconsistencies
 *
 *   Revision 1.4.4.2  2006/03/29 15:04:39  martius
 *   have pose now
 *
 *   Revision 1.4.4.1  2006/01/10 20:11:12  martius
 *   moved to osg
 *
 *   Revision 1.4  2005/11/09 13:29:21  fhesse
 *   GPL added
 *                                                                 *
 *                                                                         *
 ***************************************************************************/
#ifndef __OCTAPLAYGROUND_H
#define __OCTAPLAYGROUND_H

#include <math.h>
#include <vector>
#include <osg/Matrix>

#include "primitive.h"
#include "abstractobstacle.h"

namespace lpzrobots {

class OctaPlayground : public AbstractObstacle {

  double radius, width, height;

  vector<Primitive*> obst; //obstacles
  Box* groundPlane; // the groundplane


  int number_elements;
  double angle;    
  double box_length;

public:
  
  OctaPlayground(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
		 const Pos& geometry = Pos(7,0.2,0.5), int numberCorners=8):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle){
    
    radius = geometry.x();
    width  = geometry.y();
    height = geometry.z();

    obstacle_exists=false;
        
    number_elements=numberCorners;
    angle= 2*M_PI/number_elements;    
    obst.resize(number_elements);
    
    calcBoxLength();
  };
  
  virtual ~OctaPlayground(){
    destroy();
    obst.clear();
  }

  virtual void update(){
    for (unsigned int i=0; i<obst.size(); i++){
      if(obst[i]) obst[i]->update();
    }
    if (groundPlane) groundPlane->update();
  };
  

  virtual void setPose(const osg::Matrix& pose){
    this->pose = pose;
    if (obstacle_exists){
      destroy();
    }
    create();
  };

protected:

  virtual void create(){
    // radius for positioning is smaller than radius since we use secants. 
    //  r is the smallest distance of the secant to the center of the circle.
    double r = sqrt(pow((1+cos(angle))/2, 2) + pow( sin(angle)/2 ,2)) * radius;
    for (int i=0; i<number_elements; i++){
      obst[i] = new Box(width , box_length , height);
      obst[i]->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      osg::Matrix R = osg::Matrix::rotate(- i*angle, 0,0,1) * 
	osg::Matrix::translate( cos(M_PI - i*angle) * r, 
				sin(M_PI - i*angle) * r, 
				height/2) * pose;
      obst[i]->setPose(R);
      obst[i]->getOSGPrimitive()->setTexture("Images/wall.rgb");
    }
    // now create the plane in the middle
    groundPlane = new Box(2.0f*r,2.0f*r, 0.1f);
    groundPlane->init(odeHandle, 0, osgHandle,
		 Primitive::Geom | Primitive::Draw);
    groundPlane->setPose(osg::Matrix::translate(0.0f,0.0f,-0.05f) * pose);
    groundPlane->getOSGPrimitive()->setColor(Color(1.0f,1.0f,1.0f));
    groundPlane->getOSGPrimitive()->setTexture("Images/greenground.rgb",true,true);
    obstacle_exists=true;
  };


  virtual void destroy(){
    for(int i=0; i < number_elements; i++){
      if(obst[i]) delete obst[i];
    }
    if (groundPlane) delete(groundPlane);
    obstacle_exists=false;
  };

  virtual void calcBoxLength(){
    double r = radius+width/2; 
    //    box_length =1.4 * sqrt( 2 * pow(radius,2) * (1 - cos(angle)) );
    box_length =  sqrt(pow( 1 - cos(angle), 2) + pow(sin(angle),2)) * r;  
  }

};

}

#endif
