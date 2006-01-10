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
 *   Revision 1.10.4.2  2006-01-10 17:17:33  martius
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


#include <stdio.h>
#include <math.h>

#include "primitive.h"
#include "abstractobstacle.h"
 
namespace lpzrobots {

//Fixme: playground creates collisions with ground and itself
class Playground : public AbstractObstacle {

  double length, width, height;
  osg::Vec3 pos;
  double factorlength2;

  Box* box[4];

  bool obstacle_exists;

public:
  
  Playground(const OdeHandle& odeHandle, const OsgHandle& osgHandle , 
	     const osg::Vec3& dimension = osg::Vec3(7.0, 0.2, 0.5) , double factorxy = 1):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle){

    length=dimension.x();
    width=dimension.y();
    height=dimension.z();

    factorlength2=factorxy;

    obstacle_exists=false;
    
    setColor(Color(226 / 255.0, 103 / 255.0, 66 / 255.0));
  };

  /**
   * updates the position of the geoms  ( not nessary for static objects)
   */
  virtual void update(){
    //for(int i=0; i<4; i++){
    //      if(box[i]) box[i]->update();
    //    }    
  };
  
  
  virtual void setPosition(const osg::Vec3& pos){
    this->pos = pos;
    if (obstacle_exists){
      destroy();
    }
    create();
  };

  virtual osg::Vec3 getPosition(){
    return pos;
  }
  

 protected:
  virtual void create(){
    osg::Vec3 offset(- (length/2 + width/2), 0, height/2);
    box[0] = new Box( width , (length * factorlength2) + 2 * width , height);
    box[0]->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
    box[0]->setPosition(pos + offset);

    offset.x() = length/2 + width/2;
    box[1] = new Box( width , (length * factorlength2) + 2 * width , height);
    box[1]->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
    box[1]->setPosition(pos + offset);

    offset.x() = 0;
    offset.y() = -( (length*factorlength2)/2 +width/2);
    box[2] = new Box( length, width, height);
    box[2]->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
    box[2]->setPosition(pos + offset);

    offset.y() = (length*factorlength2)/2 +width/2;
    box[3] = new Box( length, width, height);
    box[3]->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
    box[3]->setPosition(pos + offset);
    
    obstacle_exists=true;
  };


  virtual void destroy(){
    for(int i=0; i<4; i++){
      if(box[i]) delete(box[i]);
    }
    
    obstacle_exists=false;
  };

};

}

#endif
