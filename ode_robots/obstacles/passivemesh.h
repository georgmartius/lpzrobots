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
 *   Revision 1.1.2.5  2006-06-29 16:39:55  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.1.2.4  2006/06/27 14:14:29  robot3
 *   -optimized mesh and boundingshape code
 *   -other changes
 *
 *   Revision 1.1.2.3  2006/06/23 09:01:14  robot3
 *   made changes on primitive Mesh
 *
 *   Revision 1.1.2.2  2006/06/16 22:27:26  martius
 *   getMainPrimtive
 *
 *   Revision 1.1.2.1  2006/05/29 19:17:41  robot3
 *   first version
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __PASSIVEMESH_H
#define __PASSIVEMESH_H

#include <stdio.h>
#include <math.h>

#include "primitive.h"
#include "osgprimitive.h"
#include "abstractobstacle.h"

namespace lpzrobots {

/**
 *  (Passive) mesh as obstacle
 */
class PassiveMesh : public AbstractObstacle{
  std::string filename;
  float scale;
  double mass;

  Mesh* mesh;
  GlobalData globalData;


 public:
  
  /**
   * Constructor
   */
  PassiveMesh(const OdeHandle& odeHandle, const OsgHandle& osgHandle,const string& filename,
	     GlobalData& globalData, double scale = 1.0, double mass = 1.0):
    AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), 
    filename(filename), scale(scale), mass(mass), globalData(globalData) {       
    mesh=0;
    obstacle_exists=false;    
  };

  ~PassiveMesh(){
    if(mesh) delete mesh;
  }

  /**
   * update position of mesh
   */
  virtual void update(){
    if(mesh) mesh->update();
  };

/*   virtual void setTexture(const std::string& filename){ */
/*     if(mesh) mesh->getOSGPrimitive()->setTexture(filename); */
/*   } */
  
  virtual void setPose(const osg::Matrix& pose){
    this->pose = pose;
    if (obstacle_exists){
      destroy();
    }
    create();
  };


  virtual Primitive* getMainPrimitive() const { return mesh; }
  
 protected:

  bool drawBoundings;

  virtual void create(){
    mesh = new Mesh(filename,scale,globalData);
    mesh->init(odeHandle, mass, osgHandle);
    osg::Vec3 pos=pose.getTrans();
    pos[2]+=mesh->getRadius();
    mesh->setPosition(pos);
    obstacle_exists=true;
  };


  virtual void destroy(){
    if(mesh) delete mesh;    
    obstacle_exists=false;
  };

};

}

#endif
