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
 *   Revision 1.5  2005-10-25 19:26:57  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.4  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.3  2005/07/31 22:30:56  martius
 *   textures
 *
 *   Revision 1.2  2005/07/18 14:52:33  martius
 *   world and space are not pointers anymore.
 *
 *   Revision 1.1  2005/07/08 10:00:33  fhesse
 *   initial version
 *                                                    *
 *                                                                         *
 ***************************************************************************/
#ifndef __SPHERE_H
#define __SPHERE_H


#include <stdio.h>
#include <math.h>
#include <drawstuff/drawstuff.h>

#include "abstractobstacle.h"

/**
 *  (Passive) sphere as obstacle
 */
class Sphere : public AbstractObstacle{
  double radius;
  double masse;
  /**
   * initial coordinates
   */
  double base_x, base_y, base_z;
  int texture;

  dGeomID geom;
  dBodyID body;

  bool obstacle_exists;

 public:
  
  /**
   * Constructor
   * @param odehandle containing world, space and jointgroup for sphere
   */
  Sphere(const OdeHandle& odehandle):
    AbstractObstacle::AbstractObstacle(odehandle){

    base_x=0.0;
    base_y=0.0;
    base_z=0.0;
	
    radius=0.3;

    masse=1;
 
    obstacle_exists=false;
    
    setColor(0,0,1);
    texture = DS_NONE;

  };

  /**
   * draws the obstacle sphere
   */
  virtual void draw(){
    dsSetTexture (texture);    
    dsSetColor (color.r, color.g, color.b);
    dsDrawSphere(dBodyGetPosition(body),dBodyGetRotation(body),radius );
  };
  
  /**
   * sets position of the sphere and creates/recreates it if necessary
   */
  virtual void setPosition(double x, double y, double z){
    base_x=x;
    base_y=y;
    base_z=z;
    if (obstacle_exists){
      destroy();
    }
    create();
  };

  /**
   * gives actual position of sphere
   */
  virtual void getPosition(double& x, double& y, double& z){
    x=base_x;
    y=base_y;
    z=base_z;
  }
  
  virtual void setGeometry(double length_, double width_, double height_){
    radius=length_;
  };

  virtual void setColor(double r, double g, double b){
    color.r=r;
    color.g=g;
    color.b=b;
  };

  virtual void setTexture(int texture){
    this->texture = texture;
  }

 protected:
  virtual void create(){
    dMass m;


    body= dBodyCreate(world);
    dMassSetSphere(&m,1,radius);
    dMassAdjust (&m,masse);
    dBodySetMass (body,&m);
    dBodySetPosition ( body, base_x, base_y, base_z + radius);
    printf("z-position=%f\n",base_z);
    geom = dCreateSphere ( space, radius );
    dGeomSetBody (geom, body);

    obstacle_exists=true;
  };


  virtual void destroy(){
    dGeomDestroy( geom );
    dBodyDestroy( body );
    
    obstacle_exists=false;
  };

};

#endif
