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
 *   Revision 1.4  2005-11-09 13:29:21  fhesse
 *   GPL added
 *                                                                 *
 *                                                                         *
 ***************************************************************************/
#ifndef __OCTAPLAYGROUND_H
#define __OCTAPLAYGROUND_H


#include <stdio.h>
#include <math.h>
#include <vector>

#include "abstractobstacle.h"
#include <drawstuff/drawstuff.h>


class OctaPlayground : public AbstractObstacle {

  double radius, width, height;
  double base_x, base_y, base_z;

  vector<dGeomID> obst; //obstacles

  bool obstacle_exists;

  int number_elements;
  double angle;    
  double box_length;


public:
  
  OctaPlayground(const OdeHandle& odehandle, int numberCorners=8):
    AbstractObstacle::AbstractObstacle(odehandle){

    base_x=0.0;
    base_y=0.0;
    base_z=0.0;
    
    radius=7.0;
    width=0.2;
    height=0.5;
  
    obstacle_exists=false;
        
    number_elements=numberCorners;
    angle= 2*M_PI/number_elements;    
    obst.resize(number_elements);    
    
    calcBoxLength();
    setColor(226 / 255.0, 103 / 255.0, 66 / 255.0);
  };
  
  virtual ~OctaPlayground(){
  }

  /**
   * draws the obstacle (4 boxes for the playground)
   */
  virtual void draw(){
    double box[3];
    //dsSetTexture (DS_WOOD);    
    dsSetColor (color.r, color.g, color.b);
    box[0] = width; box[1] = box_length; box[2] = height;    

    for(int i=0; i<number_elements; i++){
      dsDrawBox ( dGeomGetPosition ( obst[i] ) , dGeomGetRotation ( obst[i] ) , box );
    }
  };
  
  
  virtual void setPosition(double x, double y, double z){
    base_x=x;
    base_y=y;
    base_z=z;
    if (obstacle_exists){
      destroy();
    }
    create();
  };

  virtual void getPosition(double& x, double& y, double& z){
    x=base_x;
    y=base_y;
    z=base_z;
  }
  
  virtual void setGeometry(double radius_, double width_, double height_){
    radius=radius_;
    width=width_;
    height =height_;
    calcBoxLength();  
  };

  virtual void setColor(double r, double g, double b){
    color.r=r;
    color.g=g;
    color.b=b;
  };

protected:
  virtual void create(){
    // radius for positioning is smaller than radius since we use secants. 
    //  r is the smallest distance of the secant to the center of the circle.
    double r = sqrt(pow((1+cos(angle))/2, 2) + pow( sin(angle)/2 ,2)) * radius;
    for (int i=0; i<number_elements; i++){
      obst[i] = dCreateBox ( space, width , box_length , height);
      dMatrix3 R;
      dRFromEulerAngles(R, 0,0, i*angle);
      dGeomSetRotation ( obst[i], R);

      dGeomSetPosition ( obst[i], 
       			 base_x + cos(M_PI - i*angle) * r, 
 			 base_y + sin(M_PI - i*angle) * r, 
 			 height/2 +base_z);

    }
  };


  virtual void destroy(){
    for(int i=0; i<10; i++){
      dGeomDestroy( obst[i] );
    }
    obstacle_exists=false;
  };

  virtual void calcBoxLength(){
    double r = radius+width/2; 
    //    box_length =1.4 * sqrt( 2 * pow(radius,2) * (1 - cos(angle)) );
    box_length =  sqrt(pow( 1 - cos(angle), 2) + pow(sin(angle),2)) * r;  
  }

};

#endif
