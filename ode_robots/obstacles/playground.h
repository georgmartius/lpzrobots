#ifndef __PLAYGROUND_H
#define __PLAYGROUND_H


#include <stdio.h>
#include <math.h>
#include <drawstuff/drawstuff.h>

#include "abstractobstacle.h"

class Playground : public AbstractObstacle{

  double length, width, height;
  double base_x, base_y, base_z;

  dGeomID obst1; //Obstacle1
  dGeomID obst2; //Obstacle2
  dGeomID obst3; //Obstacle3
  dGeomID obst4; //Obstacle4

  bool obstacle_exists;

 public:
  
  Playground(dWorldID *w, dSpaceID *s):
    AbstractObstacle::AbstractObstacle(w, s){

    base_x=0.0;
    base_y=0.0;
    base_z=0.0;
	
    length=7.0;
    width=0.2;
    height=0.5;

    obstacle_exists=false;
    
    setColor(0,1,0);

  };

  /**
   * draws the obstacle (4 boxes for the playground)
   */
  virtual void draw(){
    double box[3];
    //dsSetTexture (DS_WOOD);    
    dsSetColor (color.r, color.g, color.b);

    box[0] = width; box[1] = length; box[2] = height;
    dsDrawBox ( dGeomGetPosition ( obst1 ) , dGeomGetRotation ( obst1 ) , box );
    dsDrawBox ( dGeomGetPosition ( obst2 ) , dGeomGetRotation ( obst2 ) , box );
    box[0] = length; box[1] = width; box[2] = height;
    dsDrawBox ( dGeomGetPosition ( obst3 ) , dGeomGetRotation ( obst3 ) , box );
    dsDrawBox ( dGeomGetPosition ( obst4 ) , dGeomGetRotation ( obst4 ) , box );
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
  
  virtual void setGeometry(double length_, double width_, double height_){
    length=length_;
    width=width_;
    height =height_;
  };

  virtual void setColor(double r, double g, double b){
    color.r=r;
    color.g=g;
    color.b=b;
  };

 protected:
  virtual void create(){
    obst1 = dCreateBox ( *space, width , length , height );
    dGeomSetPosition ( obst1, base_x-(length/2 + width/2), base_y, height/2 +base_z);
	
    obst2 = dCreateBox ( *space, width, length, height );
    dGeomSetPosition ( obst2, base_x+(length/2 +width/2), base_y, height/2 +base_z);
	
    obst3 = dCreateBox ( *space, length, width, height );
    dGeomSetPosition ( obst3, base_x, base_y-(length/2 +width/2), height/2 +base_z);
	
    obst4 = dCreateBox ( *space, length, width, height );
    dGeomSetPosition ( obst4, base_x, base_y+(length/2 +width/2), height/2 +base_z);

    obstacle_exists=true;
  };


  virtual void destroy(){
    dGeomDestroy( obst1 );
    dGeomDestroy( obst2 );
    dGeomDestroy( obst3 );
    dGeomDestroy( obst4 );
    
    obstacle_exists=false;
  };

};

#endif
