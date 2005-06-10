#ifndef __ABSTRACTOBSTACLE_H
#define __ABSTRACTOBSTACLE_H

#include <ode/ode.h>



typedef struct
{
	double r;
	double g;
	double b;
} color;



/**
 *  Abstract class (interface) for obstacles
 */
class AbstractObstacle{

 public:
  /**
   * Constructor
   * @param w world in which obstacle should be created
   * @param s space in which obstacle should be created
   */
  AbstractObstacle(dWorldID *w, dSpaceID *s){
    world=w;
    space=s;
  };
  
  /**
   * draws the obstacle
   */
  virtual void draw() = 0;
  
  /**
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  virtual void setPosition(double x, double y, double z) = 0;

  /**
   * gives actual position of the obstacle
   */
  virtual void getPosition(double& x, double& y, double& z) = 0;
  
  /**
   * sets geometry parameters for the obstacle
   */
  virtual void setGeometry(double length, double width, double height) = 0;

  virtual void setColor(double r, double g, double b)=0;

 protected:

  dSpaceID *space;
  dWorldID *world;

  color obst_color;

};

#endif
