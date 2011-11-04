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

#include "randomobstacles.h"
#include "primitive.h"
#include "pos.h"

#include <cmath>
#include <selforg/controller_misc.h>


namespace lpzrobots {

   
  RandomObstacles::RandomObstacles(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                                   const RandomObstaclesConf& conf)
    : AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), conf(conf) {       
    index=0;
    obstacle_exists = true;
    pose = conf.pose;
  };

  void RandomObstacles::setPose(const osg::Matrix& pose){
    this->pose = pose;      
  };


  Primitive* RandomObstacles::getMainPrimitive() const { 
    if(!obst.empty()) 
      return obst[0]; 
    else 
      return 0; 
  }

  
  void RandomObstacles::remove(bool all){
    if(all)
      obst.clear();
    else{
      if(obst.size()>0){
        delete obst.front();
        obst.erase(obst.begin());
      }
    }
  }

  void RandomObstacles::spawn(OType type , SType subtype){
    OdeHandle handle2 = odeHandle;
    OsgHandle osgHandle2;
    
    double density=((double)rand()/RAND_MAX)*(conf.maxDensity - conf.minDensity) + conf.minDensity;  
    if(subtype == SRandom){
      subtype = (SType)(rand()%4);
    }      
    switch (subtype){
    case Metal: 
      handle2.substance.toMetal(1);
      osgHandle2 = osgHandle.changeColor(Color(0.5,0.5,0.5));
      density = conf.maxDensity;
      break;
    case Plastic: 
      handle2.substance.toPlastic(1);
      osgHandle2 = osgHandle.changeColor(Color(1,1,1));
      break;
    case Rubber: 
      handle2.substance.toRubber(10); 
      osgHandle2 = osgHandle.changeColor(Color(0.2,0.2,0.2));
      break;
    case Foam: 
    default: 
      handle2.substance.toFoam(15);
      osgHandle2 = osgHandle.changeColor(Color(1,1,0));
      density = conf.minDensity;
      break;
    }
      
      
    Pos dim((double)rand() / RAND_MAX, 
            (double)rand() / RAND_MAX, 
            (double)rand() / RAND_MAX);
    dim = (dim & (conf.maxSize - conf.minSize)) + conf.minSize;

    if(type == ORandom) {
      int l = conf.boxRelFreq + conf.sphereRelFreq + conf.capRelFreq;
      int r = rand()%l;
      if(r<conf.boxRelFreq) type = RandomObstacles::Box;
      else if (r<conf.boxRelFreq + conf.sphereRelFreq) type = RandomObstacles::Sphere;
      else type = RandomObstacles::Caps;                               
    }
  
    Primitive* o;
    switch (type){
    case RandomObstacles::Box:
      o = new lpzrobots::Box(dim.x(), dim.y(), dim.z());
      o->init(handle2, dim.x()* dim.y()* dim.z() * density, osgHandle2);
      break;
    case RandomObstacles::Sphere:
      o = new lpzrobots::Sphere(dim.x()/2.0);      
      o->init(handle2, 2.0/3.0*M_PI*pow(dim.x(),3)*density , osgHandle2);
      break;        
    case RandomObstacles::Caps:
    default:
      o = new lpzrobots::Capsule(dim.x()/2.0, dim.z()/2.0); 
      o->init(handle2, M_PI*sqr(dim.x())*dim.z()/8*density, osgHandle2);
      break;
    }

    Pos pos(random_minusone_to_one(0), random_minusone_to_one(0), 1);
    pos = (pos) & conf.area;
    pos.z() += (index%3) * conf.area.z()/2;
    index++;
    o->setPosition(pos * pose);
    obst.push_back(o);
  };

  
}
