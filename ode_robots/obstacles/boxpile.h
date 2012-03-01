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
#ifndef __BOXPILE_H
#define __BOXPILE_H

#include <stdio.h>
#include <cmath>

#include "primitive.h"
#include "joint.h"
#include "abstractobstacle.h"

namespace lpzrobots {

/**
 *  Boxpile
 */
class Boxpile : public AbstractObstacle {
  Pos dimension;
  int num;
  int seed;
  Pos boxsizemean;
  Pos boxsizevar;
  RandGen randGen;

public:
  /**
   * Constructor
   * a random boxpile is fixed to the world
   * @param dimension size of pile (height is always 1, use boxsize and boxvar)
   * @param num number of boxes
   * @param seed seed for random generator (it is fixed to generate deterministic piles)
   * @param boxsizemean mean size of boxes
   * @param boxsizevar  variance of boxes sizes
   */
  Boxpile(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
          const osg::Vec3& dimension = osg::Vec3(5.0, 5.0, 1.0), 
          int num = 30, int seed = 1,
          const osg::Vec3& boxsizemean = osg::Vec3(1.0, 1.0, 0.2), 
          const osg::Vec3& boxsizevar = osg::Vec3(0.5, 0.5, 0.1) ) 
    : AbstractObstacle::AbstractObstacle(odeHandle, osgHandle), dimension(dimension),
      num(num), boxsizemean(boxsizemean), boxsizevar(boxsizevar) 
  {
    setTexture("Images/wood_sw.jpg");
    randGen.init(seed);
    this->dimension.z()=1;
    obstacle_exists=false;    
  };

  
  virtual void setPose(const osg::Matrix& pose){
    this->pose = pose;
    if (!obstacle_exists) {
      create();
    }
  };

  virtual Primitive* getMainPrimitive() const { 
    if(!obst.empty()) return obst[0]; 
    else return 0;
  }
  
protected:
  virtual void create(){
    OdeHandle oh(odeHandle);
    oh.createNewSimpleSpace(odeHandle.space,true);
    double size=dimension.length();
    for(int i=0; i< num; i++){
      Box* b;
      Pos rand(randGen.rand()-0.5,randGen.rand()-0.5,randGen.rand()-0.5);
      Pos s = boxsizemean + ((rand*2) & boxsizevar); // & component wise mult
      Pos pos = (dimension & Pos(randGen.rand()-0.5,randGen.rand()-0.5,0));
      double angle = randGen.rand()*M_PI;

      // make sure box has positive dimensions
      s.x()=fabs(s.x());
      s.y()=fabs(s.y());
      s.z()=fabs(s.z());
      // make pile round
      s.z()*=fabs((size-pos.length())/size); // linear ramping of heights
      
      pos.z() = s.z()/2.0;
      b = new Box(s);
      b->setTextures(getTextures(i));
      b->init(oh, 0, osgHandle, Primitive::Geom | Primitive::Draw);

      b->setPose(ROTM(angle, 0,0,1)*TRANSM(pos) * pose);
      obst.push_back(b);
    }    
    obstacle_exists=true;
  };

};

}

#endif

