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
#ifndef __RANDOMOBSTACLES_H
#define __RANDOMOBSTACLES_H

#include "abstractobstacle.h"
#include "abstractground.h"
#include "pos.h"

namespace lpzrobots {

  struct RandomObstaclesConf {
    Pos area; ///< zero centered, use setPose to shift around (z() component is the height)
    osg::Matrix pose;
    Pos minSize;
    Pos maxSize;
    double minDensity;
    double maxDensity;
    int boxRelFreq;   
    int sphereRelFreq;
    int capRelFreq;
  };

  /**
   * Passive random obstacles:
   * with spawn and remove obstacles can be created and removed.
   * Add an instance to global.obstacles to customize the creation
   *  otherwise a default version is used (though dependend on the playground)
   */
  class RandomObstacles : public AbstractObstacle {
    
    int index;
    RandomObstaclesConf conf;
  public:
    enum OType {Box, Sphere, Caps, ORandom};
    enum SType {Metal, Plastic, Rubber, Foam, SRandom};
    
    /// creates a default configuration, optionally with the size and position of the ground
    static RandomObstaclesConf getDefaultConf(AbstractGround* ground = 0){
      RandomObstaclesConf c;
      if(ground){
        c.area  = Pos(ground->getGroundLength()/2, ground->getGroundWidth()/2, 5)*0.95;
        c.pose =  ground->getPose();
      }else{
        c.area    = Pos(10,10,4);
        c.pose    = osg::Matrix::translate(0,0,0);
      }
      c.minSize = Pos(.5,.5,.5);
      c.maxSize = Pos(2,2,2);
      c.minDensity=1;
      c.maxDensity=10;
      c.boxRelFreq=5;
      c.sphereRelFreq=1;
      c.capRelFreq=1;
      return c;
    }

    RandomObstacles(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                    const RandomObstaclesConf& conf = getDefaultConf());

  
    virtual void setPose(const osg::Matrix& pose);

    virtual Primitive* getMainPrimitive() const;

    virtual void create(){};

    virtual void remove(bool all = false);

    virtual void spawn(OType type = ORandom , SType subtype = SRandom);

  };

}

#endif



