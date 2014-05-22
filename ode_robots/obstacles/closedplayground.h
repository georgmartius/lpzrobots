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
#ifndef __CLOSEDPLAYGROUND_H
#define __CLOSEDPLAYGROUND_H


#include "playground.h"

namespace lpzrobots {

  class ClosedPlayground : public Playground {

  protected:
    Box* roof;

  public:

    ClosedPlayground(const OdeHandle& odeHandle, const OsgHandle& osgHandle ,
                     const osg::Vec3& dimension = osg::Vec3(7.0, 0.2, 0.5) , double factorxy = 1)
      : Playground(odeHandle, osgHandle, dimension, factorxy), roof(0){
    };


  protected:
    virtual void create(){
      Playground::create();
      roof = new Box(length + 2 * width , (length * factorlength2) + 2 * width , width);
      roof->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);

      roof->setPosition(getPos() + osg::Vec3(0,0,height+width/2));
      obst.push_back(roof);
    };


    virtual void destroy(){
      Playground::destroy();
    }
  };

}

#endif
