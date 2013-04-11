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
#ifndef __PLAYGROUND_H
#define __PLAYGROUND_H

#include "mathutils.h"
#include "abstractground.h"
#include "primitive.h"

namespace lpzrobots {

  class Playground : public AbstractGround {

  protected:

    double length, width, height;
    double factorlength2;

  public:

    Playground(const OdeHandle& odeHandle, const OsgHandle& osgHandle ,
               const osg::Vec3& dimension = osg::Vec3(7.0, 0.2, 0.5) ,
               double factorxy = 1, bool createGround=true)
      : AbstractGround(odeHandle, osgHandle, createGround, dimension.x(), dimension.x()*factorxy, dimension.y()) {

      length=dimension.x();
      width=dimension.y();
      height=dimension.z();
      factorlength2=factorxy;
    };

    virtual void changeGeometry(double length, double width, double height, double factorxy){
      AbstractGround::changeGeometry(length, width, height, factorxy);
      this->length = length;
      this->width  = width;
      this->height  = height;
      this->factorlength2  = factorxy;
      if (obstacle_exists) {
        destroy();
        create();
      }
    }

  protected:
    virtual void create(){
      createGround();

      Box* box;
      osg::Vec3 offset(0,
                       (length/2 * factorlength2 + width/2),
                       height/2+0.01f/*reduces graphic errors and ode collisions*/);
      box = new Box( length + 2 * width , width, height);
      box->setTextures(getTextures(0));
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      box->setPose(osg::Matrix::translate(offset) * pose);
      obst.push_back(box);

      box = new Box( length + 2 * width , width, height);
      box->setTextures(getTextures(1));
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      box->setPose(osg::Matrix::translate(offset) * osg::Matrix::rotate(M_PI, 0,0,1) * pose);
      obst.push_back(box);

      osg::Vec3 offset2(0, (length/2 + width/2),
                       height/2+0.01f/*reduces graphic errors and ode collisions*/);
      box = new Box( length * factorlength2 , width, height);
      box->setTextures(getTextures(2));
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      box->setPose(osg::Matrix::translate(offset2) * osg::Matrix::rotate(M_PI/2.0, 0,0,1) * pose);
      obst.push_back(box);

      box = new Box( length * factorlength2 , width, height);
      box->setTextures(getTextures(3));
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      box->setPose(osg::Matrix::translate(offset2) * osg::Matrix::rotate(3.0*M_PI/2.0, 0,0,1)
                   * pose);
      obst.push_back(box);


      obstacle_exists=true;
    };

  };

}

#endif
