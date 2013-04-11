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
#ifndef __OCTAPLAYGROUND_H
#define __OCTAPLAYGROUND_H

#include "abstractground.h"

namespace lpzrobots {

  class OctaPlayground : public AbstractGround {


  protected:
    double radius, width, height;

    int number_elements;
    double angle;
    double box_length;

  public:


    OctaPlayground(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                 const Pos& geometry = Pos(7,0.2,0.5), int numberCorners=8, bool createGround=true):
    AbstractGround::AbstractGround(odeHandle, osgHandle,createGround,2*geometry.x(),2*geometry.x(), geometry.y()) {
    radius = geometry.x();
    width  = geometry.y();
    height = geometry.z();

    number_elements=numberCorners;
    angle= 2*M_PI/number_elements;
    //    obst.resize(number_elements);

    calcBoxLength();
  };

protected:

  virtual void create(){
    createGround();

    // radius for positioning is smaller than radius since we use secants.
    //  r is the smallest distance of the secant to the center of the circle.
    double r = sqrt(pow((1+cos(angle))/2, 2) + pow( sin(angle)/2 ,2)) * radius;
    for (int i=0; i<number_elements; i++){
      Box* box =  new Box(width , box_length , height);
      box->setTextures(getTextures(i));
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      osg::Matrix R = osg::Matrix::rotate(- i*angle, 0,0,1) *
        osg::Matrix::translate( cos(M_PI - i*angle) * r,
                                sin(M_PI - i*angle) * r,
                                height/2+0.01f /*reduces graphic errors and ode collisions*/
                                )* pose;
      box->setPose(R);
      obst.push_back(box);
    }
    obstacle_exists=true;
  };

  virtual void calcBoxLength(){
    double r = radius+width/2;
    //    box_length =1.4 * sqrt( 2 * pow(radius,2) * (1 - cos(angle)) );
    box_length =  sqrt(pow( 1 - cos(angle), 2) + pow(sin(angle),2)) * r;
  }

};

}

#endif
