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

#include "plattfussschlange.h"
using namespace std;

namespace lpzrobots {

  PlattfussSchlange::
  PlattfussSchlange ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      const SchlangeConf& conf, const std::string& name)
    : SchlangeServo2(odeHandle, osgHandle, conf, name,
                     "$Id$")
  {

  }

  PlattfussSchlange::~PlattfussSchlange() { }


  Primitive* PlattfussSchlange::createSegment(int index, const OdeHandle& odeHandle){
    Primitive* p;

    /////////// MIDDLE SEGMENT (BODY)
    if ( index*2 == conf.segmNumber-1) {
      //p = new Box(conf.segmLength*1.5,conf.segmLength*1.5, conf.segmLength*.6);
      //p = new Capsule(conf.segmDia*.8/*2.8*/ , conf.segmLength*1);

       //  p = new Capsule(conf.segmLength/2 , conf.segmLength*2);
      p = new Sphere(conf.segmLength*.8);
      p->setTexture("Images/wood.rgb");
      p->init(odeHandle, conf.segmMass*2, osgHandle);
      // p->setPose( osg::Matrix::rotate(M_PI/2, 0, 1, 0)*osg::Matrix::translate( conf.segmDia, 0, 0) );
    } /////// FEED
    else if( (index == 0) | (index== conf.segmNumber-1)) {
      // p = new Capsule(conf.segmDia*.8/*2.8*/ , conf.segmLength*1);
       // p = new Sphere(conf.segmLength/2*2);
      p = new Box(1.8*conf.segmLength,3*conf.segmLength, conf.segmLength*.3);
      p->setTexture("Images/whitemetal_farbig_small.rgb");
      p->init(odeHandle, conf.segmMass*3, osgHandle);
    } /////// NORMAL SEGMENT
    else {
      p = SchlangeServo2::createSegment(index, odeHandle);
    }
    return p;
  }

}
