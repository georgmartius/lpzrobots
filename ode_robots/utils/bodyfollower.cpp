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
 *   Revision 1.1.2.2  2006-01-12 15:11:02  martius
 *   moved to osg, but untested
 *
 *   Revision 1.1.2.1  2005/12/16 16:22:57  fhesse
 *   draws a line connecting the last n positions of a body
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include "bodyfollower.h" 
#include <assert.h>

namespace lpzrobots {

  void BodyFollower::init(const OsgHandle& osgHandle, int horizon, Primitive* body_to_follow)
  {
    this->osgHandle = osgHandle;
    this->horizon = horizon;
    this->body = body_to_follow;
    //  std::cout<<"1 \n";
    segments = (OSGPrimitive**) malloc(sizeof(OSGPrimitive*) * horizon);
    for (int i=0; i<horizon; i++){
      segments[i]=0;
    }
    counter=0;
    initialised=true;
  }

  void BodyFollower::update(){
    assert(initialised);
    osg::Vec3 p = body->getPosition();
    if(counter==0 || (p - lastpos).length2() > 0.5  ) {
      double len = (p - lastpos).length();
      if(segments[counter%horizon]) delete segments[counter%horizon];
      OSGPrimitive* s = new OSGCylinder(0.01, len);
      s->init(osgHandle, OSGPrimitive::Low);
      s->setMatrix(osg::Matrix::rotate(osg::Vec3(0,0,1), (p - lastpos)) * 
		   osg::Matrix::translate((p - lastpos)/2));
      segments[counter%horizon] = s;
      
      counter=counter+1;
      lastpos = p;
    }
  }

  BodyFollower::~BodyFollower(){
    for (int i=0; i<horizon; i++)
      if(segments[i]) delete(segments[i]);
    delete segments;
  }

}
