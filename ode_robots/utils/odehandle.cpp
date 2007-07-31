/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.5  2007-07-31 08:36:22  martius
 *   added a list of spaces for collision control within them
 *
 *   Revision 1.4  2007/07/03 13:04:54  martius
 *   pointer to global simulation time
 *   hash-maps are protected
 *
 *   Revision 1.3  2007/06/21 16:20:34  martius
 *   inlined isIgnoredSpace and Pair
 *
 *   Revision 1.2  2007/03/16 10:56:04  martius
 *   substance added
 *   ignoredSpaces and ignoredPairs
 *
 *                                                                 *
 ***************************************************************************/

#include "odehandle.h"
#include <assert.h>
#include <ode/ode.h>
#include "primitive.h"

namespace lpzrobots {

  OdeHandle::OdeHandle(  dWorldID _world, dSpaceID _space, dJointGroupID _jointGroup){
    world = _world; 
    space = _space; 
    jointGroup= _jointGroup;
    ignoredSpaces=0;
    ignoredPairs=0;    
  }

  void OdeHandle::init(double* time){
    assert(time);
    this->time=time;
    world = dWorldCreate ();
 
    // Create the primary world-space, which is used for collision detection
    space = dHashSpaceCreate (0);
    spaces = new std::list<dSpaceID>();
    // the jointGroup is used for collision handling, 
    //  where a lot of joints are created every step
    jointGroup = dJointGroupCreate ( 1000000 );
    ignoredSpaces = new __gnu_cxx::hash_set<long>();
    ignoredPairs  = new __gnu_cxx::hash_set<std::pair<long,long>,geomPairHash >();
  }

  // sets of ignored geom pairs  and spaces
  // adds a space to the list of ignored spaces for collision detection (i.e within this space there is no collision)
  void OdeHandle::addIgnoredSpace(dSpaceID g) { 
    ignoredSpaces->insert((long)g); 
  }
  // removes a space from the list of ignored spaces for collision detection
  void OdeHandle::removeIgnoredSpace(dSpaceID g) { 
    ignoredSpaces->erase((long)g);
  }
  

  // adds a space to the list of spaces for collision detection (ignored spaces do not need to be insered)
  void OdeHandle::addSpace(dSpaceID g){
    spaces->push_back(g);
  }

  // removes a space from the list of ignored spaces for collision detection
  void OdeHandle::removeSpace(dSpaceID g){
    std::list<dSpaceID>::iterator i = std::find(spaces->begin(), spaces->end(),g);
    if(i!=spaces->end()){
      spaces->erase(i);
    }
  }

  // returns list of all spaces
  const std::list<dSpaceID>& OdeHandle::getSpaces(){
    return *spaces;
  }


  // adds a pair of geoms to the list of ignored geom pairs for collision detection
  void OdeHandle::addIgnoredPair(dGeomID g1, dGeomID g2) { 
    ignoredPairs->insert(std::pair<long, long>((long)g1,(long)g2));
    ignoredPairs->insert(std::pair<long, long>((long)g2,(long)g1));
  }
  // removes pair of geoms from the list of ignored geom pairs for collision detection
  void OdeHandle::removeIgnoredPair(dGeomID g1, dGeomID g2) {
    ignoredPairs->erase(std::pair<long, long>((long)g1,(long)g2));
    ignoredPairs->erase(std::pair<long, long>((long)g2,(long)g1));
  }
  // adds a pair of Primitives to the list of ignored geom pairs for collision detection
  void OdeHandle::addIgnoredPair(Primitive* p1, Primitive* p2) { 
    ignoredPairs->insert(std::pair<long, long>((long)p1->getGeom(),(long)p2->getGeom()));
    ignoredPairs->insert(std::pair<long, long>((long)p2->getGeom(),(long)p1->getGeom()));
  }
  // removes pair of geoms from the list of ignored geom pairs for collision detection
  void OdeHandle::removeIgnoredPair(Primitive* p1, Primitive* p2) {
    ignoredPairs->erase(std::pair<long, long>((long)p1->getGeom(),(long)p2->getGeom()));
    ignoredPairs->erase(std::pair<long, long>((long)p2->getGeom(),(long)p1->getGeom()));
  }

}
