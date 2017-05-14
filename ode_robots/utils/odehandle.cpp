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

#include "odehandle.h"
#include <assert.h>
#include <ode-dbl/ode.h>
#include <algorithm>
#include "primitive.h"

namespace lpzrobots
{

  OdeHandle::OdeHandle()
  {
    ignoredPairs        = 0;
    spaces              = 0;
  }

  OdeHandle::OdeHandle(  dWorldID _world, dSpaceID _space, dJointGroupID _jointGroup )
  {
    world               = _world;
    space               = _space;
    jointGroup          = _jointGroup;
    ignoredPairs        = 0;
    spaces              = 0;
  }

  void OdeHandle::destroySpaces()
  {
    if (spaces)
      delete spaces;

    if (ignoredPairs)
      delete ignoredPairs;
  }

  void OdeHandle::init(double* time)
  {
    assert(time);
    this->time=time;
    dInitODE();
    world = dWorldCreate ();

    // Create the primary world-space, which is used for collision detection
    space = dHashSpaceCreate (0);
    dSpaceSetCleanup (space, 0);
    spaces = new std::vector<dSpaceID>();
    // the jointGroup is used for collision handling,
    //  where a lot of joints are created every step
    jointGroup = dJointGroupCreate ( 1000000 );
    ignoredPairs  = new HashSet<std::pair<long,long>,geomPairHash >();

  }

  void OdeHandle::close(){
    dJointGroupDestroy  ( jointGroup );
    dWorldDestroy       ( world );
    dSpaceDestroy       ( space );
    destroySpaces();
    dCloseODE();
  }


  void OdeHandle::createNewSimpleSpace(dSpaceID parentspace, bool ignore_inside_collisions){
    space = dSimpleSpaceCreate (parentspace);
    dSpaceSetCleanup (space, 0);
    if(!ignore_inside_collisions)
      addSpace(space);
  }

  void OdeHandle::createNewHashSpace(dSpaceID parentspace, bool ignore_inside_collisions){
    space = dHashSpaceCreate (parentspace);
    dSpaceSetCleanup (space, 0);
    if(!ignore_inside_collisions)
      addSpace(space);
  }

  void OdeHandle::deleteSpace(){
    removeSpace(space);
    dSpaceDestroy(space);
  }


  // adds a space to the list of spaces for collision detection (ignored spaces do not need to be insered)
  void OdeHandle::addSpace(dSpaceID g)
  {
    if(spaces)
      spaces->push_back(g);
  }

  // removes a space from the list of ignored spaces for collision detection
  void OdeHandle::removeSpace(dSpaceID g)
  {
    if(!spaces) return;

    std::vector<dSpaceID>::iterator i = std::find(spaces->begin(), spaces->end(),g);
    if(i!=spaces->end()){
      spaces->erase(i);
    }
  }

  // returns list of all spaces
  const std::vector<dSpaceID>& OdeHandle::getSpaces()
  {
    return *spaces;
  }


  // adds a pair of geoms to the list of ignored geom pairs for collision detection
  void OdeHandle::addIgnoredPair(dGeomID g1, dGeomID g2)
  {
    if (!ignoredPairs) return;

    ignoredPairs->insert(std::pair<long, long>((long)g1,(long)g2));
    ignoredPairs->insert(std::pair<long, long>((long)g2,(long)g1));
  }
  // removes pair of geoms from the list of ignored geom pairs for collision detection
  void OdeHandle::removeIgnoredPair(dGeomID g1, dGeomID g2)
  {
    if (!ignoredPairs)  return;
    if(isIgnoredPair(g1,g2)){
      ignoredPairs->erase(std::pair<long, long>((long)g1,(long)g2));
      ignoredPairs->erase(std::pair<long, long>((long)g2,(long)g1));
    }
  }
  // adds a pair of Primitives to the list of ignored geom pairs for collision detection
  void OdeHandle::addIgnoredPair(Primitive* p1, Primitive* p2)
  {
    if (!ignoredPairs)  return;
    if(!p1->getGeom() || !p2->getGeom()) return;
    addIgnoredPair(p1->getGeom(), p2->getGeom());
  }
  // removes pair of geoms from the list of ignored geom pairs for collision detection
  void OdeHandle::removeIgnoredPair(Primitive* p1, Primitive* p2)
  {
    if (!ignoredPairs) return;
    if(!p1->getGeom() || !p2->getGeom()) return;
    removeIgnoredPair(p1->getGeom(),p2->getGeom());
  }

}
