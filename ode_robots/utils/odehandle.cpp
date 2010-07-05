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
 *   Revision 1.12  2010-07-05 16:47:34  martius
 *   hashset transition to tr1
 *   new pid function for velocity servos, which work now fine
 *
 *   Revision 1.11  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.10  2010/03/05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *   Revision 1.9  2009/08/03 14:09:48  jhoffmann
 *   Remove some compiling warnings, memory leaks; Add some code cleanups
 *
 *   Revision 1.8  2009/07/30 08:55:21  jhoffmann
 *   Fix memory leak: delete osg viewer at exit
 *
 *   Revision 1.7  2008/04/17 15:59:02  martius
 *   OSG2 port finished
 *
 *   Revision 1.6.2.1  2008/04/15 16:21:53  martius
 *   Profiling
 *   Multithreading also for OSG and ODE but disables because of instabilities
 *
 *   Revision 1.6  2007/08/29 08:43:58  martius
 *   create simple space and delete space
 *
 *   Revision 1.5  2007/07/31 08:36:22  martius
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
#include <ode-dbl/ode.h>
#include <algorithm>
#include "primitive.h"

namespace lpzrobots
{

  OdeHandle::OdeHandle()
  {
    ignoredSpaces       = 0;
    ignoredPairs        = 0;
    spaces              = 0;
  }

  OdeHandle::OdeHandle(  dWorldID _world, dSpaceID _space, dJointGroupID _jointGroup )
  {
    world               = _world;
    space               = _space;
    jointGroup          = _jointGroup;
    ignoredSpaces       = 0;
    ignoredPairs        = 0;
    spaces              = 0;
  }

  void OdeHandle::destroySpaces()
  {
    if (spaces)
      delete spaces;

    if (ignoredSpaces)
      delete ignoredSpaces;

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
    ignoredSpaces = new HashSet<long>();
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
    if(ignore_inside_collisions) 
      addIgnoredSpace(space);
    else 
      addSpace(space);
  }

  void OdeHandle::deleteSpace(){
    if(isIgnoredSpace(space))
      removeIgnoredSpace(space);
    else
      removeSpace(space);
    dSpaceDestroy(space);
  }

  // sets of ignored geom pairs  and spaces
  // adds a space to the list of ignored spaces for collision detection (i.e within this space there is no collision)
  void OdeHandle::addIgnoredSpace(dSpaceID g)
  {
    if(ignoredSpaces)
      ignoredSpaces->insert((long)g);
  }
  // removes a space from the list of ignored spaces for collision detection
  void OdeHandle::removeIgnoredSpace(dSpaceID g)
  {
    if(ignoredSpaces)
      ignoredSpaces->erase((long)g);
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
    if(!spaces)
      return;

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
    if (!ignoredPairs)
      return;

      ignoredPairs->insert(std::pair<long, long>((long)g1,(long)g2));
      ignoredPairs->insert(std::pair<long, long>((long)g2,(long)g1));
  }
  // removes pair of geoms from the list of ignored geom pairs for collision detection
  void OdeHandle::removeIgnoredPair(dGeomID g1, dGeomID g2)
  {
    if (!ignoredPairs)
      return;

    ignoredPairs->erase(std::pair<long, long>((long)g1,(long)g2));
    ignoredPairs->erase(std::pair<long, long>((long)g2,(long)g1));
  }
  // adds a pair of Primitives to the list of ignored geom pairs for collision detection
  void OdeHandle::addIgnoredPair(Primitive* p1, Primitive* p2)
  {
    if (!ignoredPairs)
      return;

    ignoredPairs->insert(std::pair<long, long>((long)p1->getGeom(),(long)p2->getGeom()));
    ignoredPairs->insert(std::pair<long, long>((long)p2->getGeom(),(long)p1->getGeom()));
  }
  // removes pair of geoms from the list of ignored geom pairs for collision detection
  void OdeHandle::removeIgnoredPair(Primitive* p1, Primitive* p2)
  {
    if (!ignoredPairs)
      return;

    ignoredPairs->erase(std::pair<long, long>((long)p1->getGeom(),(long)p2->getGeom()));
    ignoredPairs->erase(std::pair<long, long>((long)p2->getGeom(),(long)p1->getGeom()));
  }

}
