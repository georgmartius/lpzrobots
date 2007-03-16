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
 *   Revision 1.4  2007-03-16 10:55:44  martius
 *   substance added
 *   ignoredSpaces and ignoredPairs
 *
 *   Revision 1.3  2006/07/14 12:23:56  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.2.4.1  2005/12/06 10:13:26  martius
 *   openscenegraph integration started
 *
 *   Revision 1.2  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef __ODEHANDLE_H
#define __ODEHANDLE_H

#include <ext/hash_set>
#include <ode/common.h>
#include "substance.h"

namespace lpzrobots {

  class Primitive;

struct geomPairHash{
  size_t operator() (const std::pair<long, long>& p) const {
    return  p.first + p.second;
  }
};

/** Data structure for accessing the ODE */
class OdeHandle
{
public:
  OdeHandle( ) { }
  OdeHandle(  dWorldID _world, dSpaceID _space, dJointGroupID _jointGroup);
  dWorldID world;
  dSpaceID space;
  dJointGroupID jointGroup;

  Substance substance;

  /// creates world at global space and so on.
  void init();

  /** sets of ignored geom pairs  and spaces
   adds a space to the list of ignored spaces for collision detection (i.e within this space there is no collision)
  */
  void addIgnoredSpace(dSpaceID g);
  /// removes a space from the list of ignored spaces for collision detection
  void removeIgnoredSpace(dSpaceID g);
  /// checks whether the space is an ignored space for collision detection
  bool isIgnoredSpace(dSpaceID g) const;
  
  /// adds a pair of geoms to the list of ignored geom pairs for collision detection
  void addIgnoredPair(dGeomID g1, dGeomID g2);
  /// like addIgnoredPair(dGeomID g1, dGeomID g2) just with primitives (provided for convinience)
  void addIgnoredPair(Primitive* p1, Primitive* p2);
  /// removes pair of geoms from the list of ignored geom pairs for collision detection
  void removeIgnoredPair(dGeomID g1, dGeomID g2);
  /// like removeIgnoredPair(dGeomID g1, dGeomID g2) just with primitives (provided for convinience)
  void removeIgnoredPair(Primitive* p1, Primitive* p2);
  /// checks whether a pair of geoms is an ignored pair for collision detection
  bool isIgnoredPair(dGeomID g1, dGeomID g2) const;

  /// set of ignored spaces
  __gnu_cxx::hash_set<long>* ignoredSpaces;
  /// set of ignored geom pairs for collision
  __gnu_cxx::hash_set<std::pair<long,long>, geomPairHash >* ignoredPairs;

};

}

#endif
