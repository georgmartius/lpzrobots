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
 *   Revision 1.11  2008-08-27 06:46:12  martius
 *   hashcode optimised and comment added
 *
 *   Revision 1.10  2008/08/26 18:58:32  martius
 *   comments
 *
 *   Revision 1.9  2008/04/17 15:59:02  martius
 *   OSG2 port finished
 *
 *   Revision 1.8.2.1  2008/04/15 16:21:53  martius
 *   Profiling
 *   Multithreading also for OSG and ODE but disables because of instabilities
 *
 *   Revision 1.8  2007/08/29 08:43:50  martius
 *   create simple space and delete space
 *
 *   Revision 1.7  2007/07/31 08:37:03  martius
 *   added a list of spaces for collision control within them
 *
 *   Revision 1.6  2007/07/03 13:04:22  martius
 *   pointer to global simulation time
 *   hash-maps are protected
 *
 *   Revision 1.5  2007/06/21 16:20:26  martius
 *   inlined isIgnoredSpace and Pair
 *
 *   Revision 1.4  2007/03/16 10:55:44  martius
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
#include <vector>
#include <ode/common.h>
#include "substance.h"

namespace lpzrobots {

class Primitive;

struct geomPairHash{
  size_t operator() (const std::pair<long, long>& p) const {
    return  2*p.first + p.second;
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

  /// creates world at global space and so on and sets global time pointer.
  void init(double* time); 

  /** use this function to create a new space with optional ignored collisions, 
      use deleteSpace to destroy it
      
      All primitives initialised with this handle are within this space.
   */
  void createNewSimpleSpace(dSpaceID parentspace, bool ignore_inside_collisions);

  /// destroys the space and unregisters them in the global lists
  void deleteSpace();

  /** adds a space to the list of ignored spaces for collision detection 
    (i.e within this space there is no collision)
  */
  void addIgnoredSpace(dSpaceID g);
  /// removes a space from the list of ignored spaces for collision detection
  void removeIgnoredSpace(dSpaceID g);
  /// checks whether the space is an ignored space for collision detection
  inline bool isIgnoredSpace(dSpaceID g) const { 
    return ignoredSpaces->find((long)g) != ignoredSpaces->end(); 
  }

  /** adds a space to the list of spaces for collision detection (ignored spaces do not need to be insered)*/
  void addSpace(dSpaceID g);
  /// removes a space from the list of ignored spaces for collision detection
  void removeSpace(dSpaceID g);
  /// returns list of all spaces (as vector for parallelisation
  const std::vector<dSpaceID>& getSpaces();


  inline double getTime(){ return *time; }
  
  /// adds a pair of geoms to the list of ignored geom pairs for collision detection
  void addIgnoredPair(dGeomID g1, dGeomID g2);
  /// like addIgnoredPair(dGeomID g1, dGeomID g2) just with primitives (provided for convinience)
  void addIgnoredPair(Primitive* p1, Primitive* p2);
  /// removes pair of geoms from the list of ignored geom pairs for collision detection
  void removeIgnoredPair(dGeomID g1, dGeomID g2);
  /// like removeIgnoredPair(dGeomID g1, dGeomID g2) just with primitives (provided for convinience)
  void removeIgnoredPair(Primitive* p1, Primitive* p2);
  /// checks whether a pair of geoms is an ignored pair for collision detection
  inline bool isIgnoredPair(dGeomID g1, dGeomID g2) const { 
    return (ignoredPairs->find(std::pair<long, long>((long)g1,(long)g2)) != ignoredPairs->end())
      || (ignoredPairs->find(std::pair<long, long>((long)g2,(long)g1)) != ignoredPairs->end());
  }

protected:
  double* time;

  /// list of spaces, except ignored spaces
  std::vector<dSpaceID>* spaces;
  /// set of ignored spaces
  __gnu_cxx::hash_set<long>* ignoredSpaces;
  /// set of ignored geom pairs for collision
  __gnu_cxx::hash_set<std::pair<long,long>, geomPairHash >* ignoredPairs;


};

}

#endif
