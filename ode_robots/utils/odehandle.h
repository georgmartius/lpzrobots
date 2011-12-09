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
#ifndef __ODEHANDLE_H
#define __ODEHANDLE_H

#include <selforg/stl_map.h>

#include <vector>
#include <ode-dbl/common.h>
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
  OdeHandle( );
  OdeHandle(  dWorldID _world, dSpaceID _space, dJointGroupID _jointGroup);

  dWorldID world;
  dSpaceID space;
  dJointGroupID jointGroup;

  Substance substance;

  /// creates world at global space and so on and sets global time pointer.
  void init(double* time); 

  /// deletes the world and global data
  void close(); 

  /** use this function to create a new space with optional ignored collisions, 
      use deleteSpace to destroy it
      
      All primitives initialised with this handle are within this space.
   */
  void createNewSimpleSpace(dSpaceID parentspace, bool ignore_inside_collisions);

  /** like createNewSimpleSpace but with a HashSpace. More efficient for large objects
   */
  void createNewHashSpace(dSpaceID parentspace, bool ignore_inside_collisions);

  /// destroys the space and unregisters them in the global lists
  void deleteSpace();

  /** adds a space to the list of spaces for collision detection (ignored spaces do not need to be insered)*/
  void addSpace(dSpaceID g);
  /// removes a space from the list of ignored spaces for collision detection
  void removeSpace(dSpaceID g);

  /** deletes all associated memory objects, handle with care - use only when program exits */
  void destroySpaces();

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
  //TODO: Destroy the internal containers (vectors, list, ...) within the destructor. Now destroySpaces is used. Maybe we can use QMP_CRITICAL for a ref_cnt-variabel, or avoid the pointers.

  /// list of spaces, except ignored spaces
  std::vector<dSpaceID>* spaces;

  /// set of ignored spaces
  HashSet<long>* ignoredSpaces;

  /// set of ignored geom pairs for collision
  HashSet<std::pair<long,long>, geomPairHash >* ignoredPairs;

};

}
#endif
