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
 ***************************************************************************
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.1  2006-01-12 14:22:49  martius
 *   invisibles
 *
 *                                                                 *
 *                                                                         *
 ***************************************************************************/

#include <assert.h>
#include <ode/ode.h>

#include "invisibleprimitive.h"

namespace lpzrobots{

  /******************************************************************************/
  InvisibleBox::InvisibleBox(float lengthX, float lengthY, float lengthZ) 
    : lengthX(lengthX), lengthY(lengthY), lengthZ(lengthZ){
  }

  void InvisibleBox::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		    char mode) {
    assert(mode & Body || mode & Geom);
    this->mode=mode;
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      // fake the mass of the plane with a thin box
      dMassSetBox(&m, 1, lengthX, lengthY, lengthZ);
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body      
    }  
    if (mode & Geom){    
      geom = dCreateBox ( odeHandle.space , lengthX, lengthY, lengthZ);
      if (mode & Body){
	dGeomSetBody (geom, body); // geom is assigned to body
      }
    }
  }


  /******************************************************************************/
  InvisibleSphere::InvisibleSphere(float radius) 
    : radius(radius)  {    
  }
  
  void InvisibleSphere::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		    char mode) {
    assert(mode & Body || mode & Geom);
    this->mode=mode;
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetSphere(&m, 1, radius);
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }  
    if (mode & Geom){
      geom = dCreateSphere ( odeHandle.space , radius);
      if (mode & Body)
	dGeomSetBody (geom, body); // geom is assigned to body
    }
    
  }
  
  /******************************************************************************/
  InvisibleCapsule::InvisibleCapsule(float radius, float height)
    : radius(radius), height(height) { 
  }
  
  void InvisibleCapsule::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		    char mode) {
    assert(mode & Body || mode & Geom);
    this->mode=mode;
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetCappedCylinder(&m, 1.0, 3 , radius, height); 
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }  
    if (mode & Geom){    
      geom = dCreateCCylinder ( odeHandle.space , radius, height); 
      if (mode & Body)
	dGeomSetBody (geom, body); // geom is assigned to body      
    }
  }

}
