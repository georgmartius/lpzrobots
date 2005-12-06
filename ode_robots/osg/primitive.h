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
 *   This file provides basic primitives for ODE and openscenegraph        *
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.1  2005-12-06 10:13:25  martius
 *   openscenegraph integration started
 *
 *                                                                 *
 *                                                                         *
 ***************************************************************************/
#ifndef __PRIMITIVE_H
#define __PRIMITIVE_H

#include "osgprimitive.h"
#include "odehandle.h"

#include <ode/ode.h>

namespace lpzrobots {


/// returns the osg (4x4) pose matrix of the ode geom
osg::Matrix odePose( dGeomID geom );
/// returns the osg (4x4) pose matrix of the ode body
osg::Matrix odePose( dBodyID body );
/// converts a position vector and a rotation matrix from ode to osg 4x4 matrix
osg::Matrix odePose( const double * position , const double * rotation );


/**************************************************************************/
class Primitive {
public:
  Primitive ();
  virtual ~Primitive ();
  /// To be overloaded
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    bool withBody = true) =0 ;

  /// should syncronise the Ode stuff and the OSG notes
  virtual void update() =0 ;

  void setPosition(const osg::Vec3& pos);
  void setPose(const osg::Matrix& pose);
  dGeomID getGeom() const;    
  dBodyID getBody() const;

protected:
  dGeomID geom;
  dBodyID body;
};


/**************************************************************************/
class Plane : public Primitive, public OSGPlane {
public:
  Plane();
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    bool withBody = true);

  virtual void update();
  
};


/**************************************************************************/
class Box : public Primitive, public OSGBox {
public:
  Box(float lengthX, float lengthY, float lengthZ);
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    bool withBody = true);

  virtual void update();
  
};


/**************************************************************************/
class Sphere : public Primitive, public OSGSphere {
public:
  Sphere(float radius);
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    bool withBody = true);

  virtual void update();

};

/**************************************************************************/
class Capsule : public Primitive, public OSGCapsule {
public:
  Capsule(float radius, float height);
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    bool withBody = true);

  virtual void update();
};

}
#endif

