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
 *   Revision 1.1.2.2  2005-12-13 18:11:14  martius
 *   transform primitive added, some joints stuff done, forward declaration
 *
 *   Revision 1.1.2.1  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *                                                                 *
 *                                                                         *
 ***************************************************************************/
#ifndef __PRIMITIVE_H
#define __PRIMITIVE_H

#include "osgprimitive.h"
#include "odehandle.h"
#include <osg/Matrix>

#include <ode/common.h>

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
  /** registers primitive in ODE and OSG. 
      @param mass Mass of the object in ODE (if withBody = true)
      @param withBody true if there should be a dynamic body, 
                      false if the geom should be static
   */
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    bool withBody = true) = 0 ;

  /// should syncronise the ODE stuff and the OSG notes
  virtual void update() =0 ;
  /// returns a osg transformation object (if any)  (it is not const because we return a pointer)
  virtual osg::Transform* getTransform() = 0;

  void setPosition(const osg::Vec3& pos);
  void setPose(const osg::Matrix& pose);
  osg::Vec3 getPosition() const;
  osg::Matrix getPose() const;

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
  virtual osg::Transform* getTransform();
  
};


/**************************************************************************/
class Box : public Primitive, public OSGBox {
public:
  Box(float lengthX, float lengthY, float lengthZ);
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    bool withBody = true);

  virtual void update();
  virtual osg::Transform* getTransform();
  
};


/**************************************************************************/
class Sphere : public Primitive, public OSGSphere {
public:
  Sphere(float radius);
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    bool withBody = true);

  virtual void update();
  virtual osg::Transform* getTransform();

};

/**************************************************************************/
class Capsule : public Primitive, public OSGCapsule {
public:
  Capsule(float radius, float height);
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    bool withBody = true);

  virtual void update();
  virtual osg::Transform* getTransform();
};

/**************************************************************************/

/**
   Primitive for transforming a geom in respect to a body. 
   The ODE geom is a TransformGeom. 
*/
class Transform : public Primitive {
public:
  /** 
      @param parent primitive should have a body
      @param child  is transformed by pose in respect to parent. 
      This Primitive must NOT have a body
  */
  Transform(Primitive* parent, Primitive* child, const osg::Matrix& pose);
  /// withBody MUST be false!
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    bool withBody = true);

  virtual void update();
  // the funny thing is that we (as a Transform) don't have a transform :-)
  virtual osg::Transform* getTransform();
protected:
  Primitive* parent;
  Primitive* child;
  osg::Matrix pose;
};


}
#endif

