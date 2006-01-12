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
 *   Revision 1.1.2.6  2006-01-12 14:21:00  martius
 *   drawmode, material
 *
 *   Revision 1.1.2.5  2005/12/29 12:58:42  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.4  2005/12/15 17:03:43  martius
 *   cameramanupulator setPose is working
 *   joints have setter and getter parameters
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them
 *
 *   Revision 1.1.2.3  2005/12/14 15:36:45  martius
 *   joints are visible now
 *
 *   Revision 1.1.2.2  2005/12/13 18:11:14  martius
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
osg::Matrix osgPose( dGeomID geom );
/// returns the osg (4x4) pose matrix of the ode body
osg::Matrix osgPose( dBodyID body );
/// converts a position vector and a rotation matrix from ode to osg 4x4 matrix
osg::Matrix osgPose( const double * position , const double * rotation );
/// converts the rotation component of pose into an ode rotation matrix
void odeRotation( const osg::Matrix& pose , dMatrix3& odematrix);

/**************************************************************************/
class Primitive {
public:
  /** Body means that is is a dynamic object with a body.
      Geom means it has a geometrical represenation used for collision detection.
      Draw means the primitive is drawn
  */
  typedef enum Modes {Body=1, Geom=2, Draw=4};

  Primitive ();
  virtual ~Primitive ();
  /** registers primitive in ODE and OSG. 
      @param mass Mass of the object in ODE (if withBody = true)
      @param mode is a conjuction of Modes.
   */
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw)  = 0 ;

  /// should syncronise the ODE stuff and the OSG notes
  virtual void update() =0 ;

  virtual OSGPrimitive* getOSGPrimitive() = 0;

  void setPosition(const osg::Vec3& pos);
  void setPose(const osg::Matrix& pose);
  osg::Vec3 getPosition() const;
  osg::Matrix getPose() const;

  dGeomID getGeom() const;    
  dBodyID getBody() const;

protected:
  dGeomID geom;
  dBodyID body;
  char mode;
};


/**************************************************************************/
class Plane : public Primitive {
public:
  Plane();
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;

  virtual void update();  
  virtual OSGPrimitive* getOSGPrimitive() { return osgplane; }

protected:
  OSGPlane* osgplane;
};


/**************************************************************************/
class Box : public Primitive {
public:

  Box(float lengthX, float lengthY, float lengthZ);
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive() { return osgbox; }

protected:
  OSGBox* osgbox;
};


/**************************************************************************/
class Sphere : public Primitive {
public:
  Sphere(float radius);
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive() { return osgsphere; }

protected:
  OSGSphere* osgsphere;
};

/**************************************************************************/
class Capsule : public Primitive {
public:
  Capsule(float radius, float height);
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive() { return osgcapsule; }

protected:
  OSGCapsule* osgcapsule;
};

/**************************************************************************/

/**
   Primitive for transforming a geom in respect to a body. 
   The ODE geom is a TransformGeom. 
*/
class Transform : public Primitive {
public:
  /** 
      @param parent primitive should have a body and should be initialised
      @param child  is transformed by pose in respect to parent. 
      This Primitive must NOT have a body
  */
  Transform(Primitive* parent, Primitive* child, const osg::Matrix& pose);
  /// withBody MUST be false!
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw);

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive() { return 0; }

protected:
  Primitive* parent;
  Primitive* child;
  osg::Matrix pose;
};


}
#endif

