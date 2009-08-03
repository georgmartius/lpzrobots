 
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
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 *   This file provides basic primitives for ODE and openscenegraph        *
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.17  2009-08-03 14:09:48  jhoffmann
 *   Remove some compiling warnings, memory leaks; Add some code cleanups
 *
 *   Revision 1.16  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.15  2008/09/11 15:24:01  martius
 *   motioncallback resurrected
 *   noContact substance
 *   use slider center of the connecting objects for slider drawing
 *
 *   Revision 1.14  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.13  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.12  2007/11/07 13:19:01  martius
 *   toLocal: coordinate transformation
 *
 *   Revision 1.11  2007/08/23 14:51:28  martius
 *   Ray
 *
 *   Revision 1.10  2007/07/31 08:21:34  martius
 *   OSGMesh does not need GlobalData
 *   drawBoundings moved to OsgHandle
 *
 *   Revision 1.9  2007/07/17 07:20:04  martius
 *   setMass added
 *
 *   Revision 1.8  2007/07/03 13:12:52  martius
 *   limitLinearVel
 *
 *   Revision 1.7  2007/03/16 10:51:36  martius
 *   each primitive has a substance
 *   geom userdata is set to primitive itself
 *
 *   Revision 1.6  2007/02/23 15:13:24  martius
 *   setColor
 *
 *   Revision 1.5  2007/01/26 12:05:36  martius
 *   joint support forces in uniform manner
 *
 *   Revision 1.4  2006/12/13 09:09:56  martius
 *   transform objects delete child
 *
 *   Revision 1.3  2006/08/30 08:58:56  martius
 *   categories and collision mask used for static geoms to reduce number of collision checks
 *
 *   Revision 1.2  2006/07/14 12:23:35  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.18  2006/07/14 11:23:38  martius
 *   revert to older revision of robot3
 *
 *   Revision 1.1.2.16  2006/06/29 16:35:32  robot3
 *   -Mesh code optimized
 *   -includes cleared up, more using forward declarations
 *    (sometimes additionally #include "osgprimitive.h" is needed)
 *
 *   Revision 1.1.2.15  2006/06/27 14:14:29  robot3
 *   -optimized mesh and boundingshape code
 *   -other changes
 *
 *   Revision 1.1.2.14  2006/06/23 08:53:56  robot3
 *   made some changes on primitive Mesh
 *
 *   Revision 1.1.2.13  2006/05/29 22:03:26  martius
 *   cylinder
 *
 *   Revision 1.1.2.12  2006/05/29 21:27:02  robot3
 *   made some preparations for the boundingshape of the Mesh
 *
 *   Revision 1.1.2.11  2006/05/28 22:14:57  martius
 *   heightfield included
 *
 *   Revision 1.1.2.10  2006/05/24 12:23:10  robot3
 *   -passive_mesh works now (simple bound_version)
 *   -Primitive Mesh now exists (simple bound_version)
 *
 *   Revision 1.1.2.9  2006/04/04 14:13:24  fhesse
 *   documentation improved
 *
 *   Revision 1.1.2.8  2006/03/29 15:07:17  martius
 *   Dummy Primitive
 *
 *   Revision 1.1.2.7  2006/01/31 15:45:35  martius
 *   proper destruction
 *
 *   Revision 1.1.2.6  2006/01/12 14:21:00  martius
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

#include <osg/Matrix>
#include <ode/common.h>

#include "substance.h"
// another forward declaration "block"
#include "osgforwarddecl.h"

namespace lpzrobots {

   /***** begin of forward declaration block *****/
   class BoundingShape;
   class OSGPrimitive;
   class OSGPlane;
   class OSGBox;
   class OSGBoxTex;
   class OSGSphere;
   class OSGCapsule;
   class OSGCylinder;
   class OSGDummy;
   class OSGMesh;
   /*typedef*/ struct GlobalData;
   class OdeHandle;
   class OsgHandle;
   class Color;
   /*****  end of forward declaration block  *****/


/// returns the osg (4x4) pose matrix of the ode geom
osg::Matrix osgPose( dGeomID geom );
/// returns the osg (4x4) pose matrix of the ode body
osg::Matrix osgPose( dBodyID body );
/// converts a position vector and a rotation matrix from ode to osg 4x4 matrix
osg::Matrix osgPose( const double * position , const double * rotation );
/// converts the rotation component of pose into an ode rotation matrix
void odeRotation( const osg::Matrix& pose , dMatrix3& odematrix);

/**
   Interface class for primitives represented in the physical and graphical world.
   This is intended to bring OSG and ODE together and hide most implementation details.
*/
class Primitive {
public:
  /** Body means that it is a dynamic object with a body.
      Geom means it has a geometrical represenation used for collision detection.
      Draw means the primitive is drawn
      Child is only used internally and is used for transformed geoms.
  */
  /*typedef*/ enum Modes {Body=1, Geom=2, Draw=4, Child=8};
  /*typedef*/ enum Category { Dyn=1, Stat=2};

  Primitive ();
  virtual ~Primitive ();
  /** registers primitive in ODE and OSG. 
      @param osgHandle scruct with ODE variables inside (to specify space, world...)
      @param mass Mass of the object in ODE (if withBody = true)
      @param osgHandle scruct with OSG variables inside (scene node, color ...)
      @param mode is a conjuction of Modes.
   */
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw)  = 0 ;

  /** Updates the OSG nodes with ODE coordinates.
      This function must be overloaded (usually calls setMatrix of OsgPrimitives)
   */
  virtual void update() =0 ;

  /// returns the assoziated osg primitive if there or 0
  virtual OSGPrimitive* getOSGPrimitive() = 0;

  /// sets the color for the underlaying osgprimitive
  virtual void setColor(const Color& color);

  /// assigns a texture to the primitive
  virtual void setTexture(const std::string& filename);
  /// assigns a texture to the primitive, you can choose if the texture should be repeated
  virtual void setTexture(const std::string& filename, double repeatOnX, double repeatOnY);
  /** assigns a texture to the x-th surface of the primitive, you can choose how often to repeat
      negative values of repeat correspond to length units. 
      E.g. assume a rectangle of size 5 in x direction: with repeatOnX = 2 the texture would be two times
      rereated. With repeatOnX = -1 the texture would be 5 times repeated because the texture is 
      made to have the size 1 
   */
  virtual void setTexture(int surface, const std::string& filename, double repeatOnX, double repeatOnY);


  /// set the position of the primitive (orientation is preserved)
  void setPosition(const osg::Vec3& pos);
  /// set the pose of the primitive
  void setPose(const osg::Matrix& pose);
  /// returns the position
  osg::Vec3 getPosition() const;
  /// returns the pose
  osg::Matrix getPose() const;

  /// sets the mass of the body (uniform)
  virtual void setMass(double mass) = 0;
  /** sets full mass specification
    \param cg center of gravity vector
    \param I  3x3 interia tensor
  */
  void setMass(double mass, double cgx, double cgy, double cgz,
	       double I11, double I22, double I33,
	       double I12, double I13, double I23);

  /// returns ODE geomID if there
  dGeomID getGeom() const;    
  /// returns ODE bodyID if there
  dBodyID getBody() const;

  /// checks whether the object has higher velocity than maxVel and limits it in case
  bool limitLinearVel(double maxVel);

  /// return the given point transformed to local coordinates of the primitive
  osg::Vec3 toLocal(const osg::Vec3& pos) const;
  /** return the given vector or axis transformed to local coordinates
      of the primitive (translation depends on the 4th coordinate)
  */
  osg::Vec4 toLocal(const osg::Vec4& axis) const;
  
protected:
  /** attaches geom to body (if any) and sets the category bits and collision bitfields.
      assumes: mode & Geom != 0
   */
  virtual void attachGeomAndSetColliderFlags();

public:
  Substance substance; // substance description
protected:
  dGeomID geom;
  dBodyID body;
  char mode;
};


/** Plane primitive */
class Plane : public Primitive {
public:
  Plane();
  virtual ~Plane();
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;

  virtual void update();  
  virtual OSGPrimitive* getOSGPrimitive();

  virtual void setMass(double mass);

protected:
  OSGPlane* osgplane;
};


/** Box primitive */
class Box : public Primitive {
public:

  Box(float lengthX, float lengthY, float lengthZ);
  virtual ~Box();

  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive();

  virtual void setMass(double mass);
protected:
  OSGBoxTex* osgbox;
};


/** Sphere primitive */
class Sphere : public Primitive {
public:
  Sphere(float radius);
  virtual ~Sphere();

  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive();

  virtual void setMass(double mass);

protected:
  OSGSphere* osgsphere;
};

/** Capsule primitive */
class Capsule : public Primitive {
public:
  Capsule(float radius, float height);
  virtual ~Capsule();
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive();

  virtual void setMass(double mass);

protected:
  OSGCapsule* osgcapsule;
};

/** Cylinder primitive */
class Cylinder : public Primitive {
public:
  Cylinder(float radius, float height);
  virtual ~Cylinder();
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive();

  virtual void setMass(double mass);
protected:
  OSGCylinder* osgcylinder;
};

/** Ray primitive 
    The actual visual representation can have different length than the
    ray object. This is specified by length. 
    SetLength is an efficient way to change the length at runtime.
*/
class Ray : public Primitive {
public:
  Ray(double range, float thickness, float length);
  virtual ~Ray();
  virtual void init(const OdeHandle& odeHandle, double mass,
      const OsgHandle& osgHandle,
      char mode = Geom | Draw) ;
  
  void setLength(float len);
  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive();
    
  virtual void setMass(double mass);
protected:
  double range;
  float thickness;
  float length;
  OSGBox* osgbox;
};




/** Mesh primitive */
class Mesh : public Primitive {
public:
  Mesh(const std::string& filename,float scale);
  virtual ~Mesh();
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw) ;
  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive();
  virtual float getRadius();

  virtual void setMass(double mass);
protected:
  OSGMesh* osgmesh;
  const std::string filename;
  float scale;
  BoundingShape* boundshape;
};


/**
   Primitive for transforming a geom (primitive without body) 
    in respect to a body (primitive with body). 
   Hides complexity of ODE TransformGeoms. 
*/
class Transform : public Primitive {
public:
  /** 
      @param parent primitive should have a body and should be initialised
      @param child  is transformed by pose in respect to parent. 
      This Primitive must NOT have a body and should not be initialised
  */
  Transform(Primitive* parent, Primitive* child, const osg::Matrix& pose);

  /// destructor deletes child object
  ~Transform();

  /** initialised the transform object. This automatically 
      initialises the child geom. 
      @param mass mass of the child
      @param mode is the mode for the child, except that Body bit is ignored (child can't have a body)
   */
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw);

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive();

  virtual void setMass(double mass);
protected:
  Primitive* parent;
  Primitive* child;
  osg::Matrix pose;
};

/**
   Dummy Primitive which returns 0 for geom and body. 
   Only useful for representing the static world in terms of primitives.
*/
class DummyPrimitive : public Primitive {
public:
  
  DummyPrimitive() {     
    body=0;
    geom=0;
  }
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle, char mode = Body | Geom | Draw) {
  }
  virtual void update() {}
  virtual OSGPrimitive* getOSGPrimitive() { return 0; }

  virtual void setMass(double mass) {}
  
};


}
#endif

