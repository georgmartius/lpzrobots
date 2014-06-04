
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
#ifndef __PRIMITIVE_H
#define __PRIMITIVE_H

#include <ode-dbl/common.h>
#include <selforg/storeable.h>

#include <vector>

#include "pos.h"
#include "pose.h"
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
   class TextureDescr;

   /* typedef */ struct GlobalData;

   class OdeHandle;
   class OsgHandle;
   class Color;
   /*****  end of forward declaration block  *****/


/// returns the osg (4x4) pose matrix of the ode geom
Pose osgPose( dGeomID geom );
/// returns the osg (4x4) pose matrix of the ode body
Pose osgPose( dBodyID body );
/// converts a position vector and a rotation matrix from ode to osg 4x4 matrix
Pose osgPose( const double * position , const double * rotation );
/// converts the rotation component of pose into an ode rotation matrix
void odeRotation( const Pose& pose , dMatrix3& odematrix);

/** counts number of max velocity violations at joints
 * (Attention, this is a global variable, initialized to 0 at start)
 */
extern int globalNumVelocityViolations;

/**
   Interface class for primitives represented in the physical and graphical world.
   This is intended to bring OSG and ODE together and hide most implementation details.
*/
class Primitive : public Storeable {
public:
  /** Body means that it is a dynamic object with a body.
      Geom means it has a geometrical represenation used for collision detection.
      Draw means the primitive is drawn.
      Density means that the mass is interpreted as a density
      _Child and _Transform are only used internally and used for transformed geoms.
  */

  /* typedef */ enum Modes {Body=1, Geom=2, Draw=4, Density=8,
                            _Child=16, _Transform=32};
  /* typedef */ enum Category { Dyn=1, Stat=2};


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

  /// sets the color using the colorschema of osgHandle
  virtual void setColor(const std::string& color);

  /// assigns a texture to the primitive
  virtual void setTexture(const std::string& filename);
  /// assigns a texture to the primitive, you can choose if the texture should be repeated
  virtual void setTexture(const TextureDescr& texture);
  /** assigns a texture to the x-th surface of the primitive.
      You can choose how often to repeat
      negative values of repeat correspond to length units.
      E.g. assume a rectangle of size 5 in x direction: with repeatOnX = 2 the texture would be two times
      rereated. With repeatOnX = -2 the texture would be 2.5 times repeated because the texture is
      made to have the size 2
   */
  virtual void setTexture(int surface, const TextureDescr& texture);
  /// assign a set of texture to the surfaces of the primitive
  virtual void setTextures(const std::vector<TextureDescr>& textures);


  /// set the position of the primitive (orientation is preserved)
  virtual void setPosition(const Pos& pos);
  /// set the pose of the primitive
  virtual void setPose(const Pose& pose);
  /// returns the position
  virtual Pos getPosition() const;
  /// returns the pose
  virtual Pose getPose() const;
  /// returns the velocity
  virtual Pos getVel() const;
  /// returns the angular velocity
  virtual Pos getAngularVel() const;

  /** apply a force (in world coordinates) to the primitive and
      returns true if it was possible */
  virtual bool applyForce(osg::Vec3 force);
  /** @see applyForce(osg::Vec3) */
  virtual bool applyForce(double x, double y, double z);

  /** apply a torque (in world coordinates) to the primitive and
      returns true if it was possible
   */
  virtual bool applyTorque(osg::Vec3 torque);
  /** @see applyTorque(osg::Vec3) */
  virtual bool applyTorque(double x, double y, double z);

  /** sets the mass of the body (uniform)
      if density==true then mass is interpreted as a density
   */
  virtual void setMass(double mass, bool density = false) = 0;
  /** sets full mass specification
    \param cg center of gravity vector
    \param I  3x3 interia tensor
  */
  void setMass(double mass, double cgx, double cgy, double cgz,
               double I11, double I22, double I33,
               double I12, double I13, double I23);

  /// returns ODE geomID if there
  dGeomID getGeom() const { return geom; }
  /// returns ODE bodyID if there
  dBodyID getBody() const { return body; }

  /// checks whether the object has higher velocity than maxVel and limits it in case
  bool limitLinearVel(double maxVel);

  /// checks whether the object has higher velocity than maxVel and limits it in case
  bool limitAngularVel(double maxVel);

  /** applies a force to the body to decellerate its linear and angular velocity with
      the given factors. (depends on stepwidth!)
   */
  void decellerate(double factorLin, double factorAng);


  /// return the given point transformed to local coordinates of the primitive
  osg::Vec3 toLocal(const osg::Vec3& pos) const;
  /** return the given vector or axis transformed to local coordinates
      of the primitive (translation depends on the 4th coordinate)
  */
  osg::Vec4 toLocal(const osg::Vec4& axis) const;

  /// transforms the given point in local corrds of the primitive to global coordinates
  osg::Vec3 toGlobal(const osg::Vec3& pos) const;
  /**  transforms the given vector or axis in local corrds of the primitive to global coordinates
       (translation depends on the 4th coordinate)
  */
  osg::Vec4 toGlobal(const osg::Vec4& axis) const;

  /**
   * 20091023; guettler:
   * hack for tasked simulations; there are some problems if running in parallel mode,
   * if you do not destroy the geom, everything is fine (should be no problem because world is destroying geoms too)
   * @param _destroyGeom set this to false if geoms must not be destroyed if the primitive is destroyed
   */
  static void setDestroyGeomFlag(bool _destroyGeom) {
    destroyGeom = _destroyGeom;
  }

  int getNumVelocityViolations(){ return numVelocityViolations; }

  void setSubstance(const Substance& substance);

  /* **** storable interface *******/
  virtual bool store(FILE* f) const;

  virtual bool restore(FILE* f);


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
  bool substanceManuallySet;
  int numVelocityViolations; ///< number of times the maximal velocity was exceeded

  // 20091023; guettler:
  // hack for tasked simulations; there are some problems if running in parallel mode,
  // if you do not destroy the geom, everything is fine (should be no problem because world is destroying geoms too)
  static bool destroyGeom;
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

  virtual void setMass(double mass, bool density = false);

protected:
  OSGPlane* osgplane;
};


/** Box primitive */
class Box : public Primitive {
public:

  Box(float lengthX, float lengthY, float lengthZ);
  Box(const osg::Vec3& dim);

  virtual ~Box();

  virtual void init(const OdeHandle& odeHandle, double mass,
                    const OsgHandle& osgHandle,
                    char mode = Body | Geom | Draw) ;

  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive();

  virtual void setMass(double mass, bool density = false);
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

  virtual void setMass(double mass, bool density = false);

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

  virtual void setMass(double mass, bool density = false);

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

  virtual void setMass(double mass, bool density = false);
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
  /**
     @param thickness if thickness == 0 then a line is used and not a box
   */
  Ray(double range, float thickness, float length);
  virtual ~Ray();
  virtual void init(const OdeHandle& odeHandle, double mass,
      const OsgHandle& osgHandle,
      char mode = Geom | Draw) ;

  void setLength(float len);
  virtual void update();
  virtual OSGPrimitive* getOSGPrimitive();

  virtual void setMass(double mass, bool density = false);
protected:
  double range;
  float thickness;
  float length;
  OSGPrimitive* osgprimitive;
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

  virtual void setMass(double mass, bool density = false);

  /**
   * Sets the BoundingShape externally (e.g. XMLBoundingShape).
   * Any existing BoundingShape will be deleted.
   */
  virtual void setBoundingShape(BoundingShape* boundingShape);

  virtual void setPose(const Pose& pose);

protected:
  OSGMesh* osgmesh;
  const std::string filename;
  float scale;
  BoundingShape* boundshape;
  Pose poseWithoutBodyAndGeom;

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
  Transform(Primitive* parent, Primitive* child, const Pose& pose, bool deleteChild = true);

  /// destructor deletes child object // it should be virtual by yuichi
  virtual ~Transform();

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

  virtual void setMass(double mass, bool density = false);

  // setting the pose is not supported
  virtual void setPose(const Pose& pose) override {}

protected:
  Primitive* parent;
  Primitive* child;
  Pose pose;
  bool deleteChild;
};

/**
   Dummy Primitive which returns 0 for geom and body.
   Only useful for representing the static world in terms of primitives
   or if virtual objects are created, but then the position and speed has
   to be set manually.
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

  virtual void setMass(double mass, bool density = false) {}

  virtual void setPosition(Pos pos){
    this->pos=pos;
  }
  virtual Pos getPosition() const {
    return pos;
  }
  virtual void setVel(Pos vel){
    this->vel=vel;
  }
  virtual Pos getVel() const {
    return vel;
  }

private:
  Pos vel;
  Pos pos;
};


}
#endif
