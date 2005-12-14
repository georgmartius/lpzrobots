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
 *   Revision 1.1.2.5  2005-12-14 15:36:45  martius
 *   joints are visible now
 *
 *   Revision 1.1.2.4  2005/12/13 18:11:13  martius
 *   transform primitive added, some joints stuff done, forward declaration
 *
 *   Revision 1.1.2.3  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.2  2005/12/09 16:54:16  martius
 *   camera is woring now
 *
 *   Revision 1.1.2.1  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *                                                                 *
 *                                                                         *
 ***************************************************************************/

#include <assert.h>
#include <osg/MatrixTransform>

#include "primitive.h"
#include "pos.h"

namespace lpzrobots{

  /// returns the osg (4x4) pose matrix of the ode geom
  osg::Matrix osgPose( dGeomID geom ){
    return osgPose(dGeomGetPosition(geom), dGeomGetRotation(geom));
  }

  /// returns the osg (4x4) pose matrix of the ode body
  osg::Matrix osgPose( dBodyID body ){
    return osgPose(dBodyGetPosition(body), dBodyGetRotation(body));
  }

  /// converts a position vector and a rotation matrix from ode to osg 4x4 matrix
  osg::Matrix osgPose( const double * V , const double * R ){
    return osg::Matrix( R[0], R[4], R[8],  0,
			R[1], R[5], R[9],  0,
			R[2], R[6], R[10], 0,
			V[0], V[1], V[2] , 1);  
  }

  /// converts a position vector and a rotation matrix from ode to osg 4x4 matrix
  void odeRotation( const osg::Matrix& pose , dMatrix3& odematrix){
    osg::Quat q;
    pose.get(q);
    dQuaternion quat = {q.x(), q.y(), q.z(), q.w() };
    dQtoR(quat, odematrix);
  }



  /******************************************************************************/

  Primitive::Primitive() 
    : geom(0), body(0) {
  }

  Primitive::~Primitive () {
    if(geom) dGeomDestroy( geom );
    if(body) dBodyDestroy( body );
  }


  void Primitive::setPosition(const osg::Vec3& pos){
    if(body){
      dBodySetPosition(body, pos.x(), pos.y(), pos.z());
    }else if(geom){ // okay there is just a geom no body
      dGeomSetPosition(geom, pos.x(), pos.y(), pos.z());
    }
    update(); // update the scenegraph stuff
  }

  void Primitive::setPose(const osg::Matrix& pose){
    if(body){
      osg::Vec3 pos = pose.getTrans();
      dBodySetPosition(body, pos.x(), pos.y(), pos.z());    
      osg::Quat q;
      pose.get(q);
      dReal quat[4] = {q.x(), q.y(), q.z(), q.w()};
      dBodySetQuaternion(body, quat);
    }else if(geom){ // okay there is just a geom no body
      osg::Vec3 pos = pose.getTrans();
      dGeomSetPosition(geom, pos.x(), pos.y(), pos.z());    
      osg::Quat q;
      pose.get(q);
      dReal quat[4] = {q.x(), q.y(), q.z(), q.w()};
      dGeomSetQuaternion(geom, quat);
    }  
  }

  osg::Vec3 Primitive::getPosition() const {
    return Pos(dBodyGetPosition(body));
  }

  osg::Matrix Primitive::getPose() const {
    return osgPose(dBodyGetPosition(body), dBodyGetRotation(body));    
  }

  dGeomID Primitive::getGeom() const { 
    return geom; 
  }

  dBodyID Primitive::getBody() const { 
    return body; 
  }

  /******************************************************************************/
  Plane::Plane() {
  }

  void Plane::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		   bool withBody){
    OSGPlane::init(osgHandle);
    geom = dCreatePlane ( odeHandle.space , 0 , 0 , 1 , 0 );
    if (withBody){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetBox(&m,1,1000,1000,0.01); // fake the mass of the plane with a thin box
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
      dGeomSetBody (geom, body); // geom is assigned to body
    }  
  }

  void Plane:: update(){
    if(body)
      OSGPlane::setMatrix(osgPose(body));
    else if(geom) OSGPlane::setMatrix(osgPose(geom));
  }

  osg::Transform* Plane::getTransform() {
    return transform.get();
  }

  /******************************************************************************/
  Box::Box(float lengthX, float lengthY, float lengthZ)
    : OSGBox(lengthX, lengthY, lengthZ) {
  }

  void Box::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		 bool withBody){
    OSGBox::init(osgHandle);
    geom = dCreateBox ( odeHandle.space , lengthX , lengthY , lengthZ);
    if (withBody){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetBox(&m, 1, lengthX, lengthY, lengthZ); // fake the mass of the plane with a thin box
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
      dGeomSetBody (geom, body); // geom is assigned to body
    }  
  }

  void Box:: update(){
    if(body)
      OSGBox::setMatrix(osgPose(body));
    else if(geom) OSGBox::setMatrix(osgPose(geom));
  }

  osg::Transform* Box::getTransform()  {
    return transform.get();
  }

  /******************************************************************************/
  Sphere::Sphere(float radius)
    : OSGSphere(radius) {
  }

  void Sphere::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		    bool withBody){
    OSGSphere::init(osgHandle);
    geom = dCreateSphere ( odeHandle.space , radius);
    if (withBody){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetSphere(&m, 1, radius); // fake the mass of the plane with a thin box
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
      dGeomSetBody (geom, body); // geom is assigned to body
    }  
  }

  void Sphere::update(){
    if(body)
      OSGSphere::setMatrix(osgPose(body));
    else if(geom) OSGSphere::setMatrix(osgPose(geom));
  }

  osg::Transform* Sphere::getTransform()  {
    return transform.get();
  }

  /******************************************************************************/
  Capsule::Capsule(float radius, float height)
    : OSGCapsule(radius, height) {
  }

  void Capsule::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		     bool withBody){
    OSGCapsule::init(osgHandle);
    geom = dCreateCCylinder ( odeHandle.space , radius, height);
    if (withBody){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetCappedCylinder(&m, 1.0, 3 , radius, height); // fake the mass of the plane with a thin box
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
      dGeomSetBody (geom, body); // geom is assigned to body
    }  
  }

  void Capsule::update(){
    if(body)
      OSGCapsule::setMatrix(osgPose(body));
    else if(geom) OSGCapsule::setMatrix(osgPose(geom));
  }

  osg::Transform* Capsule::getTransform()  {
    return transform.get();
  }

  /******************************************************************************/
  Transform::Transform(Primitive* parent, Primitive* child, const osg::Matrix& pose)
    : parent(parent), child(child), pose(pose) {
  }

  void Transform::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		       bool withBody){
    assert(withBody == false);
    assert(parent && parent->getBody() != 0 && child); // parent and child must exist
    assert(child->getBody() == 0 && child->getGeom() == 0); // child should not be initialised    

    // our own geom is just a transform
    geom = dCreateGeomTransform(odeHandle.space);
    dGeomTransformSetInfo(geom, 1);
    dGeomTransformSetCleanup(geom, 1);

    // the child geom must go into space 0 (because it inherits the space from the transform geom)
    OdeHandle odeHandleChild(odeHandle);
    odeHandleChild.space = 0;
    // the root node for the child is the transform node of the parent
    OsgHandle osgHandleChild(osgHandle);
    osgHandleChild.scene = parent->getTransform();
    assert(osgHandleChild.scene);
    // initialise the child
    child->init(odeHandleChild, 0, osgHandleChild, false);
    // move the child to the right place (in local coordinates)
    child->setPose(pose);
  
    // assoziate the child with the transform geom
    dGeomTransformSetGeom (geom, child->getGeom());
    // finally bind the transform the body of parent
    dGeomSetBody (geom, parent->getBody());
  }

  void Transform::update(){
    // we don't need to update something
  }

  osg::Transform* Transform::getTransform()  {
    // the funny thing is that we don't have one :-)
    return 0;
  }

}
