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
 *   Revision 1.1.2.10  2006-01-31 15:45:28  martius
 *   proper destruction
 *
 *   Revision 1.1.2.9  2006/01/27 13:06:21  fhesse
 *   getPose/getPosition return pose/Position of geom if no body exists
 *
 *   Revision 1.1.2.8  2006/01/12 14:21:00  martius
 *   drawmode, material
 *
 *   Revision 1.1.2.7  2006/01/03 10:06:16  fhesse
 *   primitive::getGeom and getBody return 0 if it is zero pointer
 *
 *   Revision 1.1.2.6  2005/12/15 17:03:43  martius
 *   cameramanupulator setPose is working
 *   joints have setter and getter parameters
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them
 *
 *   Revision 1.1.2.5  2005/12/14 15:36:45  martius
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
    // THIS should be
    //    dQuaternion quat = {q.x(), q.y(), q.z(), q.w() };
    dQuaternion quat = {q.w(), q.x(), q.y(), q.z() };
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
      // this should be
      //      dReal quat[4] = {q.x(), q.y(), q.z(), q.w()};
      dReal quat[4] = {q.w(), q.x(), q.y(), q.z()};
      dBodySetQuaternion(body, quat);
    }else if(geom){ // okay there is just a geom no body
      osg::Vec3 pos = pose.getTrans();
      dGeomSetPosition(geom, pos.x(), pos.y(), pos.z());    
      osg::Quat q;
      pose.get(q);
      // this should be
      // dReal quat[4] = {q.x(), q.y(), q.z(), q.w()};
      dReal quat[4] = {q.w(), q.x(), q.y(), q.z()};
      dGeomSetQuaternion(geom, quat);
    }  
  }

  osg::Vec3 Primitive::getPosition() const {
    if(!body) return Pos(dGeomGetPosition(geom));
    return Pos(dBodyGetPosition(body));
  }

  osg::Matrix Primitive::getPose() const {
    if(!body) return osgPose(dGeomGetPosition(geom), dGeomGetRotation(geom));    
    return osgPose(dBodyGetPosition(body), dBodyGetRotation(body));
  }

  dGeomID Primitive::getGeom() const { 
   if (this) return geom; 
   else return 0;
  }

  dBodyID Primitive::getBody() const { 
   if (this) return body; 
   else return 0;
  }

  /******************************************************************************/
  Plane::Plane(){    
    osgplane = new OSGPlane();    
  }

  Plane::~Plane(){
    if(osgplane) delete osgplane;
  }

  void Plane::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		   char mode) {
    assert(mode & Body || mode & Geom);
    this->mode=mode;
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetBox(&m,1,1000,1000,0.01); // fake the mass of the plane with a thin box
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }
    if(mode & Geom){
      geom = dCreatePlane ( odeHandle.space , 0 , 0 , 1 , 0 );      
      if(mode & Body)
	dGeomSetBody (geom, body); // geom is assigned to body	    
    }
    if(mode & Draw){
      osgplane->init(osgHandle);
    }

  }

  void Plane:: update(){
    if(mode & Draw) {
      if(body)
	osgplane->setMatrix(osgPose(body));
      else 
	osgplane->setMatrix(osgPose(geom));
    }
  }

  /******************************************************************************/
  Box::Box(float lengthX, float lengthY, float lengthZ) {
    osgbox = new OSGBox(lengthX, lengthY, lengthZ);    
  }

  Box::~Box(){
    if(osgbox) delete osgbox; 
  }

  void Box::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		 char mode) {
    assert((mode & Body) || (mode & Geom));
    this->mode=mode;
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      // fake the mass of the plane with a thin box
      dMassSetBox(&m, 1, osgbox->getLengthX() , osgbox->getLengthY() , osgbox->getLengthZ()); 
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body      
    }  
    if (mode & Geom){    
      geom = dCreateBox ( odeHandle.space , osgbox->getLengthX() , 
			  osgbox->getLengthY() , osgbox->getLengthZ());
      if (mode & Body){
	dGeomSetBody (geom, body); // geom is assigned to body
      }
    }
    if (mode & Draw){
      osgbox->init(osgHandle);      
    }
  }

  void Box:: update(){
    if(mode & Draw) {
      if(body)
	osgbox->setMatrix(osgPose(body));
      else
	osgbox->setMatrix(osgPose(geom));
    }
  }

  /******************************************************************************/
  Sphere::Sphere(float radius) {
    osgsphere = new OSGSphere(radius);
  }

  Sphere::~Sphere(){
    if(osgsphere) delete osgsphere; 
  }

  void Sphere::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		    char mode) {
    assert(mode & Body || mode & Geom);
    this->mode=mode;
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetSphere(&m, 1, osgsphere->getRadius());
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }  
    if (mode & Geom){
      geom = dCreateSphere ( odeHandle.space , osgsphere->getRadius());
      if (mode & Body)
	dGeomSetBody (geom, body); // geom is assigned to body
    }
    if (mode & Draw){
      osgsphere->init(osgHandle);      
    }

  }

  void Sphere::update(){
    if(mode & Draw) {
      if(body)
	osgsphere->setMatrix(osgPose(body));
      else 
	osgsphere->setMatrix(osgPose(geom));    
    }
  }

  /******************************************************************************/
  Capsule::Capsule(float radius, float height) {    
    osgcapsule = new OSGCapsule(radius, height);
  }

  Capsule::~Capsule(){
    if(osgcapsule) delete osgcapsule; 
  }

  void Capsule::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		     char mode) {
    assert(mode & Body || mode & Geom);
    this->mode=mode;
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetCappedCylinder(&m, 1.0, 3 , osgcapsule->getRadius(), osgcapsule->getHeight()); 
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }  
    if (mode & Geom){    
      geom = dCreateCCylinder ( odeHandle.space , osgcapsule->getRadius(), osgcapsule->getHeight());
      if (mode & Body)
	dGeomSetBody (geom, body); // geom is assigned to body      
    }
    if (mode & Draw){
      osgcapsule->init(osgHandle);
    }
  }

  void Capsule::update(){
    if(mode & Draw) {
      if(body)
	osgcapsule->setMatrix(osgPose(body));
      else 
	osgcapsule->setMatrix(osgPose(geom));
    }
  }

  /******************************************************************************/
  Transform::Transform(Primitive* parent, Primitive* child, const osg::Matrix& pose)
    : parent(parent), child(child), pose(pose) {
  }

  void Transform::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		       char mode) {
    // we ignore the mode!
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
    osgHandleChild.scene = parent->getOSGPrimitive()->getTransform();
    assert(osgHandleChild.scene);
    // initialise the child
    child->init(odeHandleChild, 0, osgHandleChild, Primitive::Geom | Primitive::Draw);
    // move the child to the right place (in local coordinates)
    child->setPose(pose);
  
    // assoziate the child with the transform geom
    dGeomTransformSetGeom (geom, child->getGeom());
    // finally bind the transform the body of parent
    dGeomSetBody (geom, parent->getBody());    
  }

  void Transform::update(){
    if(child) child->update();
  }

}
