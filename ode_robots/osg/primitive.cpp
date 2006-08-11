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
 *   $Log$
 *   Revision 1.4  2006-08-11 15:43:14  martius
 *   osgDB filepath search is now moved to bbox
 *
 *   Revision 1.3  2006/07/14 21:37:56  robot3
 *   .bbox-file uses now the searchpath of osgDB
 *
 *   Revision 1.2  2006/07/14 12:23:35  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.22  2006/07/14 11:23:38  martius
 *   revert to older revision of robot3
 *
 *   Revision 1.1.2.20  2006/06/29 16:35:32  robot3
 *   -Mesh code optimized
 *   -includes cleared up, more using forward declarations
 *    (sometimes additionally #include "osgprimitive.h" is needed)
 *
 *   Revision 1.1.2.19  2006/06/27 14:14:29  robot3
 *   -optimized mesh and boundingshape code
 *   -other changes
 *
 *   Revision 1.1.2.18  2006/06/26 21:51:59  robot3
 *   Mesh works now with bbox file
 *
 *   Revision 1.1.2.17  2006/06/23 08:53:40  robot3
 *   made some changes on primitive Mesh
 *
 *   Revision 1.1.2.16  2006/05/29 22:03:49  martius
 *   cylinder
 *
 *   Revision 1.1.2.15  2006/05/29 21:26:48  robot3
 *   made some preparations for the boundingshape of the Mesh
 *
 *   Revision 1.1.2.14  2006/05/28 22:14:57  martius
 *   heightfield included
 *
 *   Revision 1.1.2.13  2006/05/24 12:23:10  robot3
 *   -passive_mesh works now (simple bound_version)
 *   -Primitive Mesh now exists (simple bound_version)
 *
 *   Revision 1.1.2.12  2006/03/30 12:34:53  martius
 *   documentation updated
 *
 *   Revision 1.1.2.11  2006/03/29 15:06:58  martius
 *   update on setPose
 *
 *   Revision 1.1.2.10  2006/01/31 15:45:28  martius
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
#include "boundingshape.h"
#include "osgprimitive.h"
#include "odehandle.h"
#include "globaldata.h"


namespace lpzrobots{

  // returns the osg (4x4) pose matrix of the ode geom
  osg::Matrix osgPose( dGeomID geom ){
    return osgPose(dGeomGetPosition(geom), dGeomGetRotation(geom));
  }

  // returns the osg (4x4) pose matrix of the ode body
  osg::Matrix osgPose( dBodyID body ){
    return osgPose(dBodyGetPosition(body), dBodyGetRotation(body));
  }

  // converts a position vector and a rotation matrix from ode to osg 4x4 matrix
  osg::Matrix osgPose( const double * V , const double * R ){
    return osg::Matrix( R[0], R[4], R[8],  0,
			R[1], R[5], R[9],  0,
			R[2], R[6], R[10], 0,
			V[0], V[1], V[2] , 1);  
  }

  // converts a position vector and a rotation matrix from ode to osg 4x4 matrix
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


  
  void Primitive::setTexture(const std::string& filename){
    if(getOSGPrimitive())
      getOSGPrimitive()->setTexture(filename);
  }
  
  void Primitive::setTexture(const std::string& filename, bool repeatOnX, bool repeatOnY){
    if(getOSGPrimitive())
      getOSGPrimitive()->setTexture(filename, repeatOnX, repeatOnY);
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
    update(); // update the scenegraph stuff
  }

  osg::Vec3 Primitive::getPosition() const {
    if(!body) return Pos(dGeomGetPosition(geom));
    return Pos(dBodyGetPosition(body));
  }

  osg::Matrix Primitive::getPose() const {
    if(!body) {
      if (!geom) 
	return osg::Matrix::translate(0.0f,0.0f,0.0f); // fixes init bug
	else 
	return osgPose(dGeomGetPosition(geom), dGeomGetRotation(geom));    
    }
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

  OSGPrimitive* Plane::getOSGPrimitive() { return osgplane; }

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

  OSGPrimitive* Box::getOSGPrimitive() { return osgbox; }

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

  OSGPrimitive* Sphere::getOSGPrimitive() { return osgsphere; }

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

  OSGPrimitive* Capsule::getOSGPrimitive() { return osgcapsule; }

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
  Cylinder::Cylinder(float radius, float height) {    
    osgcylinder = new OSGCylinder(radius, height);
  }

  Cylinder::~Cylinder(){
    if(osgcylinder) delete osgcylinder; 
  }

  OSGPrimitive* Cylinder::getOSGPrimitive() { return osgcylinder; }

  void Cylinder::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		     char mode) {
    assert(mode & Body || mode & Geom);
    this->mode=mode;
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetCylinder(&m, 1.0, 3 , osgcylinder->getRadius(), osgcylinder->getHeight()); 
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }  
    if (mode & Geom){    
      geom = dCreateCylinder ( odeHandle.space , osgcylinder->getRadius(), osgcylinder->getHeight());
      if (mode & Body)
	dGeomSetBody (geom, body); // geom is assigned to body      
    }
    if (mode & Draw){
      osgcylinder->init(osgHandle);
    }
  }

  void Cylinder::update(){
    if(mode & Draw) {
      if(body)
	osgcylinder->setMatrix(osgPose(body));
      else 
	osgcylinder->setMatrix(osgPose(geom));
    }
  }

  /******************************************************************************/
  Transform::Transform(Primitive* parent, Primitive* child, const osg::Matrix& pose)
    : parent(parent), child(child), pose(pose) {
  }

  OSGPrimitive* Transform::getOSGPrimitive() { return 0; }

  void Transform::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		       char mode) {
    // Primitive::body is ignored (removed) from mode
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
    child->init(odeHandleChild, 0, osgHandleChild, mode & ~Primitive::Body );
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

  /******************************************************************************/

  Mesh::Mesh(const std::string& filename,float scale,GlobalData& global) :
    filename(filename), scale(scale)  {    
    osgmesh = new OSGMesh(filename, scale);
    if (global.odeConfig.drawBoundings)
      drawBoundingMode=Primitive::Geom | Primitive::Draw;
    else
      drawBoundingMode=Primitive::Geom;
  }

  Mesh::~Mesh(){
    if(osgmesh) delete osgmesh; 
  }

  OSGPrimitive* Mesh::getOSGPrimitive() { return osgmesh; }

  void Mesh::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		     char mode) {
    assert(mode & Body || mode & Geom);
    this->mode=mode;
    if (mode & Draw){
      osgmesh->init(osgHandle);
    }
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetSphere(&m, 1, osgmesh->getRadius()); // we use a sphere
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    } 
    // read boundingshape file
    //    const osg::BoundingSphere& bsphere = osgmesh->getGroup()->getBound(); 
    boundshape = new BoundingShape(filename+".bbox" ,this);
    if(!boundshape->init(odeHandle, osgHandle.changeColor(Color(1,0,0,0.3)), scale, drawBoundingMode)){
      printf("use default bounding box, because bbox file not found!\n");
      Primitive* bound = new Sphere(osgmesh->getRadius()); 
      Transform* trans = new Transform(this,bound,osg::Matrix::translate(0.0f,0.0f,0.0f));
      trans->init(odeHandle, 0, osgHandle.changeColor(Color(1,0,0,0.3)),drawBoundingMode);
      osgmesh->setMatrix(osg::Matrix::translate(0.0f,0.0f,osgmesh->getRadius())*getPose()); // set obstacle higher
    }
  }

  float Mesh::getRadius() { return osgmesh->getRadius(); }

  void Mesh::update(){
    if(mode & Draw) {
      if(body) {
	osgmesh->setMatrix(osgPose(body));
      }
      else {
	osgmesh->setMatrix(osgPose(geom));
      }
    }
  }

}
