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
 *   Revision 1.33  2010-11-05 13:54:05  martius
 *   store and restore for robots implemented
 *
 *   Revision 1.32  2010/09/24 13:38:48  martius
 *   added toGlobal and applyForce/Torque with doubles
 *
 *   Revision 1.31  2010/09/17 10:07:45  martius
 *   changing size requires invalidation of display list
 *
 *   Revision 1.30  2010/03/25 16:39:51  martius
 *   primitive has addForce/addTorque function
 *
 *   Revision 1.29  2010/03/16 15:47:46  martius
 *   osgHandle has now substructures osgConfig and osgScene
 *    that minimized amount of redundant data (this causes a lot of changes)
 *   Scenegraph is slightly changed. There is a world and a world_noshadow now.
 *    Main idea is to have a world without shadow all the time avaiable for the
 *    Robot cameras (since they do not see the right shadow for some reason)
 *   tidied up old files
 *
 *   Revision 1.28  2010/03/11 15:17:19  guettler
 *   -BoundingShape can now be set from outside (see XMLBoundingShape)
 *   -Mesh can be created without Body and Geom.
 *
 *   Revision 1.27  2010/03/07 22:47:59  guettler
 *   - support for manually setting the substance (substance was overwritten in initialise methods)
 *   - enhancements for Mesh (mode Body, Geom, Draw), is needed for XMLImport
 *
 *   Revision 1.26  2010/03/05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *   Revision 1.25  2010/01/26 09:38:17  martius
 *   getVelocity, getAngularVel added
 *
 *   Revision 1.24  2009/10/23 12:47:13  guettler
 *   hack for tasked simulations:
 *   there are some problems if running in parallel mode,
 *   if you do not destroy the geom, everything is fine
 *   (should be no problem because world is destroying geoms too)
 *
 *   Revision 1.23  2009/08/10 07:47:58  guettler
 *   added some QMP critical sections (not compromising normal use of this class)
 *
 *   Revision 1.22  2009/08/03 14:09:48  jhoffmann
 *   Remove some compiling warnings, memory leaks; Add some code cleanups
 *
 *   Revision 1.21  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.20  2009/01/20 22:41:19  martius
 *   manipulation of agents with the mouse implemented ( a dream... )
 *
 *   Revision 1.19  2009/01/20 17:29:10  martius
 *   changed texture handling. In principle it is possible to set multiple textures
 *   per osgPrimitive.
 *   New osgboxtex started that supports custom textures.
 *
 *   Revision 1.18  2008/09/11 15:24:01  martius
 *   motioncallback resurrected
 *   noContact substance
 *   use slider center of the connecting objects for slider drawing
 *
 *   Revision 1.17  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.16  2007/11/07 13:18:44  martius
 *   toLocal: coordinate transformation
 *
 *   Revision 1.15  2007/09/06 18:47:17  martius
 *   deletion of geom now here again
 *
 *   Revision 1.14  2007/08/23 15:33:19  martius
 *   geoms are actually destroyed by DestroySpace
 *
 *   Revision 1.13  2007/08/23 14:52:07  martius
 *   Ray
 *
 *   Revision 1.12  2007/07/31 08:21:34  martius
 *   OSGMesh does not need GlobalData
 *   drawBoundings moved to OsgHandle
 *
 *   Revision 1.11  2007/07/17 07:19:54  martius
 *   setMass added
 *
 *   Revision 1.10  2007/07/03 13:12:40  martius
 *   limitLinearVel
 *
 *   Revision 1.9  2007/03/16 10:51:45  martius
 *   each primitive has a substance
 *   geom userdata is set to primitive itself
 *
 *   Revision 1.8  2007/02/23 15:13:32  martius
 *   setColor
 *
 *   Revision 1.7  2007/01/26 12:05:36  martius
 *   joint support forces in uniform manner
 *
 *   Revision 1.6  2006/12/13 09:09:56  martius
 *   transform objects delete child
 *
 *   Revision 1.5  2006/08/30 08:59:07  martius
 *   categories and collision mask used for static geoms to reduce number of collision checks
 *
 *   Revision 1.4  2006/08/11 15:43:14  martius
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
#include <osg/Vec4>

#include "primitive.h"
#include "pos.h"
#include "boundingshape.h"
#include "osgprimitive.h"
#include "odehandle.h"
#include "globaldata.h"

#include <selforg/quickmp.h>

namespace lpzrobots{

  // 20091023; guettler:
  // hack for tasked simulations; there are some problems if running in parallel mode,
  // if you do not destroy the geom, everything is fine (should be no problem because world is destroying geoms too)
  bool Primitive::destroyGeom = true; // this is the default case, is set to false in SimulationTaskSupervisor

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

  // converts a osg 4x4 matrix to an ode version of it
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
    : geom(0), body(0), substanceManuallySet(false) {
  }

  Primitive::~Primitive () {
    QMP_CRITICAL(8);
    // 20091023; guettler:
    // hack for tasked simulations; there are some problems if running in parallel mode,
    // if you do not destroy the geom, everything is fine (should be no problem because world is destroying geoms too)
    if(destroyGeom && geom) dGeomDestroy( geom );
    if(body) dBodyDestroy( body );
    QMP_END_CRITICAL(8);
  }


  void Primitive::attachGeomAndSetColliderFlags(){
    if(mode & Body){
      // geom is assigned to body and is set into category Dyn
      dGeomSetBody (geom, body); 
      dGeomSetCategoryBits (geom, Dyn);
      dGeomSetCollideBits (geom, ~0x0); // collides with everything
    } else {
      // geom is static, so it is member of the static category and will collide not with other statics
      dGeomSetCategoryBits (geom, Stat);
      dGeomSetCollideBits (geom, ~Stat);
    }
    if(mode & Child){ // in case of a child object it is always dynamic
      dGeomSetCategoryBits (geom, Dyn);
      dGeomSetCollideBits (geom, ~0x0); // collides with everything
    }
    dGeomSetData(geom, (void*)this); // set primitive as geom data
  }


  void Primitive::setColor(const Color& color){
    if(getOSGPrimitive())
      getOSGPrimitive()->setColor(color);
  }

  
  void Primitive::setTexture(const std::string& filename){
    if(getOSGPrimitive())
      getOSGPrimitive()->setTexture(filename);
  }
  
  void Primitive::setTexture(const std::string& filename, double repeatOnX, double repeatOnY){
    if(getOSGPrimitive())
      getOSGPrimitive()->setTexture(filename, repeatOnX, repeatOnY);
  }

  void Primitive::setTexture(int surface, const std::string& filename, double repeatOnX, double repeatOnY){
    if(getOSGPrimitive()) 
      getOSGPrimitive()->setTexture(surface, filename, repeatOnX, repeatOnY);
  }

  void Primitive::setPosition(const Pos& pos){
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

  Pos Primitive::getPosition() const {
    if(geom) return Pos(dGeomGetPosition(geom));
    else if(body) return Pos(dBodyGetPosition(body));
    else return Pos(0,0,0);
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

  Pos Primitive::getVel() const{  
    if(body)
      return Pos(dBodyGetLinearVel(body));     
    else return Pos(0,0,0);
  }

  Pos Primitive::getAngularVel() const {
    if(body)
      return Pos(dBodyGetAngularVel(body));     
    else return Pos(0,0,0);
  }
  
  bool Primitive::applyForce(osg::Vec3 force){
    return applyForce(force.x(), force.y(), force.z()); 
  }

  bool Primitive::applyForce(double x, double y, double z){
    if(body){
      dBodyAddForce(body, x, y, z); 
      return true;
    } else return false;

  }

  bool Primitive::applyTorque(osg::Vec3 torque){
    return applyTorque(torque.x(), torque.y(), torque.z()); 
  }

  bool Primitive::applyTorque(double x, double y, double z){
    if(body){
      dBodyAddTorque(body, x, y, z); 
      return true;
    } else return false;
  }

  dGeomID Primitive::getGeom() const { 
   return geom;    
  }

  dBodyID Primitive::getBody() const { 
   return body;    
  }

  /** sets full mass specification.
    \b cg is center of gravity vector
    \b I are parts of the 3x3 interia tensor
  */
  void Primitive::setMass(double mass, double cgx, double cgy, double cgz,
			  double I11, double I22, double I33,
			  double I12, double I13, double I23){
    dMass mass0;
    dMassSetParameters(&mass0, mass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23);
    dBodySetMass(body, &mass0);
  }


  bool Primitive::limitLinearVel(double maxVel){
    // check for maximum speed:
    if(!body) return false;
    const double* vel = dBodyGetLinearVel( body );
    double vellen = vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2];
    if(vellen > maxVel*maxVel){
      fprintf(stderr, ".");
      double scaling = sqrt(vellen)/maxVel;
      dBodySetLinearVel(body, vel[0]/scaling, vel[1]/scaling, vel[2]/scaling);
      return true;
    }else 
      return false;
  }


  osg::Vec3 Primitive::toLocal(const osg::Vec3& pos) const {
    const osg::Matrix& m = osg::Matrix::inverse(getPose());
    return pos*m;
//     osg::Vec4 p(pos,1);    
//     const osg::Vec4& pl = p*m;    
//     // one should only use the transpose here, but osg does not have it!
//     return osg::Vec3(pl.x(),pl.y(), pl.z());
  }

  osg::Vec4 Primitive::toLocal(const osg::Vec4& v) const {
    osg::Matrix m = getPose();
    return v*osg::Matrix::inverse(m);
  }

  osg::Vec3 Primitive::toGlobal(const osg::Vec3& pos) const {
    return pos*getPose();
  }

  osg::Vec4 Primitive::toGlobal(const osg::Vec4& v) const {
    return v*getPose();
  }

  void Primitive::setSubstance(Substance substance) {
    this->substance = substance;
    substanceManuallySet = true;
  }

  bool Primitive::store(FILE* f) const {
    const osg::Matrix& pose  = getPose();
    const Pos& vel = getVel();  
    const Pos& avel = getAngularVel();  
    
    if ( fwrite ( pose.ptr() , sizeof ( osg::Matrix::value_type), 16, f ) == 16 )
      if( fwrite ( vel.ptr() , sizeof ( Pos::value_type), 3, f ) == 3 )
        if( fwrite ( avel.ptr() , sizeof ( Pos::value_type), 3, f ) == 3 )
          return true;
    return false;
  }
  
  bool Primitive::restore(FILE* f){
    osg::Matrix pose;
    Pos vel;
    Pos avel;
    
    if ( fread ( pose.ptr() , sizeof ( osg::Matrix::value_type), 16, f ) == 16 )
      if( fread ( vel.ptr() , sizeof ( Pos::value_type), 3, f ) == 3 )
        if( fread ( avel.ptr() , sizeof ( Pos::value_type), 3, f ) == 3 ){
          setPose(pose);
          if(body){
            dBodySetLinearVel(body,vel.x(),vel.y(),vel.z());     
            dBodySetAngularVel(body,avel.x(),avel.y(),avel.z());     
          }
          return true;
        }        
    fprintf ( stderr, "Primitve::restore: cannot read primitive from data\n" );
    return false;
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
    QMP_CRITICAL(0);
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetBox(&m,1,1000,1000,0.01); // fake the mass of the plane with a thin box
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }
    if(mode & Geom){
      geom = dCreatePlane ( odeHandle.space , 0 , 0 , 1 , 0 );      
      attachGeomAndSetColliderFlags();
    }
    if(mode & Draw){
      osgplane->init(osgHandle);
    }
    QMP_END_CRITICAL(0);
  }

  void Plane:: update(){
    if(mode & Draw) {
      if(body)
	osgplane->setMatrix(osgPose(body));
      else 
	osgplane->setMatrix(osgPose(geom));
    }
  }

  void Plane::setMass(double mass){
    dMass m;
    dMassSetBox(&m,1,1000,1000,0.01); // fake the mass of the plane with a thin box
    dMassAdjust (&m, mass); 
    dBodySetMass (body,&m); //assign the mass to the body
  }


  /******************************************************************************/
  Box::Box(float lengthX, float lengthY, float lengthZ) {
    osgbox = new OSGBoxTex(lengthX, lengthY, lengthZ);    
  }

  Box::~Box(){
    if(osgbox) delete osgbox; 
  }

  OSGPrimitive* Box::getOSGPrimitive() { return osgbox; }

  void Box::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		 char mode) {
    assert((mode & Body) || (mode & Geom));
    if (!substanceManuallySet)
      substance = odeHandle.substance;
    QMP_CRITICAL(1);
    this->mode=mode;
    osg::Vec3 dim = osgbox->getDim();
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetBox(&m, 1, dim.x() , dim.y() , dim.z()); 
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body      
    }  
    if (mode & Geom){    
      geom = dCreateBox ( odeHandle.space , dim.x() , dim.y() , dim.z());
      attachGeomAndSetColliderFlags();
    }
    if (mode & Draw){
      osgbox->init(osgHandle);      
    }
    QMP_END_CRITICAL(1);
  }

  void Box:: update(){
    if(mode & Draw) {
      if(body)
	osgbox->setMatrix(osgPose(body));
      else
	osgbox->setMatrix(osgPose(geom));
    }
  }

  void Box::setMass(double mass){
    dMass m;
    osg::Vec3 dim = osgbox->getDim();    
    dMassSetBox(&m, 1, dim.x() , dim.y() , dim.z()); 
    dMassAdjust (&m, mass); 
    dBodySetMass (body,&m); //assign the mass to the body      
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
    if (!substanceManuallySet)
      substance = odeHandle.substance;
    this->mode=mode;
    QMP_CRITICAL(2);
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetSphere(&m, 1, osgsphere->getRadius());
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }  
    if (mode & Geom){
      geom = dCreateSphere ( odeHandle.space , osgsphere->getRadius());
      attachGeomAndSetColliderFlags();
    }
    if (mode & Draw){
      osgsphere->init(osgHandle);      
    }
    QMP_END_CRITICAL(2);
  }

  void Sphere::update(){
    if(mode & Draw) {
      if(body)
	osgsphere->setMatrix(osgPose(body));
      else 
	osgsphere->setMatrix(osgPose(geom));    
    }
  }

  void Sphere::setMass(double mass){
    dMass m;
    dMassSetSphere(&m, 1, osgsphere->getRadius());
    dMassAdjust (&m, mass); 
    dBodySetMass (body,&m); //assign the mass to the body      
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
    if (!substanceManuallySet)
      substance = odeHandle.substance;
    this->mode=mode;
    QMP_CRITICAL(3);
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetCapsule(&m, 1.0, 3 , osgcapsule->getRadius(), osgcapsule->getHeight()); 
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }  
    if (mode & Geom){    
      geom = dCreateCCylinder ( odeHandle.space , osgcapsule->getRadius(), osgcapsule->getHeight());
      attachGeomAndSetColliderFlags();
    }
    if (mode & Draw){
      osgcapsule->init(osgHandle);
    }
    QMP_END_CRITICAL(3);
  }

  void Capsule::update(){
    if(mode & Draw) {
      if(body)
	osgcapsule->setMatrix(osgPose(body));
      else 
	osgcapsule->setMatrix(osgPose(geom));
    }
  }

  void Capsule::setMass(double mass){
    dMass m;
    dMassSetCapsule(&m, 1.0, 3 , osgcapsule->getRadius(), osgcapsule->getHeight()); 
    dMassAdjust (&m, mass); 
    dBodySetMass (body,&m); //assign the mass to the body      
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
    if (!substanceManuallySet)
      substance = odeHandle.substance;
    this->mode=mode;
    QMP_CRITICAL(4);
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      dMass m;
      dMassSetCylinder(&m, 1.0, 3 , osgcylinder->getRadius(), osgcylinder->getHeight()); 
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    }  
    if (mode & Geom){    
      geom = dCreateCylinder ( odeHandle.space , osgcylinder->getRadius(), osgcylinder->getHeight());
      attachGeomAndSetColliderFlags();
    }
    if (mode & Draw){
      osgcylinder->init(osgHandle);
    }
    QMP_END_CRITICAL(4);
  }

  void Cylinder::update(){
    if(mode & Draw) {
      if(body)
	osgcylinder->setMatrix(osgPose(body));
      else 
	osgcylinder->setMatrix(osgPose(geom));
    }
  }

  void Cylinder::setMass(double mass){
    dMass m;
    dMassSetCylinder(&m, 1.0, 3 , osgcylinder->getRadius(), osgcylinder->getHeight());
    dMassAdjust (&m, mass); 
    dBodySetMass (body,&m); //assign the mass to the body      
  }

  /******************************************************************************/
  Ray::Ray(double range, float thickness, float length)
    : range(range), thickness(thickness), length(length)
  {    
    osgbox = new OSGBox(thickness, thickness, length);
  }

  Ray::~Ray(){
    if(osgbox) delete osgbox; 
  }

  OSGPrimitive* Ray::getOSGPrimitive() { return osgbox; }

  void Ray::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		 char mode) {
    assert(!(mode & Body) && (mode & Geom));
    if (!substanceManuallySet)
      substance = odeHandle.substance;
    this->mode=mode;
    QMP_CRITICAL(5);
    geom = dCreateRay ( odeHandle.space, range);
    attachGeomAndSetColliderFlags();
    
    if (mode & Draw){
      osgbox->init(osgHandle);      
    }
    QMP_END_CRITICAL(5);
  }
  
  void Ray::setLength(float len){
    length=len;
    if (mode & Draw){
      osgbox->setDim(osg::Vec3(thickness,thickness,length));         
    }
  }

  void Ray::update(){
    if(mode & Draw) {
      osgbox->setMatrix(osg::Matrix::translate(0,0,length/2)*osgPose(geom));
    }
  }

  void Ray::setMass(double mass){
  }




  /******************************************************************************/
  Transform::Transform(Primitive* parent, Primitive* child, const osg::Matrix& pose)
    : parent(parent), child(child), pose(pose) {
  }

  Transform::~Transform(){
    if(child)
      delete child;
  }

  OSGPrimitive* Transform::getOSGPrimitive() { return 0; }

  void Transform::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		       char mode) {
    // Primitive::body is ignored (removed) from mode
    assert(parent && parent->getBody() != 0 && child); // parent and child must exist
    assert(child->getBody() == 0 && child->getGeom() == 0); // child should not be initialised    
    if (!substanceManuallySet)
      substance = odeHandle.substance;

    QMP_CRITICAL(6);
    // our own geom is just a transform
    geom = dCreateGeomTransform(odeHandle.space);
    dGeomTransformSetInfo(geom, 1);
    dGeomTransformSetCleanup(geom, 0);

    // the child geom must go into space 0 (because it inherits the space from the transform geom)
    OdeHandle odeHandleChild(odeHandle);
    odeHandleChild.space = 0;
    // the root node for the child is the transform node of the parent
    OsgHandle osgHandleChild(osgHandle);
    osgHandleChild.parent = parent->getOSGPrimitive()->getTransform();
    assert(osgHandleChild.scene);
    // initialise the child
    child->init(odeHandleChild, mass, osgHandleChild, (mode & ~Primitive::Body) | Primitive::Child );
    // move the child to the right place (in local coordinates)
    child->setPose(pose);
  
    // assoziate the child with the transform geom
    dGeomTransformSetGeom (geom, child->getGeom());
    // finally bind the transform the body of parent
    dGeomSetBody (geom, parent->getBody());    
    dGeomSetData(geom, (void*)this); // set primitive as geom data
    QMP_END_CRITICAL(6);
  }

  void Transform::update(){
    if(child) child->update();
  }
    
  void Transform::setMass(double mass){
    child->setMass(mass);
  }

  /******************************************************************************/

  Mesh::Mesh(const std::string& filename,float scale) :
    filename(filename), scale(scale), boundshape(0) {
    osgmesh = new OSGMesh(filename, scale);
  }

  Mesh::~Mesh(){
    if(osgmesh) delete osgmesh; 
  }

  OSGPrimitive* Mesh::getOSGPrimitive() { return osgmesh; }

  void Mesh::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
		     char mode) {
    // 20100307; guettler: sometimes the Geom is created later (XMLBoundingShape),
    // if no body is created, this Mesh seems to be static. Then the BoundingShape must not attach
    // any Primitive to the body of the Mesh by a Transform.
    //assert(mode & Body || mode & Geom);
    if (!substanceManuallySet)
      substance = odeHandle.substance;
    this->mode=mode;
    double r=0.01;
    QMP_CRITICAL(7);
    if (mode & Draw){
      osgmesh->init(osgHandle);
      r =  osgmesh->getRadius();
    }
    else {
      osgmesh->virtualInit(osgHandle);
    }
    if (r<0) r=0.01;
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      // Todo: use compound bounding box mass instead
      dMass m;
      dMassSetSphere(&m, 1, r); // we use a sphere
      dMassAdjust (&m, mass); 
      dBodySetMass (body,&m); //assign the mass to the body
    } 
    // read boundingshape file
    //    const osg::BoundingSphere& bsphere = osgmesh->getGroup()->getBound(); 
    // 20100307; guettler: if no Geom, don't create any Geom or Boundings (this is used e.g. for Meshes loaded from XML)
    if (mode & Geom) {
      short drawBoundingMode;
      if (osgHandle.drawBoundings)
        drawBoundingMode=Primitive::Geom | Primitive::Draw;
      else
        drawBoundingMode=Primitive::Geom;
      boundshape = new BoundingShape(filename+".bbox" ,this);
      if(!boundshape->init(odeHandle, osgHandle.changeColor(Color(1,0,0,0.3)), scale, drawBoundingMode)){
        printf("use default bounding box, because bbox file not found!\n");
        Primitive* bound = new Sphere(r);
        Transform* trans = new Transform(this,bound,osg::Matrix::translate(0.0f,0.0f,0.0f));
        trans->init(odeHandle, 0, osgHandle.changeColor(Color(1,0,0,0.3)),drawBoundingMode);
        osgmesh->setMatrix(osg::Matrix::translate(0.0f,0.0f,osgmesh->getRadius())*getPose()); // set obstacle higher
      }
    }
    QMP_END_CRITICAL(7);
  }

  void Mesh::setBoundingShape(BoundingShape* boundingShape) {
    if (boundshape)
      delete boundshape;
    boundshape = boundingShape;
  }

  void Mesh::setPose(const osg::Matrix& pose){
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
     } else
       poseWithoutBodyAndGeom = osg::Matrix(pose);
     update(); // update the scenegraph stuff
   }


  float Mesh::getRadius() { return osgmesh->getRadius(); }

  void Mesh::update(){
    if(mode & Draw) {
      if(body) {
        osgmesh->setMatrix(osgPose(body));
      }
      else if (geom) {
        osgmesh->setMatrix(osgPose(geom));
      }
      else {
        osgmesh->setMatrix(poseWithoutBodyAndGeom);
        boundshape->setPose(poseWithoutBodyAndGeom);
      }
    }
  }

  void Mesh::setMass(double mass){
    // we should use the bouding box here
    dMass m;
    dMassSetSphere(&m, 1, osgmesh->getRadius()); // we use a sphere
    dMassAdjust (&m, mass); 
    dBodySetMass (body,&m); //assign the mass to the body
  }

}
