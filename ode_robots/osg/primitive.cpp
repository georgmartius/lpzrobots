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

  /** counts number of max velocity violations at joints
   * (Attention, this is a global variable, initialized to 0 at start)
   */
  int globalNumVelocityViolations = 0;


  // returns the osg (4x4) pose matrix of the ode geom
  Pose osgPose( dGeomID geom ){
    return osgPose(dGeomGetPosition(geom), dGeomGetRotation(geom));
  }

  // returns the osg (4x4) pose matrix of the ode body
  Pose osgPose( dBodyID body ){
    return osgPose(dBodyGetPosition(body), dBodyGetRotation(body));
  }

  // converts a position vector and a rotation matrix from ode to osg 4x4 matrix
  Pose osgPose( const double * V , const double * R ){
    return Pose( R[0], R[4], R[8],  0,
                 R[1], R[5], R[9],  0,
                 R[2], R[6], R[10], 0,
                 V[0], V[1], V[2] , 1);
  }

  // converts a osg 4x4 matrix to an ode version of it
  void odeRotation( const Pose& pose , dMatrix3& odematrix){
    osg::Quat q;
    pose.get(q);
    // THIS should be
    //    dQuaternion quat = {q.x(), q.y(), q.z(), q.w() };
    dQuaternion quat = {q.w(), q.x(), q.y(), q.z() };
    dQtoR(quat, odematrix);
  }


  /******************************************************************************/

  Primitive::Primitive()
    : geom(0), body(0), mode(0), substanceManuallySet(false), numVelocityViolations(0) {
  }

  Primitive::~Primitive () {
    QMP_CRITICAL(8);
    // 20091023; guettler:
    // hack for tasked simulations; there are some problems if running in parallel mode,
    // if you do not destroy the geom, everything is fine (should be no problem because world is destroying geoms too)
    if(destroyGeom && geom) dGeomDestroy( geom );
    if(body && ((mode & _Transform) == 0) ) dBodyDestroy( body );
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
    if(mode & _Child){ // in case of a child object it is always dynamic
      dGeomSetCategoryBits (geom, Dyn);
      dGeomSetCollideBits (geom, ~0x0); // collides with everything
    }
    dGeomSetData(geom, (void*)this); // set primitive as geom data
  }


  void Primitive::setColor(const Color& color){
    if(getOSGPrimitive())
      getOSGPrimitive()->setColor(color);
  }

  void Primitive::setColor(const std::string& color){
    if(getOSGPrimitive())
      getOSGPrimitive()->setColor(color);
  }


  void Primitive::setTexture(const std::string& filename){
    if(getOSGPrimitive())
      getOSGPrimitive()->setTexture(filename);
  }

  void Primitive::setTexture(const TextureDescr& texture){
    if(getOSGPrimitive())
      getOSGPrimitive()->setTexture(texture);
  }

  void Primitive::setTexture(int surface, const TextureDescr& texture){
    if(getOSGPrimitive())
      getOSGPrimitive()->setTexture(surface, texture);
  }

  void Primitive::setTextures(const std::vector<TextureDescr>& textures){
    if(getOSGPrimitive())
      getOSGPrimitive()->setTextures(textures);
  }

  void Primitive::setPosition(const Pos& pos){
    if(body){
      dBodySetPosition(body, pos.x(), pos.y(), pos.z());
    }else if(geom){ // okay there is just a geom no body
      dGeomSetPosition(geom, pos.x(), pos.y(), pos.z());
    }
    update(); // update the scenegraph stuff
  }

  void Primitive::setPose(const Pose& pose){
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
    } else {
      assert(0 && "Call setPose only after initialization");
    }
    update(); // update the scenegraph stuff
  }

  Pos Primitive::getPosition() const {
    if(geom) return Pos(dGeomGetPosition(geom));
    else if(body) return Pos(dBodyGetPosition(body));
    else return Pos(0,0,0);
  }

  Pose Primitive::getPose() const {
    if(!geom) {
      if (!body)
        return Pose::translate(0.0f,0.0f,0.0f); // fixes init bug
      else
        return osgPose(dBodyGetPosition(body), dBodyGetRotation(body));
    }
    return osgPose(dGeomGetPosition(geom), dGeomGetRotation(geom));
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
      numVelocityViolations++;
      globalNumVelocityViolations++;
      double scaling = sqrt(vellen)/maxVel;
      dBodySetLinearVel(body, vel[0]/scaling, vel[1]/scaling, vel[2]/scaling);
      return true;
    }else
      return false;
  }

  bool Primitive::limitAngularVel(double maxVel){
    // check for maximum speed:
    if(!body) return false;
    const double* vel = dBodyGetAngularVel( body );
    double vellen = vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2];
    if(vellen > maxVel*maxVel){
      fprintf(stderr, ".");
      numVelocityViolations++;
      globalNumVelocityViolations++;
      double scaling = sqrt(vellen)/maxVel;
      dBodySetAngularVel(body, vel[0]/scaling, vel[1]/scaling, vel[2]/scaling);
      return true;
    }else
      return false;
  }


  void Primitive::decellerate(double factorLin, double factorAng){
    if(!body) return;
    Pos vel;
    if(factorLin!=0){
      vel = getVel();
      applyForce(vel*(-factorLin));
    }
    if(factorAng!=0){
      vel = getAngularVel();
      applyTorque(vel*(-factorAng));
    }
  }



  osg::Vec3 Primitive::toLocal(const osg::Vec3& pos) const {
    const Pose& m = Pose::inverse(getPose());
    return pos*m;
//     osg::Vec4 p(pos,1);
//     const osg::Vec4& pl = p*m;
//     // one should only use the transpose here, but osg does not have it!
//     return osg::Vec3(pl.x(),pl.y(), pl.z());
  }

  osg::Vec4 Primitive::toLocal(const osg::Vec4& v) const {
    Pose m = getPose();
    return v*Pose::inverse(m);
  }

  osg::Vec3 Primitive::toGlobal(const osg::Vec3& pos) const {
    return pos*getPose();
  }

  osg::Vec4 Primitive::toGlobal(const osg::Vec4& v) const {
    return v*getPose();
  }

  void Primitive::setSubstance(const Substance& substance) {
    this->substance = substance;
    substanceManuallySet = true;
  }

  bool Primitive::store(FILE* f) const {
    const Pose& pose  = getPose();
    const Pos& vel = getVel();
    const Pos& avel = getAngularVel();

    if ( fwrite ( pose.ptr() , sizeof ( Pose::value_type), 16, f ) == 16 )
      if( fwrite ( vel.ptr() , sizeof ( Pos::value_type), 3, f ) == 3 )
        if( fwrite ( avel.ptr() , sizeof ( Pos::value_type), 3, f ) == 3 )
          return true;
    return false;
  }

  bool Primitive::restore(FILE* f){
    Pose pose;
    Pos vel;
    Pos avel;

    if ( fread ( pose.ptr() , sizeof ( Pose::value_type), 16, f ) == 16 )
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
      setMass(mass, mode & Density);
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

  void Plane::setMass(double mass, bool density){
    if(body){
      dMass m;
      dMassSetBox(&m,mass,1000,1000,0.01); // fake the mass of the plane with a thin box
      if(!density)
        dMassAdjust (&m, mass);
      dBodySetMass (body,&m); //assign the mass to the body
    }
  }


  /******************************************************************************/
  Box::Box(float lengthX, float lengthY, float lengthZ) {
    osgbox = new OSGBoxTex(lengthX, lengthY, lengthZ);
  }

  Box::Box(const osg::Vec3& dim) {
    osgbox = new OSGBoxTex(dim);
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
      setMass(mass, mode & Density);
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

  void Box::setMass(double mass, bool density){
    if(body){
      dMass m;
      osg::Vec3 dim = osgbox->getDim();
      dMassSetBox(&m, mass, dim.x() , dim.y() , dim.z());
      if(!density)
        dMassAdjust (&m, mass);
      dBodySetMass (body,&m); //assign the mass to the body
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
    if (!substanceManuallySet)
      substance = odeHandle.substance;
    this->mode=mode;
    QMP_CRITICAL(2);
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      setMass(mass, mode & Density);
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

  void Sphere::setMass(double mass, bool density){
    if(body){
      dMass m;
      dMassSetSphere(&m, mass, osgsphere->getRadius());
      if(!density)
        dMassAdjust (&m, mass);
      dBodySetMass (body,&m); //assign the mass to the body
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
    if (!substanceManuallySet)
      substance = odeHandle.substance;
    this->mode=mode;
    QMP_CRITICAL(3);
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      setMass(mass, mode & Density);
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

  void Capsule::setMass(double mass, bool density){
    if(mass){
      dMass m;
      dMassSetCapsule(&m, mass, 3 , osgcapsule->getRadius(), osgcapsule->getHeight());
      if(!density)
        dMassAdjust (&m, mass);
      dBodySetMass (body,&m); //assign the mass to the body
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
    if (!substanceManuallySet)
      substance = odeHandle.substance;
    this->mode=mode;
    QMP_CRITICAL(4);
    if (mode & Body){
      body = dBodyCreate (odeHandle.world);
      setMass(mass, mode & Density);
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

  void Cylinder::setMass(double mass, bool density){
    if(body){
      dMass m;
      dMassSetCylinder(&m, mass, 3 , osgcylinder->getRadius(), osgcylinder->getHeight());
      if(!density)
        dMassAdjust (&m, mass);
      dBodySetMass (body,&m); //assign the mass to the body
    }
  }

  /******************************************************************************/
  Ray::Ray(double range, float thickness, float length)
    : range(range), thickness(thickness), length(length)
  {
    if(thickness==0){
      std::list<osg::Vec3> pnts;
      pnts.push_back(osg::Vec3(0,0,-length/2));
      pnts.push_back(osg::Vec3(0,0,length/2));
      osgprimitive = new OSGLine(pnts);
    } else
      osgprimitive = new OSGBox(thickness, thickness, length);
  }

  Ray::~Ray(){
    if(osgprimitive) delete osgprimitive;
  }

  OSGPrimitive* Ray::getOSGPrimitive() { return osgprimitive; }

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
      osgprimitive->init(osgHandle);
    }
    QMP_END_CRITICAL(5);
  }

  void Ray::setLength(float len){
    length=len;
    if (mode & Draw){
      OSGBox* b = dynamic_cast<OSGBox*>(osgprimitive);
      if(b)
        b->setDim(osg::Vec3(thickness,thickness,length));
      else{
        OSGLine* l = dynamic_cast<OSGLine*>(osgprimitive);
        std::list<osg::Vec3> pnts;
        pnts.push_back(osg::Vec3(0,0,-length/2));
        pnts.push_back(osg::Vec3(0,0,length/2));
        l->setPoints(pnts);
      }
    }
  }

  void Ray::update(){
    if(mode & Draw) {
      osgprimitive->setMatrix(Pose::translate(0,0,length/2)*osgPose(geom));
    }
  }

  void Ray::setMass(double mass, bool density){
  }




  /******************************************************************************/
  Transform::Transform(Primitive* parent, Primitive* child, const Pose& pose, bool deleteChild)
    : parent(parent), child(child), pose(pose), deleteChild(deleteChild) {
  }

  Transform::~Transform(){
    if(child && deleteChild)
      delete child;
  }

  OSGPrimitive* Transform::getOSGPrimitive() { return 0; }

  void Transform::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
                       char mode) {
    // Primitive::body is ignored (removed) from mode
    assert(parent && parent->getBody() != 0 && child); // parent and child must exist
    assert(child->getBody() == 0 && child->getGeom() == 0); // child should not be initialised
    this->mode = mode | Primitive::_Transform;
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
    child->init(odeHandleChild, mass, osgHandleChild, (mode & ~Primitive::Body) | Primitive::_Child );
    // move the child to the right place (in local coordinates)
    child->setPose(pose);

    // assoziate the child with the transform geom
    dGeomTransformSetGeom (geom, child->getGeom());
    // finally bind the transform the body of parent
    dGeomSetBody (geom, parent->getBody());
    dGeomSetData(geom, (void*)this); // set primitive as geom data

    // we assign the body here. Since our mode is Transform it is not destroyed
    body=parent->getBody();

    QMP_END_CRITICAL(6);
  }

  void Transform::update(){
    if(child) child->update();
  }

  void Transform::setMass(double mass, bool density){
    child->setMass(mass, density);
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
      setMass(mass, mode & Density);
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
        Transform* trans = new Transform(this,bound,Pose::translate(0.0f,0.0f,0.0f));
        trans->init(odeHandle, 0, osgHandle.changeColor(Color(1,0,0,0.3)),drawBoundingMode);
        osgmesh->setMatrix(Pose::translate(0.0f,0.0f,osgmesh->getRadius())*getPose()); // set obstacle higher
      }
    }
    QMP_END_CRITICAL(7);
  }

  void Mesh::setBoundingShape(BoundingShape* boundingShape) {
    if (boundshape)
      delete boundshape;
    boundshape = boundingShape;
  }

  void Mesh::setPose(const Pose& pose){
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
       poseWithoutBodyAndGeom = Pose(pose);
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

  void Mesh::setMass(double mass, bool density){
    if(body){
      // we should use the bouding box here
      dMass m;
      dMassSetSphere(&m, mass, osgmesh->getRadius()); // we use a sphere
      if(!density)
        dMassAdjust (&m, mass);
      dBodySetMass (body,&m); //assign the mass to the body
    }
  }

}
