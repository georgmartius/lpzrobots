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

//TODO: add dParamFudgeFactor, see ode docs (0.5 maybe?)

#include <osg/Vec3>
#include <osg/Matrix>

#include "joint.h"
#include "pos.h"
#include "mathutils.h"
#include "odehandle.h"
#include "osgprimitive.h"

namespace lpzrobots {

  using namespace osg;

  Matrix Joint::anchorAxisPose(const osg::Vec3& anchor, const Axis& axis){
    return rotationMatrixFromAxisZ(axis) * Matrix::translate(anchor);
  }

  Joint::~Joint(){
    setFeedBackMode(false);
    if (joint) dJointDestroy(joint);
    if(part1->getGeom() && part2->getGeom()){
      odeHandle.removeIgnoredPair(part1->getGeom(), part2->getGeom());
    }
  }

  void Joint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                   bool withVisual, double visualSize, bool ignoreColl){
    this->odeHandle = odeHandle;
    // only part 2 is allowed to be a static object
    assert(part1->getBody() && "MSG: only part2 can be static");
    if(ignoreColl && part1->getGeom() && part2->getGeom())
      this->odeHandle.addIgnoredPair(part1->getGeom(), part2->getGeom());
  }


  void Joint::setFeedBackMode(bool mode) {
    if (mode){
      if((feedback=dJointGetFeedback(joint))==0){
        feedback = (dJointFeedback*)malloc(sizeof(dJointFeedback));
        dJointSetFeedback (joint, feedback);
      }
    }else{
      if(feedback){
        dJointSetFeedback (joint, 0);
        free(feedback);
      }
    }
  }

  bool Joint::getTorqueFeedback(Pos& t1, Pos& t2) const {
    dJointFeedback* fb = dJointGetFeedback(joint);
    if (!fb) return false;
    t1 = Pos(fb->t1);
    t2 = Pos(fb->t2);
    return true;
  }
  bool Joint::getForceFeedback(Pos& f1, Pos& f2) const {
    dJointFeedback* fb = dJointGetFeedback(joint);
    if (!fb) return false;
    f1 = Pos(fb->f1);
    f2 = Pos(fb->f2);
    return true;
  }


  std::list<double> OneAxisJoint::getPositions() const {
    std::list<double> l;
    l.push_back(getPosition1());
    return l;
  }
  std::list<double> OneAxisJoint::getPositionRates() const {
    std::list<double> l;
    l.push_back(getPosition1Rate());
    return l;
  }

  std::list<double> TwoAxisJoint::getPositions() const {
    std::list<double> l;
    l.push_back(getPosition1());
    l.push_back(getPosition2());
    return l;
  }
  std::list<double> TwoAxisJoint::getPositionRates() const {
    std::list<double> l;
    l.push_back(getPosition1Rate());
    l.push_back(getPosition2Rate());
    return l;
  }

  int OneAxisJoint::getPositions(double* sensorarray) const {
    sensorarray[0] = getPosition1();
    return 1;
  }
  int OneAxisJoint::getPositionRates(double* sensorarray) const{
    sensorarray[0] = getPosition1Rate();
    return 1;
  }

  int TwoAxisJoint::getPositions(double* sensorarray) const {
    sensorarray[0] = getPosition1();
    sensorarray[1] = getPosition2();
    return 2;
  }
  int TwoAxisJoint::getPositionRates(double* sensorarray) const{
    sensorarray[0] = getPosition1Rate();
    sensorarray[1] = getPosition2Rate();
    return 2;
  }



/***************************************************************************/


  FixedJoint::FixedJoint(Primitive* part1, Primitive* part2,
                         const osg::Vec3& anchor)
    : Joint(part1, part2, anchor), visual(0) {
    // anchor is here used as a relative position to part1
      if(anchor.x()!=0 || anchor.y()!=0 || anchor.z()!=0 ){ // some anchor given
        this->anchor = part1->toLocal(anchor);
      } // 0,0,0 at part1 position
  }

  FixedJoint::~FixedJoint(){
    if (visual) delete visual;
  }

    /** initialises (and creates) the joint.
    */
  void FixedJoint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      bool withVisual, double visualSize, bool ignoreColl){
    Joint::init(odeHandle, osgHandle, withVisual, visualSize, ignoreColl);
    joint = dJointCreateFixed (odeHandle.world,0);
    dJointAttach (joint, part1->getBody(),part2->getBody());
    dJointSetFixed (joint);

    if(withVisual){
      visual = new OSGSphere(visualSize/2.0);
      visual->init(osgHandle, OSGPrimitive::Low);

      visual->setMatrix(TRANSM(anchor) * part1->getPose());
    }
  }

  void FixedJoint::update(){
    if(visual){
      visual->setMatrix(TRANSM(anchor) * part1->getPose());
    }
  }

  void FixedJoint::setParam(int parameter, double value) {
  }

  double FixedJoint::getParam(int parameter) const {
    return 0;
  }

/***************************************************************************/

  HingeJoint::HingeJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor,
                         const Axis& axis1)
    : OneAxisJoint(part1, part2, anchor, axis1),  visual(0) {
  }

  HingeJoint::~HingeJoint(){
    if (visual) delete visual;
  }

  void HingeJoint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                         bool withVisual, double visualSize, bool ignoreColl){
    Joint::init(odeHandle, osgHandle, withVisual, visualSize, ignoreColl);
    joint = dJointCreateHinge (odeHandle.world,0);
    dJointAttach (joint, part1->getBody(),part2->getBody());
    dJointSetHingeAnchor (joint, anchor.x(), anchor.y(), anchor.z());
    dJointSetHingeAxis (joint,  axis1.x(), axis1.y(), axis1.z());
    if(withVisual){
      visual = new OSGCylinder(visualSize/15.0, visualSize);
      visual->init(osgHandle, OSGPrimitive::Low);
      Matrix t = anchorAxisPose(anchor, axis1);

      visual->setMatrix(t);
    }
  }

  void HingeJoint::update(){
    if(visual){
      dVector3 v;
      dJointGetHingeAnchor(joint, v);
      anchor.x() = v[0];
      anchor.y() = v[1];
      anchor.z() = v[2];
      dJointGetHingeAxis(joint, v);
      axis1.x() = v[0];
      axis1.y() = v[1];
      axis1.z() = v[2];
      visual->setMatrix(anchorAxisPose(anchor, axis1));
    }
  }

  void HingeJoint::addForce1(double t){
    dJointAddHingeTorque(joint, t);
  }

  double HingeJoint::getPosition1() const{
    return dJointGetHingeAngle(joint);
  }

  double HingeJoint::getPosition1Rate() const{
    return dJointGetHingeAngleRate(joint);
  }

  void HingeJoint::setParam(int parameter, double value) {
    assert(joint); // joint is not initialised
    dJointSetHingeParam(joint, parameter, value);
  }

  double HingeJoint::getParam(int parameter) const{
    assert(joint); // joint is not initialised
    return dJointGetHingeParam(joint, parameter);
  }

/***************************************************************************/

  Hinge2Joint::Hinge2Joint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor,
                           const Axis& axis1, const Axis& axis2)
    : TwoAxisJoint(part1, part2, anchor, axis1, axis2), visual(0) {
  }

  Hinge2Joint::~Hinge2Joint(){
    if (visual) delete visual;
  }

  void Hinge2Joint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                         bool withVisual, double visualSize, bool ignoreColl){
    Joint::init(odeHandle, osgHandle, withVisual, visualSize, ignoreColl);
    joint = dJointCreateHinge2 (odeHandle.world,0);
    dJointAttach (joint, part1->getBody(),part2->getBody());
    dJointSetHinge2Anchor (joint, anchor.x(), anchor.y(), anchor.z());
    dJointSetHinge2Axis1 (joint,  axis1.x(), axis1.y(), axis1.z());
    dJointSetHinge2Axis2 (joint,  axis2.x(), axis2.y(), axis2.z());
    if(withVisual){
      visual = new OSGCylinder(visualSize/15.0, visualSize);
      visual->init(osgHandle, OSGPrimitive::Low);
      Matrix t = anchorAxisPose(anchor, axis2);

      visual->setMatrix(t);
    }
  }

  void Hinge2Joint::update(){
    if(visual){
      dVector3 v;
      dJointGetHinge2Anchor(joint, v);
      anchor.x() = v[0];
      anchor.y() = v[1];
      anchor.z() = v[2];
      dJointGetHinge2Axis2(joint, v);
      axis2.x() = v[0];
      axis2.y() = v[1];
      axis2.z() = v[2];
      visual->setMatrix(anchorAxisPose(anchor, axis2));

    }
  }

  /// add torque to axis 1
  void Hinge2Joint::addForce1(double t1){
     dJointAddHinge2Torques(joint, t1, 0);
  }
  /// add torque to axis 1
  void Hinge2Joint::addForce2(double t2){
     dJointAddHinge2Torques(joint, 0, t2);
  }

  double Hinge2Joint::getPosition1()  const{
    return dJointGetHinge2Angle1(joint);
  }

  double Hinge2Joint::getPosition2() const{
    fprintf(stderr, "Hinge2Joint::getPosition2() is called, but not supported!\n");
    return 0;
  }

  double Hinge2Joint::getPosition1Rate() const{
    return dJointGetHinge2Angle1Rate(joint);
  }

  double Hinge2Joint::getPosition2Rate() const{
    return dJointGetHinge2Angle2Rate(joint);
  }

  void Hinge2Joint::setParam(int parameter, double value) {
    assert(joint); // joint is not initialised
    dJointSetHinge2Param(joint, parameter, value);
  }

  double Hinge2Joint::getParam(int parameter) const{
    assert(joint);// joint is not initialised
    return dJointGetHinge2Param(joint, parameter);
  }

/***************************************************************************/

  UniversalJoint::UniversalJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor,
                           const Axis& axis1, const Axis& axis2)
    : TwoAxisJoint(part1, part2, anchor, axis1, axis2), visual1(0), visual2(0) {
  }

  UniversalJoint::~UniversalJoint(){
    if (visual1) delete visual1;
    if (visual2) delete visual2;
  }

  void UniversalJoint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                         bool withVisual, double visualSize, bool ignoreColl){
    Joint::init(odeHandle, osgHandle, withVisual, visualSize, ignoreColl);
    joint = dJointCreateUniversal (odeHandle.world,0);
    dJointAttach (joint, part1->getBody(),part2->getBody());
    dJointSetUniversalAnchor (joint, anchor.x(), anchor.y(), anchor.z());
    dJointSetUniversalAxis1 (joint,  axis1.x(), axis1.y(), axis1.z());
    dJointSetUniversalAxis2 (joint,  axis2.x(), axis2.y(), axis2.z());
    if(withVisual){
      visual1 = new OSGCylinder(visualSize/15.0, visualSize);
      visual1->init(osgHandle, OSGPrimitive::Low);
      Matrix t = anchorAxisPose(anchor, axis1);
      visual1->setMatrix(t);
      visual2 = new OSGCylinder(visualSize/15.0, visualSize);
      visual2->init(osgHandle, OSGPrimitive::Low);
      t = anchorAxisPose(anchor, axis2);
      visual2->setMatrix(t);
    }
  }

  void UniversalJoint::update(){
    if(visual1 && visual2){
      dVector3 v;
      dJointGetUniversalAnchor(joint, v);
      anchor.x() = v[0];
      anchor.y() = v[1];
      anchor.z() = v[2];
      dJointGetUniversalAxis1(joint, v);
      axis1.x() = v[0];
      axis1.y() = v[1];
      axis1.z() = v[2];
      dJointGetUniversalAxis2(joint, v);
      axis2.x() = v[0];
      axis2.y() = v[1];
      axis2.z() = v[2];
      visual1->setMatrix(anchorAxisPose(anchor, axis1));
      visual2->setMatrix(anchorAxisPose(anchor, axis2));
    }
  }

  void UniversalJoint::addForce1(double t1){
    dJointAddUniversalTorques(joint, t1,0);
  }
  void UniversalJoint::addForce2(double t2){
    dJointAddUniversalTorques(joint, 0,t2);
  }

  double UniversalJoint::getPosition1() const{
    return dJointGetUniversalAngle1(joint);
  }

  double UniversalJoint::getPosition2() const{
    return dJointGetUniversalAngle2(joint);
  }

  double UniversalJoint::getPosition1Rate() const{
    return dJointGetUniversalAngle1Rate(joint);
  }

  double UniversalJoint::getPosition2Rate() const{
    return dJointGetUniversalAngle2Rate(joint);
  }

  void UniversalJoint::setParam(int parameter, double value) {
    dJointSetUniversalParam(joint, parameter, value);
  }

  double UniversalJoint::getParam(int parameter) const{
    return dJointGetUniversalParam(joint, parameter);
  }

/***************************************************************************/

  BallJoint::BallJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor)
    : Joint(part1, part2, anchor), visual(0){

  }

  BallJoint::~BallJoint() {
    if (visual) delete visual;
  }

  void BallJoint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       bool withVisual, double visualSize, bool ignoreColl){
    Joint::init(odeHandle, osgHandle, withVisual, visualSize, ignoreColl);
    joint = dJointCreateBall(odeHandle.world, 0);
    dJointAttach (joint, part1->getBody(),part2->getBody());
    dJointSetBallAnchor (joint, anchor.x(), anchor.y(), anchor.z());
    if(withVisual){
      visual = new OSGSphere(visualSize);
      visual->init(osgHandle, OSGPrimitive::Low);
      visual->setMatrix(osg::Matrix::translate(anchor));
    }
  }

  void BallJoint::update(){
    if(visual){
      dVector3 v;
      dJointGetBallAnchor(joint, v);
      anchor.x() = v[0];
      anchor.y() = v[1];
      anchor.z() = v[2];
      visual->setMatrix(osg::Matrix::translate(anchor));
    }
  }

  // Ball and Socket has no parameter
  void BallJoint::setParam(int parameter, double value) { }

  double BallJoint::getParam(int parameter) const{
    return 0; // Ball and Socket has no parameter
  }


/***************************************************************************/

  SliderJoint::SliderJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor,
                         const Axis& axis1)
    : OneAxisJoint(part1, part2, anchor, axis1), visual(0), visualSize(0) {
  }

  SliderJoint::~SliderJoint(){
    if (visual) delete visual;
  }

  void SliderJoint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                         bool withVisual, double visualSize, bool ignoreColl){
    this->osgHandle= osgHandle;
    this->visualSize = visualSize;
    Joint::init(odeHandle, osgHandle, withVisual, visualSize, ignoreColl);

    joint = dJointCreateSlider (odeHandle.world,0);
    dJointAttach (joint, part1->getBody(),part2->getBody());
//     osg::Vec3 p1 = part1->getPosition();
//     osg::Vec3 p2 = part2->getPosition();
//     anchor = (p1+p2)*0.5;
    dJointSetSliderAxis (joint,  axis1.x(), axis1.y(), axis1.z());
    if(withVisual){
      double len = getPosition1();
      visual = new OSGCylinder(visualSize/10, fabs(len)+visualSize);
      visual->init(osgHandle, OSGPrimitive::Low);
      Matrix t = anchorAxisPose(anchor, axis1);

      visual->setMatrix(t);
    }
  }

  void SliderJoint::update(){
    if(visual){
      // this might create problems because if a slider is connected
      //  to the environment this create a weird position
      osg::Vec3 p1 = part1->getPosition();
      osg::Vec3 p2 = part2->getPosition();
      anchor = (p1+p2)*0.5;
      dVector3 v;
      dJointGetSliderAxis(joint, v);
      axis1.x() = v[0];
      axis1.y() = v[1];
      axis1.z() = v[2];

      double len = getPosition1();
      delete visual;
      visual = new OSGCylinder(visualSize/10, std::max(len+visualSize,0.001));
      visual->init(osgHandle, OSGPrimitive::Low);
      Matrix t = anchorAxisPose(anchor, axis1);
      visual->setMatrix(t);
    }
  }

  void SliderJoint::addForce1(double t){
    dJointAddSliderForce(joint, t);
  }

  double SliderJoint::getPosition1() const{
    return dJointGetSliderPosition(joint);
  }

  double SliderJoint::getPosition1Rate() const{
    return dJointGetSliderPositionRate(joint);
  }

  void SliderJoint::setParam(int parameter, double value) {
    assert(joint); // joint is not initialised
    dJointSetSliderParam(joint, parameter, value);
  }

  double SliderJoint::getParam(int parameter) const{
    assert(joint); // joint is not initialised
    return dJointGetSliderParam(joint, parameter);
  }

  // /***************************************************************************/

  // LMotorJoint::LMotorJoint(Primitive* part1, Primitive* part2, int relative,
  //                          const Axis& axis1, const Axis& axis2)
  //   : TwoAxisJoint(part1, part2, Vec3(0,0,0) , axis1, axis2),
  //     visual1(0), visual2(0), relative(relative) {
  // }

  // LMotorJoint::~LMotorJoint(){
  //   if (visual1) delete visual1;
  //   if (visual2) delete visual2;
  // }

  // void LMotorJoint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
  //                        bool withVisual, double visualSize, bool ignoreColl){
  //   Joint::init(odeHandle, osgHandle, withVisual, visualSize, ignoreColl);
  //   joint = dJointCreateLMotor (odeHandle.world,0);
  //   dJointAttach (joint, part1->getBody(),part2->getBody());
  //   dJointSetLMotorNumAxes(joint,  3);
  //   // both axis are anchored  to first body (but given in global coords)
  //   dJointSetLMotorAxis (joint, 0, relative, axis1.x(), axis1.y(), axis1.z());
  //   dJointSetLMotorAxis (joint, 1, relative, axis2.x(), axis2.y(), axis2.z());
  //   Axis axis3 = axis1.crossProduct(axis2);
  //   dJointSetLMotorAxis (joint, 2, relative, axis3.x(), axis3.y(), axis3.z());
  //   setParam(dParamLoStop2, 0);
  //   setParam(dParamHiStop2, 0);

  //   if(withVisual){
  //     visual1 = new OSGCylinder(visualSize/15.0, visualSize);
  //     visual1->init(osgHandle, OSGPrimitive::Low);
  //     Matrix t = anchorAxisPose(part1->getPosition(), axis1);
  //     visual1->setMatrix(t);
  //     visual2 = new OSGCylinder(visualSize/15.0, visualSize);
  //     visual2->init(osgHandle, OSGPrimitive::Low);
  //     t = anchorAxisPose(part1->getPosition(), axis2);
  //     visual2->setMatrix(t);
  //   }
  // }

  // void LMotorJoint::update(){
  //   if(visual1 && visual2){
  //     dVector3 v;
  //     Vec3 pos = part1->getPosition();
  //     anchor.x() = v[0];
  //     anchor.y() = v[1];
  //     anchor.z() = v[2];
  //     dJointGetLMotorAxis(joint, 0, v);
  //     axis1.x() = v[0];
  //     axis1.y() = v[1];
  //     axis1.z() = v[2];
  //     dJointGetLMotorAxis(joint, 1, v);
  //     axis2.x() = v[0];
  //     axis2.y() = v[1];
  //     axis2.z() = v[2];
  //     visual1->setMatrix(anchorAxisPose(pos, axis1));
  //     visual2->setMatrix(anchorAxisPose(pos, axis2));
  //   }
  // }

  // void LMotorJoint::addForce1(double t1){
  //   assert("not implemented!");
  // }
  // void LMotorJoint::addForce2(double t2){
  //   assert("not implemented!");
  // }

  // double LMotorJoint::getPosition1() const{
  //   assert("not implemented!");
  //   return 0;
  // }

  // double LMotorJoint::getPosition2() const{
  //   assert("not implemented!");
  //   return 0;
  // }

  // double LMotorJoint::getPosition1Rate() const{
  //   assert("not implemented!");
  //   return 0;
  // }

  // double LMotorJoint::getPosition2Rate() const{
  //   assert("not implemented!");
  //   return 0;
  // }

  // void LMotorJoint::setParam(int parameter, double value) {
  //   dJointSetLMotorParam(joint, parameter, value);
  // }

  // double LMotorJoint::getParam(int parameter) const{
  //   return dJointGetLMotorParam(joint, parameter);
  // }


}
