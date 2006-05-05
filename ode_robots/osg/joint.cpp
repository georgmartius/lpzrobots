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
 ***************************************************************************
 *                                                                         *
 *  Different Joint wrappers                                               *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.14  2006-05-05 16:07:23  fhesse
 *   dJointSetFixed(joint); added in init of FixedJoint
 *   to remember relative positions
 *
 *   Revision 1.1.2.13  2006/03/29 15:05:57  martius
 *   fixed joint
 *
 *   Revision 1.1.2.12  2006/02/01 18:34:03  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.1.2.11  2006/01/12 22:18:31  martius
 *   delete slider visual
 *
 *   Revision 1.1.2.10  2006/01/12 14:21:00  martius
 *   drawmode, material
 *
 *   Revision 1.1.2.9  2006/01/11 14:11:28  fhesse
 *   moved anchor up into Joint and introduced getAnchor()
 *
 *   Revision 1.1.2.8  2006/01/11 10:53:36  fhesse
 *   delete visual; in SliderJoint::update() removed
 *
 *   Revision 1.1.2.7  2005/12/22 14:09:18  martius
 *   low quality for joint axes
 *
 *   Revision 1.1.2.6  2005/12/21 15:39:22  martius
 *   OneAxisJoint and TwoAxisJoint as superclasses
 *
 *   Revision 1.1.2.5  2005/12/19 16:34:12  martius
 *   added Ball and Universal joint
 *
 *   Revision 1.1.2.4  2005/12/15 17:03:42  martius
 *   cameramanupulator setPose is working
 *   joints have setter and getter parameters
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them
 *
 *   Revision 1.1.2.3  2005/12/14 15:36:45  martius
 *   joints are visible now
 *
 *   Revision 1.1.2.2  2005/12/13 18:11:13  martius
 *   transform primitive added, some joints stuff done, forward declaration
 *
 *   Revision 1.1.2.1  2005/12/12 23:40:42  martius
 *   hinge2 started
 *
 *
 *                                                                 *
 ***************************************************************************/

#include <osg/Vec3>
#include <osg/Matrix>

#include "joint.h"
#include "pos.h"
#include "mathutils.h"


namespace lpzrobots {

  using namespace osg;

  Matrix Joint::anchorAxisPose(const osg::Vec3& anchor, const Axis& axis){
    return rotationMatrixFromAxisZ(axis) * Matrix::translate(anchor);
  }

 Joint::~Joint(){ 
   if (joint) dJointDestroy(joint);
 }


/***************************************************************************/

  
  FixedJoint::FixedJoint(Primitive* part1, Primitive* part2)
  : Joint(part1, part2, osg::Vec3(0,0,0)){
  }

  FixedJoint::~FixedJoint(){}

    /** initialises (and creates) the joint. 
    */
  void FixedJoint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      bool withVisual, double visualSize){
    joint = dJointCreateFixed (odeHandle.world,0);
    dJointAttach (joint, part1->getBody(),part2->getBody()); 
    dJointSetFixed (joint);
  }
 
  void FixedJoint::update(){}

  void FixedJoint::setParam(int parameter, double value) {
  }

  double FixedJoint::getParam(int parameter){
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
			 bool withVisual, double visualSize){
    joint = dJointCreateHinge (odeHandle. world,0);
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

  void HingeJoint::addTorque(double t){
    dJointAddHingeTorque(joint, t);    
  }

  double HingeJoint::getPosition1(){
    return dJointGetHingeAngle(joint);
  }
  
  double HingeJoint::getPosition1Rate(){
    return dJointGetHingeAngleRate(joint);
  }

  void HingeJoint::setParam(int parameter, double value) {
    dJointSetHingeParam(joint, parameter, value);
  }

  double HingeJoint::getParam(int parameter){
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
			 bool withVisual, double visualSize){
    joint = dJointCreateHinge2 (odeHandle. world,0);
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

  /// adds torques to axis 1 and 2
  void Hinge2Joint::addTorques(double t1, double t2){
     dJointAddHinge2Torques(joint, t1, t2); 
  }
  
  double Hinge2Joint::getPosition1(){
    return dJointGetHinge2Angle1(joint);
  }

  double Hinge2Joint::getPosition2(){
    fprintf(stderr, "Hinge2Joint::getPosition2() is called, but not supported!\n");
    return 0;
  }
  
  double Hinge2Joint::getPosition1Rate(){
    return dJointGetHinge2Angle1Rate(joint);
  }
  
  double Hinge2Joint::getPosition2Rate(){
    return dJointGetHinge2Angle2Rate(joint);
  }

  void Hinge2Joint::setParam(int parameter, double value) {
    dJointSetHinge2Param(joint, parameter, value);
  }

  double Hinge2Joint::getParam(int parameter){
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
			 bool withVisual, double visualSize){
    joint = dJointCreateUniversal (odeHandle. world,0);
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

  /// adds torques to axis 1 and 2
  void UniversalJoint::addTorques(double t1, double t2){
    dJointAddUniversalTorques(joint, t1,t2);    
  }

  double UniversalJoint::getPosition1(){
    return dJointGetUniversalAngle1(joint); 
  }

  double UniversalJoint::getPosition2(){
    return dJointGetUniversalAngle2(joint); 
  }

  double UniversalJoint::getPosition1Rate(){
    return dJointGetUniversalAngle1Rate(joint);    
  }

  double UniversalJoint::getPosition2Rate(){
    return dJointGetUniversalAngle2Rate(joint);    
  }

  void UniversalJoint::setParam(int parameter, double value) {
    dJointSetUniversalParam(joint, parameter, value);
  }

  double UniversalJoint::getParam(int parameter){
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
		       bool withVisual, double visualSize){
    joint = dJointCreateBall(odeHandle. world,0);
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

  double BallJoint::getParam(int parameter){
    return 0; // Ball and Socket has no parameter
  }


/***************************************************************************/

  SliderJoint::SliderJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, 
			 const Axis& axis1)
    : OneAxisJoint(part1, part2, anchor, axis1), visual(0) {   
  }

  SliderJoint::~SliderJoint(){
    if (visual) delete visual;
  }

  void SliderJoint::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			 bool withVisual, double visualSize){
    this->osgHandle= osgHandle;
    this->visualSize = visualSize;

    joint = dJointCreateSlider (odeHandle. world,0);
    dJointAttach (joint, part1->getBody(),part2->getBody()); 
    osg::Vec3 p1 = part1->getPosition();
    osg::Vec3 p2 = part2->getPosition();
    anchor = (p1+p2)*0.5;
    dJointSetSliderAxis (joint,  axis1.x(), axis1.y(), axis1.z());
    if(withVisual){
      double len = getPosition1();
      visual = new OSGCylinder(visualSize/10, len+visualSize);
      visual->init(osgHandle, OSGPrimitive::Low);      
      Matrix t = anchorAxisPose(anchor, axis1);
      
      visual->setMatrix(t); 
    }
  }
    
  void SliderJoint::update(){
    if(visual){
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
      visual = new OSGCylinder(visualSize/10, len+visualSize);
      visual->init(osgHandle, OSGPrimitive::Low);      
      Matrix t = anchorAxisPose(anchor, axis1);      
      visual->setMatrix(t); 
    }
  }

  void SliderJoint::addForce(double t){
    dJointAddSliderForce(joint, t);    
  }

  double SliderJoint::getPosition1(){
    return dJointGetSliderPosition(joint);
  }
  
  double SliderJoint::getPosition1Rate(){
    return dJointGetSliderPositionRate(joint);
  }

  void SliderJoint::setParam(int parameter, double value) {
    dJointSetSliderParam(joint, parameter, value);
  }

  double SliderJoint::getParam(int parameter){
    return dJointGetSliderParam(joint, parameter);
  }
    

}
