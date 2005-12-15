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
 *   Revision 1.1.2.4  2005-12-15 17:03:42  martius
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

  Matrix Joint::anchorAxisPose(const osg::Vec3& anchor, const osg::Vec3& axis){
    return rotationMatrixFromAxisZ(axis) * Matrix::translate(anchor);
  }

 Joint::~Joint(){ 
   if (joint) dJointDestroy(joint);
 }

/***************************************************************************/

  HingeJoint::HingeJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, 
			 const Vec3& axis1)
    : Joint(part1, part2), anchor(anchor), axis1(axis1), visual(0) {   
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
      visual->init(osgHandle);      
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

  double HingeJoint::getAngle(){
    return dJointGetHingeAngle(joint);
  }
  
  double HingeJoint::getAngleRate(){
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
			   const Vec3& axis1, const Vec3& axis2)
    : Joint(part1, part2), anchor(anchor), axis1(axis1), axis2(axis2), visual(0) {   
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
      visual->init(osgHandle);      
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

  double Hinge2Joint::getAngle1(){
    return dJointGetHinge2Angle1(joint);
  }
  
  double Hinge2Joint::getAngle1Rate(){
    return dJointGetHinge2Angle1Rate(joint);
  }
  
  double Hinge2Joint::getAngle2Rate(){
    return dJointGetHinge2Angle2Rate(joint);
  }

  void Hinge2Joint::setParam(int parameter, double value) {
    dJointSetHinge2Param(joint, parameter, value);
  }

  double Hinge2Joint::getParam(int parameter){
    return dJointGetHinge2Param(joint, parameter);
  }

/***************************************************************************/

//  BallJoint

}
