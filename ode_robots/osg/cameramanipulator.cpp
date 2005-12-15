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
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.3  2005-12-15 17:03:42  martius
 *   cameramanupulator setPose is working
 *   joints have setter and getter parameters
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them
 *
 *   Revision 1.1.2.2  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.1  2005/12/09 16:56:21  martius
 *   camera is working now
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/

#include <osg/Notify>
#include "cameramanipulator.h"
#include "mathutils.h"
#include "pos.h"

namespace lpzrobots {

  using namespace osg;
  using namespace osgGA;

  CameraManipulator::CameraManipulator(osg::Node* node)
    : node(node), eye(0,0,0), view(0,0,0) {
    if (this->node.get()) {    
      const BoundingSphere& boundingSphere=this->node->getBound();
      modelScale = boundingSphere._radius;
    }else 
      modelScale = 0.01f;
  }

  CameraManipulator::~CameraManipulator(){
  }

  void CameraManipulator::setNode(Node* node){
    // we do not support this since we give it manually to the constructor
  }
  const Node* CameraManipulator::getNode() const{
    return node.get();
  }

  Node* CameraManipulator::getNode(){
    return node.get();
  }

  void CameraManipulator::home(const GUIEventAdapter& ea,GUIActionAdapter& us){
    if(node.get()) {
      const BoundingSphere& boundingSphere=node->getBound();

      eye = boundingSphere._center+
	Vec3(-boundingSphere._radius*1.2f,0, boundingSphere._radius*0.2f);

      view = Vec3(-90,-10,0);
      computeMatrix();

      us.requestRedraw();

      us.requestWarpPointer((ea.getXmin()+ea.getXmax())/2.0f,(ea.getYmin()+ea.getYmax())/2.0f);

      flushMouseEventStack();
    }
  }

  void CameraManipulator::init(const GUIEventAdapter& ea,GUIActionAdapter& us){
    flushMouseEventStack();

    us.requestContinuousUpdate(false);

    if (ea.getEventType()!=GUIEventAdapter::RESIZE)
      {
        us.requestWarpPointer((ea.getXmin()+ea.getXmax())/2.0f,(ea.getYmin()+ea.getYmax())/2.0f);
      }
  }
  bool CameraManipulator::handle(const GUIEventAdapter& ea,GUIActionAdapter& us){
    switch(ea.getEventType())
      {
      case(GUIEventAdapter::PUSH): 
	{
	  flushMouseEventStack();
	  return true;
	}
         
      case(GUIEventAdapter::RELEASE):
	{
	  flushMouseEventStack();
	  return true;
	}
         
      case(GUIEventAdapter::DRAG):
        {
	  addMouseEvent(ea);
	  us.requestContinuousUpdate(true);
	  if (calcMovement()) us.requestRedraw();
	  return true;
        }

      case(GUIEventAdapter::KEYDOWN):
	switch(ea.getKey()) {
	case ' ':	
	  {
	    flushMouseEventStack();
	    home(ea,us);
	    us.requestRedraw();
	    us.requestContinuousUpdate(false);
	    break;
	  }
	case 'v':
	  {
	    printf("Camera Position: (%g, %g, %g)", eye.x(), eye.y(), eye.z());
	    printf(" Rotation: (%g, %g, %g)\n", view.x(), view.y(), view.z());
	    break;
	  }	  
	default:
	  return false;
	}
      case(GUIEventAdapter::RESIZE):
	init(ea,us);
	us.requestRedraw();
	return true;
	
      default:
	return false;
      }
    return true;
  }
  
  void CameraManipulator::getUsage(ApplicationUsage& usage) const{
    usage.addKeyboardMouseBinding("Static: Space","Reset the viewing position to home");
  }

  void CameraManipulator::flushMouseEventStack(){
    event_old = NULL;
    event = NULL;
  }
  void CameraManipulator::addMouseEvent(const GUIEventAdapter& ea){
    event_old = event;
    event = &ea;
  }

  void CameraManipulator::setByMatrix(const Matrixd& matrix){
    eye = matrix.getTrans();
    Vec3 xaxis(1,0,0);    
    Pos head = Matrix::transform3x3(xaxis, matrix);
    view.x() = RadiansToDegrees(getAngle(xaxis, head)) *       
      sign(head.y()); // this resolves the ambiguity of getAngle

    Pos tilt = Matrix::transform3x3(Vec3(0,1,0), matrix);
    tilt.print();
    view.y() = RadiansToDegrees(getAngle(Vec3(0,0,1), tilt)) * 
      -1 * sign(tilt.y()); // this resolves the ambiguity of getAngle    
    computeMatrix();    
  }

  Matrixd CameraManipulator::getMatrix() const{
    return pose;
  }

  Matrixd CameraManipulator::getInverseMatrix() const{
    return Matrixd::inverse(pose);
  }

  void CameraManipulator::computeMatrix(){
    for (int i=0; i<3; i++) {
      while (view[i] > 180) view[i] -= 360;
      while (view[i] < -180) view[i] += 360;
    }
    osg::Matrix rot;
    rot.makeRotate( M_PI/2,                          osg::Vec3(1, 0, 0),
		    osg::DegreesToRadians(view.x()), osg::Vec3(0, 0, 1), // heading
		    osg::DegreesToRadians(view.y()), osg::Vec3(cos(osg::DegreesToRadians(view.x())), 
							       sin(osg::DegreesToRadians(view.x())), 
							       0) // pitch
		    );

    pose = rot * Matrix::translate(eye);
  }

  bool CameraManipulator::calcMovement(){
    // _camera->setFusionDistanceMode(Camera::PROPORTIONAL_TO_SCREEN_DISTANCE);

    // return if less then two events have been added.
    if (event.get()==NULL || event_old.get()==NULL) return false;

    //    double dt = event->time()-event_old->time();
    double dx = 10.0*(event->getXnormalized() - event_old->getXnormalized());
    double dy = 10.0*(event->getYnormalized() - event_old->getYnormalized());
    double s =  sin(osg::DegreesToRadians(view.x()));
    double c =  cos(osg::DegreesToRadians(view.x()));

    unsigned int buttonMask = event_old->getButtonMask();
    if (buttonMask==GUIEventAdapter::LEFT_MOUSE_BUTTON) {
      view.x() += dx*3.0f;
      view.y() -= dy*3.0f;
    } else if (buttonMask==GUIEventAdapter::MIDDLE_MOUSE_BUTTON ||
	       buttonMask==(GUIEventAdapter::LEFT_MOUSE_BUTTON | GUIEventAdapter::RIGHT_MOUSE_BUTTON)) { 
      eye.z() += -dy;
      eye.x() += - c*dx;
      eye.y() += - s*dx;
    } else if (buttonMask==GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
      eye.x() +=  s*dy - c*dx;
      eye.y() +=  -c*dy - s*dx;      
    } else return false;
    computeMatrix();
    return true;
  }

}
