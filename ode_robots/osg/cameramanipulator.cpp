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
 ***************************************************************************
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2006-08-04 15:05:42  martius
 *   documentation
 *
 *   Revision 1.3  2006/08/02 09:43:07  martius
 *   comments
 *
 *   Revision 1.2  2006/07/14 12:23:34  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.17  2006/06/16 22:30:52  martius
 *   removed key debug output
 *
 *   Revision 1.1.2.16  2006/05/29 20:00:49  robot3
 *   added pos1 (center on agent) and end (move behind agent)
 *
 *   Revision 1.1.2.15  2006/04/25 09:06:16  robot3
 *   *** empty log message ***
 *
 *   Revision 1.1.2.14  2006/03/28 09:55:12  robot3
 *   -main: fixed snake explosion bug
 *   -odeconfig.h: inserted cameraspeed
 *   -camermanipulator.cpp: fixed setbyMatrix,
 *    updateFactor
 *
 *   Revision 1.1.2.13  2006/03/19 13:32:48  robot3
 *   race mode now works
 *
 *   Revision 1.1.2.12  2006/03/18 12:03:25  robot3
 *   some prints removed
 *
 *   Revision 1.1.2.11  2006/03/08 13:19:13  robot3
 *   basic modifications, follow mode now works
 *
 *   Revision 1.1.2.10  2006/03/06 16:56:44  robot3
 *   -more stable version
 *   -code optimized
 *   -some static variables used by all cameramanipulators
 *
 *   Revision 1.1.2.9  2006/03/05 15:01:57  robot3
 *   camera moves now smooth
 *
 *   Revision 1.1.2.8  2006/03/04 15:04:33  robot3
 *   cameramanipulator is now updated with every draw intervall
 *
 *   Revision 1.1.2.7  2006/03/03 12:08:50  robot3
 *   preparations made for new cameramanipulators
 *
 *   Revision 1.1.2.6  2006/02/01 10:24:34  robot3
 *   new camera manipulator added
 *
 *   Revision 1.1.2.5  2006/01/30 13:12:45  martius
 *   bug in setByMatrix
 *
 *   Revision 1.1.2.4  2005/12/29 12:55:59  martius
 *   setHome
 *
 *   Revision 1.1.2.3  2005/12/15 17:03:42  martius
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

#include <vector>
#include <iterator>

namespace lpzrobots {
  
  using namespace osg;
  using namespace osgGA;
  
  
  osg::Vec3 CameraManipulator::eye(0,0,0);
  osg::Vec3 CameraManipulator::view(0,0,0);
  osg::Vec3 CameraManipulator::home_eye;
  osg::Vec3 CameraManipulator::home_view;
  osg::Vec3 CameraManipulator::desiredEye;
  osg::Vec3 CameraManipulator::desiredView;
  bool CameraManipulator::home_externally_set=false;
  OdeAgent* CameraManipulator::watchingAgent;
  bool CameraManipulator::watchingAgentDefined=false;
  Position CameraManipulator::oldPositionOfAgent;
  bool CameraManipulator::oldPositionOfAgentDefined=false;

  // globalData braucht er für alles
  CameraManipulator::CameraManipulator(osg::Node* node, GlobalData& global)
    : node(node), globalData(global) {
    if (this->node.get()) {    
      const BoundingSphere& boundingSphere=this->node->getBound();
      modelScale = boundingSphere._radius;
    }else 
      modelScale = 0.01f;
    desiredEye=eye;
    desiredView=view;
    // default values for smoothness (function updte())
    // can be owerwritten by new cameramanipulator if needed
    degreeSmoothness=0.03;
    lengthSmoothness=0.03;
    lengthAccuracy=0.02;
    degreeAccuracy=0.03;
  }

  CameraManipulator::~CameraManipulator(){
  }

  void CameraManipulator::setNode(osg::Node* node){
    // we do not support this since we give it manually to the constructor
  }
  const Node* CameraManipulator::getNode() const{
    return node.get();
  }

  Node* CameraManipulator::getNode(){
    return node.get();
  }

  /// set the home position of the camera. (and place it there)
  void CameraManipulator::setHome(const osg::Vec3& _eye, const osg::Vec3& _view){
    home_eye = _eye;
    home_view = _view;
    home_externally_set=true;
    eye  = home_eye;
    view = home_view;
    desiredEye=_eye;
    desiredView=_view;
    computeMatrix(); // i think we don't need this???
  }


  void CameraManipulator::home(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us){
    if(node.get() && !home_externally_set) {
      const BoundingSphere& boundingSphere=node->getBound();

      home_eye = boundingSphere._center+
	Vec3(-boundingSphere._radius*1.2f,0, boundingSphere._radius*0.2f);

      home_view = Vec3(-90,-10,0);
    }
    //   eye  = home_eye;
    //    view = home_view;
    desiredEye=home_eye;
    desiredView=home_view;
    computeMatrix();
    
    us.requestRedraw();
    
    us.requestWarpPointer((ea.getXmin()+ea.getXmax())/2.0f,(ea.getYmin()+ea.getYmax())/2.0f);
    
    flushMouseEventStack();
    
  }

  void CameraManipulator::init(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us){
    flushMouseEventStack();

    us.requestContinuousUpdate(false);

    if (ea.getEventType()!=GUIEventAdapter::RESIZE)
      {
        us.requestWarpPointer((ea.getXmin()+ea.getXmax())/2.0f,(ea.getYmin()+ea.getYmax())/2.0f);
      }
  }

  bool CameraManipulator::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us){
    int key=0;
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
	key=ea.getKey();
	// F-keys (F1 to F12)
	if ((65470<=key)&&(key<=65481)) {
	  manageAgents(key-65469);
	  return true; // was handled
	}
	//	std::cout << key << " was pressed." << std::endl;
	switch(key) {
	case ' ':	
	  {
	    flushMouseEventStack();
	    home(ea,us);
	    us.requestRedraw();
	    us.requestContinuousUpdate(false);
	    break;
	  }
	case 'p':
	  {
	    printf("Camera Position/View: (Pos(%g, %g, %g), ", eye.x(), eye.y(), eye.z());
	    printf(" Pos(%g, %g, %g));\n", view.x(), view.y(), view.z());
	    break;
	  }	  
	case 65360: // pos1
	  {
	    centerOnAgent();
	    break;
	  }
	case 65367: // end
	  {
	    moveBehindAgent();
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
  
  void CameraManipulator::getUsage(osg::ApplicationUsage& usage) const{
    usage.addKeyboardMouseBinding("Camera: Space","Reset the viewing position to home");
    usage.addKeyboardMouseBinding("Camera: p","Print position of the camera");
    usage.addKeyboardMouseBinding("Camera: F1-F12","switch the Agent to be watched");
    usage.addKeyboardMouseBinding("Camera: end","move behind the Agent to be watched");
  }

  void CameraManipulator::flushMouseEventStack(){
    event_old = NULL;
    event = NULL;
  }
  void CameraManipulator::addMouseEvent(const osgGA::GUIEventAdapter& ea){
    event_old = event;
    event = &ea;
  }

  /** normally called only when this manipulator is choosed
   */
  void CameraManipulator::setByMatrix(const Matrixd& matrix){

    eye = matrix.getTrans();
    Vec3 xaxis(1,0,0);    
    Pos head = Matrix::transform3x3(xaxis, matrix);
    view.x() = RadiansToDegrees(getAngle(xaxis, head)) *       
      sign(head.y()); // this resolves the ambiguity of getAngle

    Pos tilt = Matrix::transform3x3(Vec3(0,0,1), matrix);
    //    head.print();
    //    tilt.print();    
    std::cout << "Manipulator choosed: " <<  className() << std::endl;
    view.y() = RadiansToDegrees(getAngle(Vec3(0,0,1), tilt)-M_PI/2);
    desiredEye=eye;
    desiredView=view;
    computeMatrix();
  }



  // hier reinhängen nicht, wird nur beim switch des manipulators aufgerufen
  Matrixd CameraManipulator::getMatrix() const {
    return pose;
  }

  void CameraManipulator::update() {
    // the call from simulation.cpp works, but is made for ALL cameramanipulators!
    // which is now neccessary for the smoothness for the mouse interactions
    
    // modify the desiredView and desiredEye by the movement of the agent
    calcMovementByAgent();

    // now do smoothness
    float updateFactor;
    updateFactor = globalData.odeConfig.drawInterval * globalData.odeConfig.simStepSize * globalData.odeConfig.cameraSpeed;
      //    std::cout << "drawInt: " << globalData.odeConfig.drawInterval << ", realtimefactor: "
      //      << globalData.odeConfig.realTimeFactor << ", updateFactor: " 
      //      << updateFactor << "\n";
    for (int i=0;i<=2;i++) {
      // view is in °, we must be careful for switches at the 360°-point
      if ((desiredView[i]-view[i])>180) // desiredView is to high
	view[i]+=360;
      else if ((view[i]-desiredView[i])>180) // view is to high
	desiredView[i]+=360;
      if (abs(desiredView[i]-view[i])>degreeAccuracy)
	view[i]= normalize360(degreeSmoothness * updateFactor * desiredView[i] + 
			      (1.0 - degreeSmoothness * updateFactor) * view[i]);
      if (abs(desiredEye[i]-eye[i])>lengthAccuracy)
	eye[i]= lengthSmoothness * updateFactor * desiredEye[i]
	  + (1.0 - lengthSmoothness * updateFactor) * eye[i];
    }

    // now set the current robots-position
    if (watchingAgentDefined) {
      oldPositionOfAgent = watchingAgent->getRobot()->getPosition();
      oldPositionOfAgentDefined=true;
    }
    computeMatrix();
  }


  
  // hier reinhaengen?? is called every drawstep!!!! really? ;)
  // should we call a CameraManipulator-routine from simulation.cpp?
  Matrixd CameraManipulator::getInverseMatrix() const {
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
    //    std::cout << "i calc mouse movement!" << std::endl;
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
      desiredView.x() += dx*3.0f;
      desiredView.y() -= dy*3.0f;
    } else if (buttonMask==GUIEventAdapter::MIDDLE_MOUSE_BUTTON ||
	       buttonMask==(GUIEventAdapter::LEFT_MOUSE_BUTTON | GUIEventAdapter::RIGHT_MOUSE_BUTTON)) { 
      desiredEye.z() += -dy;
      desiredEye.x() += - c*dx;
      desiredEye.y() += - s*dx;
    } else if (buttonMask==GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
      desiredEye.x() +=  s*dy - c*dx;
      desiredEye.y() +=  -c*dy - s*dx;      
    } else return false;
    return true;
  }



  void CameraManipulator::manageAgents(const int& fkey) {
    //std::cout << "new robot to choose: " << fkey << "\n";
    watchingAgentDefined=false;
    oldPositionOfAgentDefined=false;
    int i=1;
    // go through the agent list
    for(OdeAgentList::iterator it=globalData.agents.begin(); it != globalData.agents.end(); it++){
      if (fkey==i++) {
	watchingAgent=(*it);
	watchingAgentDefined=true;
	break;
      }
    }
    if (!watchingAgentDefined)
      std::cout << "no agent was choosed!\n";
    else {
      std::cout << "the agent was choosed: " << i-1 << "\n";
      setHomeViewByAgent();
      setHomeEyeByAgent();
    }
  }

  void CameraManipulator::moveBehindAgent() {
    // taken from the race camera
    if (watchingAgent!=NULL) {
      // manipulate desired eye by the move of the robot
      const double* robMove = (watchingAgent->getRobot()->getPosition()-oldPositionOfAgent).toArray();
      // attach the robSpeed to desired eye
      for (int i=0;i<=2;i++) {
	if (!isNaN(robMove[i])) {
	  desiredEye[i]+=robMove[i];}
	else 
	  std::cout << "NAN exception!" << std::endl;
      }
      // move behind the robot
      // returns the orientation of the robot in matrix style
      matrix::Matrix Orientation= (watchingAgent->getRobot()->getOrientation());
      Orientation.toTranspose();
      // first get the normalized vector of the orientation
      double eVecX[3] = {0,1,0};
      double eVecY[3] = {1,0,0};
      matrix::Matrix normVecX = Orientation * matrix::Matrix(3,1,eVecX);
      matrix::Matrix normVecY = Orientation * matrix::Matrix(3,1,eVecY);
      // then get the distance between robot and camera
      Position robPos = watchingAgent->getRobot()->getPosition();
      double distance = sqrt(square(desiredEye[0]-robPos.x)+
			     square(desiredEye[1]-robPos.y));
      // then new eye = robPos minus normalized vector * distance
      desiredEye[0]=robPos.x + distance *normVecX.val(1,0);
      desiredEye[1]=robPos.y - distance *normVecY.val(1,0);

      // now do center on the robot (manipulate the view)
      // desiredEye is the position of the camera
      // calculate the horizontal angle, means pan (view.x)
      if (robPos.x-desiredEye[0]!=0) { // division by zero
	desiredView[0]= atan((desiredEye[0]-robPos.x)/(robPos.y-desiredEye[1]))
	  / PI*180.0f+180.0f;
       	if (desiredEye[1]-robPos.y<0) // we must switch
		  desiredView[0]+=180.0f;
      }
      // calculate the vertical angle
      if (robPos.z-desiredEye[2]!=0) { // division by zero
	// need dz and sqrt(dx^2+dy^2) for calulation
	desiredView[1]=-atan((sqrt(square(desiredEye[0]-robPos.x)+
				  square(desiredEye[1]-robPos.y)))
			    /(robPos.z-desiredEye[2]))
	  / PI*180.0f-90.0f;
	if (desiredEye[2]-robPos.z<0) // we must switch
	  desiredView[1]+=180.0f;
      }
    }
  }

  void CameraManipulator::centerOnAgent() {
    // taken from the follow camera
    // ok here the camera will center on the robot
    if (watchingAgent!=NULL) {
      // the actual position of the agent has to be recognized
      // we use the Position getPosition() from OdeRobot
      Position robPos = watchingAgent->getRobot()->getPosition();
      // desiredEye is the position of the camera
      // calculate the horizontal angle, means pan (view.x)
      if (robPos.x-desiredEye[0]!=0) { // division by zero
	desiredView[0]= atan((desiredEye[0]-robPos.x)/(robPos.y-desiredEye[1]))
	  / PI*180.0f+180.0f;
       	if (desiredEye[1]-robPos.y<0) // we must switch
		  desiredView[0]+=180.0f;
      }
      // calculate the vertical angle
      if (robPos.z-desiredEye[2]!=0) { // division by zero
	// need dz and sqrt(dx^2+dy^2) for calulation
	desiredView[1]=-atan((sqrt(square(desiredEye[0]-robPos.x)+
				  square(desiredEye[1]-robPos.y)))
			    /(robPos.z-desiredEye[2]))
	  / PI*180.0f-90.0f;
	if (desiredEye[2]-robPos.z<0) // we must switch
	  desiredView[1]+=180.0f;
      }
    }
  }
  
  void CameraManipulator::calcMovementByAgent() {
    if (watchingAgentDefined && oldPositionOfAgentDefined) {
      // then manipulate desired view and desired eye
      // the default camera manipulator does not need to change the eye and view
    }
  }

  void CameraManipulator::setHomeViewByAgent() {
    // the default camera manipulator does not need to change the view
    // normally the desired view should be changed
  }

  void CameraManipulator::setHomeEyeByAgent() {
    // the default camera manipulator does not need to change the eye
    // normally the desired eye should be changed
  }

}

