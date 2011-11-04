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

#include <osg/Notify>
#include "cameramanipulator.h"
#include "osgprimitive.h"
#include "odeagent.h"
#include "mathutils.h"
#include "pos.h"
#include "selforg/stl_adds.h"

#include <vector>
#include <iterator>

namespace lpzrobots {

  using namespace osg;
  using namespace osgGA;
  using namespace std;




  CameraManipulator::CameraManipulator(osg::Node* node, GlobalData& global, CameraHandle& cameraHandle)
    : node(node), camHandle(cameraHandle), globalData(global) {
    if (this->node.get()) {
      const BoundingSphere& boundingSphere=this->node->getBound();
      modelScale = boundingSphere._radius;
    }else
      modelScale = 0.01f;
    cameraHandle.desiredEye=camHandle.eye;
    cameraHandle.desiredView=camHandle.view;
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
    camHandle.home_eye = _eye;
    camHandle.home_view = _view;
    camHandle.home_externally_set=true;
    camHandle.eye  = camHandle.home_eye;
    camHandle.view = camHandle.home_view;
    camHandle.desiredEye=_eye;
    camHandle.desiredView=_view;
    computeMatrix(); // i think we don't need this???
  }


  void CameraManipulator::home(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us){
    if(node.get() && !camHandle.home_externally_set) {
      const BoundingSphere& boundingSphere=node->getBound();

      camHandle.home_eye = boundingSphere._center+
	Vec3(-boundingSphere._radius*1.2f,0, boundingSphere._radius*0.2f);

      camHandle.home_view = Vec3(-90,-10,0);
    }
    camHandle.desiredEye=camHandle.home_eye;
    camHandle.desiredView=camHandle.home_view;
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
    // if control is pressed then manipulation of robot
    if(ea.getModKeyMask() & GUIEventAdapter::MODKEY_LEFT_CTRL){
      switch(ea.getEventType())
	{
	case(GUIEventAdapter::PUSH):
	  {
	    float x = ea.getXnormalized();
	    float y = ea.getYnormalized();
	    calcManipulationPoint(x,y);
	    if(ea.getButtonMask() & GUIEventAdapter::RIGHT_MOUSE_BUTTON){
	      camHandle.doManipulation = camHandle.Rotational;
	    }else{
	      camHandle.doManipulation = camHandle.Translational;
	    }
	    flushMouseEventStack();
	    return true;
	  }
	case(GUIEventAdapter::RELEASE):
	  {
	    flushMouseEventStack();
	    camHandle.doManipulation = camHandle.No;
	    return true;
	  }
	case(GUIEventAdapter::DRAG):
	  {
	    addMouseEvent(ea);
	    us.requestContinuousUpdate(true);
	    float x = ea.getXnormalized();
	    float y = ea.getYnormalized();
	    calcManipulationPoint(x,y);
	    return true;
	  }
	default:
	  return false;
	}
    } else {
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
	    camHandle.doManipulation = camHandle.No;
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
	      printf("Camera Position/View: (Pos(%g, %g, %g), ", camHandle.eye.x(), camHandle.eye.y(), camHandle.eye.z());
	      printf(" Pos(%g, %g, %g));\n", camHandle.view.x(), camHandle.view.y(), camHandle.view.z());
        printf("XML-notation:\n");
        printf("<Position X=\"%g\" Y=\"%g\" Z=\"%g\"/>\n", camHandle.eye.x(), camHandle.eye.y(), camHandle.eye.z());
        printf("<Rotation Alpha=\"%g\" Beta=\"%g\" Gamma=\"%g\"/>\n", camHandle.view.x(), camHandle.view.y(), camHandle.view.z());
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
    }
    return true;
  }

  void CameraManipulator::getUsage(osg::ApplicationUsage& usage) const{
    usage.addKeyboardMouseBinding("Cam: Space","Reset the viewing position to home");
    usage.addKeyboardMouseBinding("Cam: p","Print position of the camera");
    usage.addKeyboardMouseBinding("Cam: F1-F12","switch the Agent to be watched");
    usage.addKeyboardMouseBinding("Cam: End","move behind the Agent to be watched");
    usage.addKeyboardMouseBinding("Cam: Left Mousebtn","Turn Camera (not in TV mode)");
    usage.addKeyboardMouseBinding("Cam: Middle M.btn","Move Camera up and down");
    usage.addKeyboardMouseBinding("Cam: Right M.btn","Move Camera along the plane");
    usage.addKeyboardMouseBinding("Cam: Ctrl+Mouse","Drag watched Agent (Left:translation, Right:rotation)");
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
  void CameraManipulator::setByMatrix(const osg::Matrixd& matrix){

    camHandle.eye = matrix.getTrans();
    Vec3 xaxis(1,0,0);
    Pos head = Matrix::transform3x3(xaxis, matrix);
    camHandle.view.x() = RadiansToDegrees(getAngle(xaxis, head)) *
      sign(head.y()); // this resolves the ambiguity of getAngle

    Pos tilt = Matrix::transform3x3(Vec3(0,0,1), matrix);
    //    head.print();
    //    tilt.print();
    std::cout <<  className() << " selected" << std::endl;
    camHandle.view.y() = RadiansToDegrees(getAngle(Vec3(0,0,1), tilt)-M_PI/2);
    camHandle.desiredEye=camHandle.eye;
    camHandle.desiredView=camHandle.view;
    computeMatrix();
  }



  // hier nicht reinhaengen, wird nur beim switch des manipulators aufgerufen
  Matrixd CameraManipulator::getMatrix() const {
    return pose;
  }

  void CameraManipulator::update() {
    // first look if some robot is selected, if not, select the first one
    if (!this->isWatchingAgentDefined())
    {
      manageAgents(1);
    }
    // the call from simulation.cpp works, but is made for ALL cameramanipulators!
    // which is now neccessary for the smoothness for the mouse interactions

    // modify the desiredView and desiredEye by the movement of the agent
    calcMovementByAgent();

    // now do smoothness
    float updateFactor;
    updateFactor = globalData.odeConfig.drawInterval * globalData.odeConfig.simStepSize *
      globalData.odeConfig.cameraSpeed;
    if(lengthSmoothness * updateFactor > 1) updateFactor = 1/lengthSmoothness;
    if(degreeSmoothness * updateFactor > 1) updateFactor = 1/degreeSmoothness;
    // /std::max(0.1,globalData.odeConfig.realTimeFactor)
    //    std::cout << "drawInt: " << globalData.odeConfig.drawInterval << ", realtimefactor: "
    //      << globalData.odeConfig.realTimeFactor << ", updateFactor: "
    //      << updateFactor << "\n";
    for (int i=0;i<=2;i++) {
      // view is in , we must be careful for switches at the 360-point
      if ((camHandle.desiredView[i]-camHandle.view[i])>180) // desiredView is to high
        camHandle.view[i]+=360;
      else if ((camHandle.view[i]-camHandle.desiredView[i])>180) // view is to high
        camHandle.desiredView[i]+=360;
	    if (std::abs(camHandle.desiredView[i]-camHandle.view[i])>degreeAccuracy)
	      camHandle.view[i]= normalize360(degreeSmoothness * updateFactor * camHandle.desiredView[i] +
			      (1.0 - degreeSmoothness * updateFactor) * camHandle.view[i]);
	    if (std::abs(camHandle.desiredEye[i]-camHandle.eye[i])>lengthAccuracy)
	      camHandle.eye[i]= lengthSmoothness * updateFactor * camHandle.desiredEye[i]
	  + (1.0 - lengthSmoothness * updateFactor) * camHandle.eye[i];

      // if out of bounds then just go the to disired position
      if(fabs(camHandle.eye[i])+fabs(camHandle.view[i])>20000) {
        camHandle.eye[i]=camHandle.desiredEye[i];
        camHandle.view[i]=camHandle.desiredView[i];
      }
    }

    // now set the current robots-position
    if (this->isWatchingAgentDefined())
    {
      camHandle.oldPositionOfAgent = camHandle.watchingAgent->getRobot()->getPosition();
      camHandle.oldPositionOfAgentDefined=true;
    }
    computeMatrix();
  }



  Matrixd CameraManipulator::getInverseMatrix() const {
    return Matrixd::inverse(pose);
  }

  void CameraManipulator::computeMatrix(){
    for (int i=0; i<3; i++) {
      while (camHandle.view[i] > 180) camHandle.view[i] -= 360;
      while (camHandle.view[i] < -180) camHandle.view[i] += 360;
    }
    osg::Matrix rot;
    rot.makeRotate( M_PI/2,                          osg::Vec3(1, 0, 0),
		    osg::DegreesToRadians(camHandle.view.x()), osg::Vec3(0, 0, 1), // heading
		    osg::DegreesToRadians(camHandle.view.y()), osg::Vec3(cos(osg::DegreesToRadians(camHandle.view.x())),
							       sin(osg::DegreesToRadians(camHandle.view.x())),
							       0) // pitch
		    );

    pose = rot * Matrix::translate(camHandle.eye);
  }

  bool CameraManipulator::calcMovement(){
    //    std::cout << "i calc mouse movement!" << std::endl;
  // _camera->setFusionDistanceMode(Camera::PROPORTIONAL_TO_SCREEN_DISTANCE);

    // return if less then two events have been added.
    if (event.get()==NULL || event_old.get()==NULL) return false;

    //    double dt = event->time()-event_old->time();
    double dx = 10.0*(event->getXnormalized() - event_old->getXnormalized());
    double dy = 10.0*(event->getYnormalized() - event_old->getYnormalized());
    double s =  sin(osg::DegreesToRadians(camHandle.view.x()));
    double c =  cos(osg::DegreesToRadians(camHandle.view.x()));

    unsigned int buttonMask = event_old->getButtonMask();
    if (buttonMask==GUIEventAdapter::LEFT_MOUSE_BUTTON) {
      camHandle.desiredView.x() += dx*3.0f;
      camHandle.desiredView.y() -= dy*3.0f;
    } else if (buttonMask==GUIEventAdapter::MIDDLE_MOUSE_BUTTON ||
	       buttonMask==(GUIEventAdapter::LEFT_MOUSE_BUTTON | GUIEventAdapter::RIGHT_MOUSE_BUTTON)) {
      camHandle.desiredEye.z() += -dy;
      camHandle.desiredEye.x() += - c*dx;
      camHandle.desiredEye.y() += - s*dx;
    } else if (buttonMask==GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
      camHandle.desiredEye.x() +=  s*dy - c*dx;
      camHandle.desiredEye.y() +=  -c*dy - s*dx;
    } else return false;
    return true;
  }



  void CameraManipulator::manageAgents(const int& fkey) {
    assert(fkey>0);
    camHandle.watchingAgentDefined=false;
    if(globalData.agents.size() >= (unsigned) fkey){
      setWatchedAgent(globalData.agents[fkey-1]);
    }
  }

  void CameraManipulator::moveBehindAgent() {
    // taken from the race camera
    if (!this->isWatchingAgentDefined()) return;
    // manipulate desired eye by the move of the robot
    const double* robMove = (camHandle.watchingAgent->getRobot()->getPosition()-camHandle.oldPositionOfAgent).toArray();
    // attach the robSpeed to desired eye
    for (int i=0;i<=2;i++) {
      if (!isNaN(robMove[i])) {
        camHandle.desiredEye[i]+=robMove[i];}
      else
        std::cout << "NAN exception!" << std::endl;
    }
    // move behind the robot
    // returns the orientation of the robot in matrix style
    matrix::Matrix Orientation= (camHandle.watchingAgent->getRobot()->getOrientation());
    Orientation.toTranspose();
    // first get the normalized vector of the orientation
    double eVecX[3] = {0,1,0};
    double eVecY[3] = {1,0,0};
    matrix::Matrix normVecX = Orientation * matrix::Matrix(3,1,eVecX);
    matrix::Matrix normVecY = Orientation * matrix::Matrix(3,1,eVecY);
    // then get the distance between robot and camera
    Position robPos = camHandle.watchingAgent->getRobot()->getPosition();
    double distance = sqrt(square(camHandle.desiredEye[0]-robPos.x)+
                           square(camHandle.desiredEye[1]-robPos.y));
    // then new eye = robPos minus normalized vector * distance
    camHandle.desiredEye[0]=robPos.x + distance *normVecX.val(1,0);
    camHandle.desiredEye[1]=robPos.y - distance *normVecY.val(1,0);

    // now do center on the robot (manipulate the view)
    // desiredEye is the position of the camera
    // calculate the horizontal angle, means pan (view.x)
    if (robPos.x-camHandle.desiredEye[0]!=0) { // division by zero
      camHandle.desiredView[0]= atan((camHandle.desiredEye[0]-robPos.x)/(robPos.y-camHandle.desiredEye[1]))
        / PI*180.0f+180.0f;
      if (camHandle.desiredEye[1]-robPos.y<0) // we must switch
        camHandle.desiredView[0]+=180.0f;
    }
    // calculate the vertical angle
    if (robPos.z-camHandle.desiredEye[2]!=0) { // division by zero
      // need dz and sqrt(dx^2+dy^2) for calulation
      camHandle.desiredView[1]=-atan((sqrt(square(camHandle.desiredEye[0]-robPos.x)+
                                square(camHandle.desiredEye[1]-robPos.y)))
                          /(robPos.z-camHandle.desiredEye[2]))
        / PI*180.0f-90.0f;
      if (camHandle.desiredEye[2]-robPos.z<0) // we must switch
        camHandle.desiredView[1]+=180.0f;
    }
  }

  void CameraManipulator::centerOnAgent() {
    // taken from the follow camera
    // ok here the camera will center on the robot
    if (!this->isWatchingAgentDefined()) return;
    // the actual position of the agent has to be recognized
    // we use the Position getPosition() from OdeRobot
    Position robPos = camHandle.watchingAgent->getRobot()->getPosition();
    // desiredEye is the position of the camera
    // calculate the horizontal angle, means pan (view.x)
    if (robPos.x-camHandle.desiredEye[0]!=0) { // division by zero
      camHandle.desiredView[0]= atan((camHandle.desiredEye[0]-robPos.x)/(robPos.y-camHandle.desiredEye[1]))
        / PI*180.0f+180.0f;
      if (camHandle.desiredEye[1]-robPos.y<0) // we must switch
        camHandle.desiredView[0]+=180.0f;
    }
    // calculate the vertical angle
    if (robPos.z-camHandle.desiredEye[2]!=0) { // division by zero
      // need dz and sqrt(dx^2+dy^2) for calulation
      camHandle.desiredView[1]=-atan((sqrt(square(camHandle.desiredEye[0]-robPos.x)+
                                square(camHandle.desiredEye[1]-robPos.y)))
                          /(robPos.z-camHandle.desiredEye[2]))
        / PI*180.0f-90.0f;
      if (camHandle.desiredEye[2]-robPos.z<0) // we must switch
        camHandle.desiredView[1]+=180.0f;
    }
  }

  void CameraManipulator::calcMovementByAgent() {
    if ( this->isWatchingAgentDefined() && camHandle.oldPositionOfAgentDefined)
    {
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

  void CameraManipulator::calcManipulationPoint(float x, float y){
    if (!this->isWatchingAgentDefined()) return;

    osg::Vec3 near_point = osg::Vec3(x, y, -1.0f) * pose;
    osg::Vec3 far_point = osg::Vec3(x, y, 1.0f) * pose;

    Pos n = (near_point-far_point);
    // we put here a scale of 5 to make the distance larger
    Pos mousepos = ( Pos(x, y, .0f)*3 )  * pose;
    Pos p = camHandle.watchingAgent->getRobot()->getPosition();
    // we have a plane through p normal to camera view
    // we have the vector normal to the plane and intersect now plane and ray
    // the ray has parametric form (mousepos + k*n)
    double k = (n*p - n*mousepos)/(n*n);
    Pos lookat = mousepos + n*k;
    camHandle.manipulationPoint = lookat;
  }

  bool CameraManipulator::isWatchingAgentDefined()
  {
    if(!camHandle.watchingAgent || !camHandle.watchingAgentDefined)
      return false;
    OdeAgentList::iterator itr = find (globalData.agents.begin(),globalData.agents.end(),camHandle.watchingAgent);
    // if watchingAgent is defined but not in list, select the first one in the list
    if (itr==globalData.agents.end() && !globalData.agents.empty()) {
      camHandle.watchingAgent = (*globalData.agents.begin());
    }
    if (!camHandle.watchingAgent)
      return false;
    // additional check if robot is available
    if (!camHandle.watchingAgent->getRobot())
      return false;
    return true;
  }


  void CameraManipulator::manipulateAgent( OsgHandle& osgHandle){
    if (!this->isWatchingAgentDefined()) return;
    if(camHandle.manipulationViz)
      delete camHandle.manipulationViz;
    camHandle.manipulationViz=0;
    if(camHandle.doManipulation != camHandle.No){
      Primitive* body = camHandle.watchingAgent->getRobot()->getMainPrimitive();
      if(body && body->getBody()){
	Pos p = camHandle.watchingAgent->getRobot()->getPosition();
	camHandle.manipulationViz = new OSGSphere(0.1);
	//camHandle.manipulationViz->init(osgHandle);
        camHandle.manipulationViz->init(osgHandle.changeColor(camHandle.doManipulation==camHandle.Translational, camHandle.doManipulation==camHandle.Rotational,0));
	camHandle.manipulationViz->setMatrix(osg::Matrix::translate(camHandle.manipulationPoint));
	Pos force = (camHandle.manipulationPoint-p);
	double factor = force.length()>10 ? 10/force.length() : 1;
	if(camHandle.doManipulation==camHandle.Translational){
	  force *= factor/globalData.odeConfig.simStepSize;
	  dBodyAddForce(body->getBody(),force.x(),force.y(),force.z());
	} else {
	  force *= 0.3*factor/globalData.odeConfig.simStepSize;
	  dBodyAddTorque(body->getBody(),force.x(),force.y(),force.z());
	}
	// Damp both, rotation and speed
	const double* vel = dBodyGetAngularVel( body->getBody());
	dBodyAddTorque ( body->getBody() , -0.2*vel[0] , -0.2*vel[1] , -0.2*vel[2]);
	vel = dBodyGetLinearVel( body->getBody());
	dBodyAddForce ( body->getBody() , -0.2*vel[0] , -0.2*vel[1] , -0.2*vel[2]);
      }
    }
  }

  void CameraManipulator::setWatchedAgent(OdeAgent* agent) {
    if (agent) {
      camHandle.watchingAgent = agent;
      camHandle.watchingAgentDefined = true;
      cout << "Agent " << camHandle.watchingAgent->getRobot()->getName() << " selected" << endl;
      setHomeViewByAgent();
      setHomeEyeByAgent();
      // maybe highlight agent here
      camHandle.oldPositionOfAgentDefined=false;
    }
  }

  OdeAgent* CameraManipulator::getWatchedAgent() {   
    if (!this->isWatchingAgentDefined()) return 0;
    return camHandle.watchingAgent;
  }

}

