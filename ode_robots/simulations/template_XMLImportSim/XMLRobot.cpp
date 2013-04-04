/*
 * XMLRobot.cpp
 *
 *  Created on: 03.02.2010
 *      Author: robot3
 */

#include "XMLRobot.h"
#include "XMLHelper.h"
#include <xercesc/dom/DOMNode.hpp>

using namespace std;
using namespace XERCESC;
using namespace lpzrobots;

XMLRobot::XMLRobot(DOMNode* robotNode, XMLParserEngine& xmlEngine, const string& name)
: OdeRobot(xmlEngine.getOdeHandle(), xmlEngine.getOsgHandle(), name, "$Id$"), XMLObject(xmlEngine) {
        // TODO Auto-generated constructor stub

}

XMLRobot::~XMLRobot() {
        // TODO Auto-generated destructor stub
}


/// update the OSG notes here
 void XMLRobot::update() {

 }

 /** sets the pose of the vehicle
        @param pose desired 4x4 pose matrix
 */
  void XMLRobot::place(const osg::Matrix& pose) {

  }

 /** this function is called in each timestep after control. It
        should perform robot-internal checks and actions,
        like acting and sensing of internal motors/sensors etc.
        @param globalData structure that contains global data from the simulation environment
 */
  void XMLRobot::doInternalStuff(GlobalData& globalData) {

  }

 /// return the primitive of the robot that is used for tracking and camera following
  Primitive* XMLRobot::getMainPrimitive() const {

  }
