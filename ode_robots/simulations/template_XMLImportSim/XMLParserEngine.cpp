/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *  DESCRIPTION                                                            *
 *  
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.2  2010-03-08 07:19:14  guettler
 *  StandardCamera renamed to StandardMode
 *
 *  Revision 1.1  2010/03/07 22:50:38  guettler
 *  first development state for feature XMLImport
 *										   *
 *                                                                         *
 **************************************************************************/
#include "XMLParserEngine.h"

#include <iostream>

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>

#include "XMLHelper.h"
#include "XMLErrorHelper.h"
#include "XMLPassiveObject.h"
#include "XMLDefinitions.h"
#include "XMLSimulation.h"

#include <ode_robots/odeagent.h>
#include "XMLRobot.h"
#include <ode_robots/truckmesh.h>
#include <selforg/invertmotorspace.h>
#include <selforg/one2onewiring.h>
#include <ode/ode.h>
#include <ode_robots/simulation.h>
#include <osg/Light>
#include <osg/LightSource>


using namespace std;
using namespace xercesc;
using namespace xercesc_3_1;
using namespace lpzrobots;
using namespace osg;

XMLParserEngine::XMLParserEngine(GlobalData& globalData, const OdeHandle& odeHandle,
    const OsgHandle& osgHandle, XMLSimulation* simulation) :
    globalData(globalData), odeHandle(odeHandle), osgHandle(osgHandle), simulation(simulation)
    {

	 primitiveFactory = new XMLPrimitiveFactory(this,globalData,odeHandle,osgHandle);
	  // initialisation of XMLPlatformUtils (Xerces)
	  try
	  {
		XMLPlatformUtils::Initialize();
	  }
	  catch (XMLPlatformUtilsException toCatch)
	  {
		cerr << "Error during Xerces-c Initialization." << endl << "  Exception message:" << C(toCatch.getMessage());
		cerr << "Exiting." << endl;
		exit(-1);
	  }
	  // get the DOMParser
	  parser = new XercesDOMParser;
	  parser->setValidationScheme(XercesDOMParser::Val_Auto);
	  parser->setDoNamespaces(false);
	  parser->setDoSchema(false);
	  parser->setValidationSchemaFullChecking(false);
	  parser->setCreateEntityReferenceNodes(false);

	  // set our own error reporter
	  // so later you can do filtering, forward and so on
	  XMLErrorHelper* errReporter = new XMLErrorHelper();
	  parser->setErrorHandler(errReporter);
	}

XMLParserEngine::~XMLParserEngine() {
  XMLPlatformUtils::Terminate();
}

bool XMLParserEngine::loadXMLFile(string XMLFile)
{
  bool returnWithErrors = false;

  try
  {
    parser->parse(X(XMLFile));
    cout << "######Begin of XML parsing######" << endl;
    //global->obstacles->push_back(mymesh);

    // get the DOM representation
    DOMDocument* doc = parser->getDocument();

    //cout << "number nodes: " << doc->getChildNodes()->getLength() << endl;
    //cout << "first node: " << C(doc->getChildNodes()->item(0)->getNodeName()) << endl;
    //cout << "second node: " << C(doc->getChildNodes()->item(1)->getNodeName()) << endl;

    DOMNodeList* list = doc->getElementsByTagName(X("Scene"));
    //cout << "Scene child length: " << list->item(0)->getChildNodes()->getLength() <<endl;
    //int number=1;
    /*for(int number=0;number<= list->item(0)->getChildNodes()->getLength();number++)
    {
    	cout << "Scene child name "<< number << ": " << list->item(0)->getChildNodes()->item(number)->getNodeName() <<endl;
    }*/
    if (list->getLength()>0) {
      DOMNode* sceneNode = list->item(0);
      // a scene can contain: objects, camera, light
      for EACHCHILDNODE(sceneNode, nodeOfScene) {
          if (XMLHelper::matchesName(nodeOfScene,XMLDefinitions::GlobalVariablesNode))
          {
			  DOMNode* GlobalVariablesNode=nodeOfScene;
			  cout << "Global Variables found " << "noise " << XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::noiseAtt)<<endl;
			  cout << "Global Variables found " << "gravity " << XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::gravityAtt)<<endl;
			  cout << "Global Variables found " << "fps " << XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::fpsAtt)<<endl;
			  //Shadowtype && Drawboundings
			  simulation->osgHandle.drawBoundings = XMLHelper::getNodeAtt(GlobalVariablesNode, XMLDefinitions::DrawBoundingsAtt);
			  simulation->osgHandle.shadowType = XMLHelper::getNodeAtt(GlobalVariablesNode, XMLDefinitions::shadowTypeAtt);
			  globalData.odeConfig.noise=XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::noiseAtt);
			  globalData.odeConfig.gravity=XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::gravityAtt);
			  globalData.odeConfig.realTimeFactor=XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::realTimeFactorAtt);
			  globalData.odeConfig.controlInterval=XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::controlintervalAtt);
			  globalData.odeConfig.simStepSize=XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::simStepSizeAtt);
			  globalData.odeConfig.randomSeed=XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::randomSeedAtt);
			  globalData.odeConfig.fps=XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::fpsAtt);
			  globalData.odeConfig.motionPersistence=XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::motionPersistenceAtt);
			  globalData.odeConfig.cameraSpeed=XMLHelper::getNodeAtt(GlobalVariablesNode,XMLDefinitions::cameraSpeedAtt);
          }
    	  if (XMLHelper::matchesName(nodeOfScene,"PassiveObjects")) {
			  for EACHCHILDNODE(nodeOfScene, nodeOfPassiveObject) {
					  new XMLPassiveObject(nodeOfPassiveObject,*this);
					  //XMLPassiveObject* passiveObject = new XMLPassiveObject(nodeOfPassiveObject,*this);
					  					   }
    	  }
          if (XMLHelper::matchesName(nodeOfScene,"Agents"))
          {
        	  for EACHCHILDNODE(nodeOfScene, AgentNode)
			  {
        		  if (XMLHelper::matchesName(AgentNode,"Agent")){
					  cout << "found "<<C(AgentNode->getNodeName())<< " name " << XMLHelper::getChildNodeValueAsString(nodeOfScene,"Agent","id") <<  endl;
					  const DOMNode* RobotNode = XMLHelper::getChildNode(AgentNode,"Robot");
					  const DOMNode* ControllerNode = XMLHelper::getChildNode(AgentNode,"Controller");

					  cout << "found "<<C(RobotNode->getNodeName())<< "Position " <<  endl;

					  OsgHandle osgHandle_orange = osgHandle.changeColor(Color(2, 156/255.0, 0));
//todo Truckmesh durch Abstrakten Roboter ersetzen
					  OdeRobot* vehicle = new TruckMesh(odeHandle, osgHandle_orange, // ODE- and OSGHandle
										  XMLHelper::getChildNodeValueAsString(nodeOfScene,"Agent","id"), // the final name of the Meshrobot in the simulation
										  1.2, // scale factor (size) of the robot
										  2, // the force of the motors (scales automatically with size)
										  5, // the max speed of the vehicle
										  1); // the mass of the vehicle (scales automatically with size)
						   vehicle->place(XMLHelper::getPosition(RobotNode));

						  // create pointer to controller
						  // push controller in global list of configurables
						  //  AbstractController *controller = new InvertNChannelController(10);
						  if(XMLHelper::getChildNodeValueAsString(AgentNode,"Controller","name")=="InvertMotorSpace"){
							  AbstractController *controller = new InvertMotorSpace(15);
							  cout << "  s4avg found " << XMLHelper::getChildNodeValue(ControllerNode,"Param","s4avg") << endl;
							  controller->setParam("s4avg",XMLHelper::getChildNodeValue(ControllerNode,"Param","s4avg"));
//todo weiter Controller Param setzen
							  globalData.configs.push_back(controller);

							  if(XMLHelper::getChildNodeValueAsString(AgentNode,"Wiring","name")=="One2OneWiring"){
								  // create pointer to one2onewiring
								  cout << "  ColorUniformNoise found " << XMLHelper::getChildNodeValue(AgentNode,"Wiring","ColorUniformNoise") << endl;
								  One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(XMLHelper::getChildNodeValue(AgentNode,"Wiring","ColorUniformNoise")));

								  // create pointer to agent
								  // initialize pointer with controller, robot and wiring
								  // push agent in globel list of agents

								  OdeAgent* agent = new OdeAgent(simulation->plotoptions);
								  agent->init(controller, vehicle, wiring);
								  globalData.agents.push_back(agent);
								  showParams(globalData.configs);

							  }else{cout<<"no WiringNode found"<<endl;}
						  }else{cout<<"no ControllerNode found"<<endl;}
        		  //new XMLRobot(AgentNode,*this,XMLHelper::getNodeAtt(AgentNode,"id"));
        		  }
			  }
          }
          if (XMLHelper::matchesName(nodeOfScene,XMLDefinitions::cameraNode))
          {
        	  for EACHCHILDNODE(nodeOfScene, CameraNode)
			  {
        		  cout << "Camera found " << endl;
        		  if (XMLHelper::matchesName(CameraNode,XMLDefinitions::cameraStandardModeNode))
        		  {
					  cout << "  StandardCamera found " << endl;
					  //war protected geändert auf public in simulation.h
					  //cout << "    Pos-X " <<  << endl;
					  //cout << "    Pos-Y " <<  << endl;
					  //cout << "    Pos-Z " <<  << endl;
					  //cout << "    View-X " <<  << endl;
					  //cout << "    View-Y " <<  << endl;
					  //cout << "    View-Z " <<  << endl;

					  simulation->setCameraHomePos(XMLHelper::getPosition(CameraNode),  XMLHelper::getRotation(CameraNode));
					  //simulation->setCameraHomePos(Pos(5.77213, -1.65879, 2.31173),  Pos(67.1911, -18.087, 0));
					  }
			  }
          }
          if (XMLHelper::matchesName(nodeOfScene,"Lights"))
          {
        	  for EACHCHILDNODE(nodeOfScene, StandardLightNode)
			  {
        		  cout << "Light found " << endl;

        		  Light* light_0 = new Light;
        		  light_0->setPosition(Vec4(40.0, 40.0, 50.0, 1.0));
        		  LightSource* light_source_0 = new LightSource;
        		  light_source_0->setLight(light_0);
        		  light_source_0->setLocalStateSetModes(StateAttribute::ON);

/*        		  //light_0->set
        		  virtual osg::LightSource* makeLights(osg::StateSet* stateset)
        		  {
        		    // create a spot light.
        		    Light* light_0 = new Light;
        		    light_0->setLightNum(0);
        		    light_0->setPosition(Vec4(40.0f, 40.0f, 50.0f, 1.0f));
        		    // note that the blue component doesn't work!!! (bug in OSG???)
        		    //   Really? It works with me! (Georg)
        		    light_0->setAmbient(Vec4(0.5f, 0.5f, 0.5f, 1.0f));
        		    light_0->setDiffuse(Vec4(0.8f, 0.8f, 0.8f, 1.0f));
        		    //    light_0->setDirection(Vec3(-1.0f, -1.0f, -1.2f));
        		    light_0->setSpecular(Vec4(1.0f, 0.9f, 0.8f, 1.0f));

        		    LightSource* light_source_0 = new LightSource;
        		    light_source_0->setLight(light_0);
        		    light_source_0->setLocalStateSetModes(StateAttribute::ON);
        		    light_source_0->setStateSetModes(*stateset, StateAttribute::ON);

        		    return light_source_0;
        		  }*/
			  }
          }
      }
    } else {
      XMLErrorHelper::printError("No node scene found!");
      returnWithErrors =true;
    }


  }
  catch (const XMLException& e)
  {
    cerr << "An error occurred during parsing" << endl;
    cerr << "Message: " << C(e.getMessage()) << endl;
    returnWithErrors = true;
  }
  catch (const OutOfMemoryException&)
  {
    cerr << "OutOfMemoryException while reading document " << XMLFile << "!" << endl;
    returnWithErrors = true;
  }
  catch (const DOMException& e)
  {
    cerr << "DOMException code is:  " << e.code << "while reading document " << XMLFile << "!" << endl;
    // it's a little bit knotty to get the message
    const unsigned int maxChars = 2047;
    XMLCh errText[maxChars + 1];
    if (DOMImplementation::loadDOMExceptionMsg(e.code, errText, maxChars))
    {
    	cerr << "Message is: " << C(errText) << endl;
    }
    returnWithErrors = true;
  }
  catch (...)
  {
    cerr << "An error occurred during parsing the document " << XMLFile << "!" << endl;
    returnWithErrors = true;
  }
  return !returnWithErrors;
}

bool XMLParserEngine::isValidateXML() {
  return validateXML;
}

void XMLParserEngine::setValidateXML(bool validate) {
  validateXML = validate;
}

