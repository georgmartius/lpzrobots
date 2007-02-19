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
 *                                                                         *
 *   Revision 1.1.4.8  2006/05/15 13:11:29  robot3                         *
 *   -handling of starting guilogger moved to simulation.cpp							 *
 *    (is in internal simulation routine now)															 *
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off 		 *
 *   -CTRL-G now restarts the GuiLogger																		 *
 *																																				 *
 ***************************************************************************/
#include <stdio.h>
#include <iostream>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environmet stuff
#include "simulation.h"

// include agent (class for holding a robot, a controller and a wiring)
#include "odeagent.h"

// used wiring
#include <selforg/one2onewiring.h>

// used robot
#include "../../robots/arm.h"

// used arena
#include "playground.h"
// used passive spheres
#include "passivesphere.h"

// used controller
#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>

using namespace lpzrobots;

Arm* arm;
InvertMotorNStep* controller;
// distal learning stuff
bool dteaching;
double* dteachingSignal;
int dteachingLen;

class ThisSim : public Simulation {
public:


  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // initial camera position and viewpoint
		setCameraHomePos(Pos(13.6696, 9.12317, 5.54366),  Pos(119.528, -8.6947, 0)); // watch robot
    //setCameraHomePos(Pos(2.69124, 4.76157, 8.87839),  Pos(134.901, -47.8333, 0)); // control initial arm config
		
		// initialization
    global.odeConfig.noise=0.1;

    Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(18, 0.2, 1.0));
		playground->setColor(Color(0.88f,0.4f,0.26f,0.2f));
		playground->setPosition(osg::Vec3(0,0,0));
		global.obstacles.push_back(playground);

//		// X-direction sphere
//		PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
//    s1->setPosition(osg::Vec3(4,0,0));
//		s1->setTexture("Images/dusty.rgb");
//		global.obstacles.push_back(s1);
//									 
//		// Y-direction sphere
//		PassiveSphere* s2 = new PassiveSphere(odeHandle, osgHandle, 1.5);
//    s2->setPosition(osg::Vec3(0,4,0));
//		s2->setTexture("Images/dusty.rgb");
//		global.obstacles.push_back(s2);
									 
    ArmConf conf = Arm::getDefaultConf();
    
    arm = new Arm(odeHandle, osgHandle, conf, "Arm");

    ((OdeRobot*)arm)->place(Position(-0.7,0.9,0.1));
    global.configs.push_back(arm);

//		AbstractController* controller; // wegen command funktion jetzt ausserhalb von start definiert
//	  controller = new InvertNChannelController(10);
//		controller = new InvertMotorNStep();
		AbstractController* controller = new SineController();
		// create pointer to one2onewiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
		
		// create pointer to agent
		// initialize pointer with controller, robot and wiring
		// push agent in globel list of agents
		OdeAgent*  agent = new OdeAgent(plotoptions);
		agent->init(controller, arm, wiring);

//		// @InvertNChannelController
//		controller->setParam("adaptrate", 0.000);
//		//    controller->setParam("nomupdate", 0.0005);
//		controller->setParam("epsC", 0.0005);
//		controller->setParam("epsA", 0.0001);
//		controller->setParam("rootE", 0);
//		controller->setParam("steps", 2);
//		controller->setParam("s4avg", 5);
//		controller->setParam("factorB",0);

		// @InvertMotorNStep
		controller->setParam("adaptrate", 0.000);
		controller->setParam("epsC", 0.05);
		controller->setParam("epsA", 0.01);
		controller->setParam("epsC", 0.05);
		controller->setParam("rootE", 0);
		controller->setParam("steps", 2);
		controller->setParam("s4avg", 5); // zeitglaettung controllerinput ueber # steps
		controller->setParam("s4del", 1); // verzoegerung bis motor echt greift
		// controlinterval - anzahl zeitschritte mit gleichem controller
		global.configs.push_back(controller);
		
		//  agent->setTrackOptions(TrackRobot(true, false, false,50));
		global.agents.push_back(agent);

		//-----------------------
		// switch off/change gravity
		global.odeConfig.setParam("gravity",-3);
		//     global.odeConfig.setParam("realtimefactor",0);

		// show (print to console) params of all objects in configurable list 
		showParams(global.configs);
		//printf("===== started. =====\n");
		
		dteaching=false;
		dteachingLen = arm->getSensorNumber();
		dteachingSignal = new double[dteachingLen];
										 
}

// add distal teaching signal (desired behavior! not error)
virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) 
{
	double sineRate=30;
  double phaseShift=0.65;
	if(dteaching)
	{
//		// alle sensoren schwingen phasenverschoben	
//		for(int i=0; i<dteachingLen; i++)
//		{
//			dteachingSignal[i]=sin(globalData.time/sineRate + i*phaseShift*M_PI/2);
//			printf("%f ",dteachingSignal[i]);
//		}
		
//		// ausgewaehlter sensor schwingt
//		dteachingSignal[3]=sin(globalData.time/3);// + phaseShift*M_PI/2);
//		printf("%f ",dteachingSignal[0]);
//		dteachingSignal[1]=0;
//		dteachingSignal[2]=0;
//		dteachingSignal[0]=0;
	
		// alle sensoren schwingen gleichphasig	
		dteachingSignal[0]=sin(globalData.time/3);// + phaseShift*M_PI/2);
		dteachingSignal[1]=sin(globalData.time/3);// + phaseShift*M_PI/2);
		dteachingSignal[2]=sin(globalData.time/3);// + phaseShift*M_PI/2);
		dteachingSignal[3]=sin(globalData.time/3);// + phaseShift*M_PI/2);
		printf("4x %f ",dteachingSignal[0]);
		
		printf("\n");
		controller->setSensorTeachingSignal(dteachingSignal, 4); // teaching signal and its length 
	}
};

//Funktion die eingegebene Befehle/kommandos verarbeitet
virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
{
	if (!down) return false;
	bool handled = false;
	switch ( key )
	{
		case 'u' :
			dteaching=!dteaching;
			printf("Distal teaching %s.\n", dteaching ? "on" : "off");
			break;
		case 'i' :
			dteachingSignal[0] = std::min(0.95, dteachingSignal[0]+0.1);
			dteachingSignal[1] = std::min(0.95, dteachingSignal[1]+0.1);
			dteachingSignal[2] = std::min(0.95, dteachingSignal[2]+0.1);
			dteachingSignal[3] = std::min(0.95, dteachingSignal[3]+0.1);
			controller->setSensorTeachingSignal(dteachingSignal, 4); // teaching signal and its length
			printf("DistalTeachingSignal: %f, %f, %f, %f\n", dteachingSignal[0], dteachingSignal[1], dteachingSignal[2], dteachingSignal[3]);
			handled = true;
			break;
		case 'k' :
			dteachingSignal[0] = std::max(-0.95, dteachingSignal[0]-0.1);
			dteachingSignal[1] = std::max(-0.95, dteachingSignal[1]-0.1);
			dteachingSignal[2] = std::max(-0.95, dteachingSignal[2]-0.1);
			dteachingSignal[3] = std::max(-0.95, dteachingSignal[3]-0.1);
			controller->setSensorTeachingSignal(dteachingSignal, 4); // teaching signal and its length 
			printf("Distal  Teaching  Signal: %f, %f, %f, %f\n", dteachingSignal[0], dteachingSignal[1], dteachingSignal[2], dteachingSignal[3]);
			handled = true;
			break;
	}	 
	fflush(stdout);
  return handled;
}

virtual void bindingDescription(osg::ApplicationUsage & au) const 
{
	au.addKeyboardMouseBinding("Distal Teaching: i","increase error (nimm: forward)");
	au.addKeyboardMouseBinding("Distal Teaching: k","decrease error (nimm: backward)");
}

};

int main (int argc, char **argv)
{ 
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
