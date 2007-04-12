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
 *   $Log$
 *   Revision 1.2  2007-04-12 13:30:03  robot3
 *   tests
 *
 *   Revision 1.1  2007/03/22 08:03:35  robot3
 *   simulation for testing the mutual information of an arbitrary controller
 *
 *   Revision 1.16  2007/01/15 15:17:33  robot3
 *   fixed compile bug
 *
 *   Revision 1.15  2007/01/15 09:38:06  martius
 *   number of agents per type as variables on top of start function
 *   sphere gets axisorientationsensor
 *
 *   Revision 1.14  2006/09/22 06:18:56  robot8
 *   - added SliderWheelie - robot
 *
 *   Revision 1.13  2006/07/23 10:24:09  fhesse
 *   a few std:: added
 *
 *   Revision 1.12  2006/07/14 12:23:55  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.11.4.13  2006/06/25 21:57:47  martius
 *   robot names with numbers
 *
 *   Revision 1.11.4.12  2006/06/25 17:01:57  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.11.4.11  2006/05/24 09:17:10  robot3
 *   snake is now colored (not blue)
 *
 *   Revision 1.11.4.10  2006/05/18 14:16:03  robot3
 *   -inserted SphereRobot3Masses
 *   -inserted CaterPillar
 *   -inserted SchlangeServo2
 *   -removed multiple same robots (performance issues)
 *   -inserted new passiveCapsules
 *   -passiveBoxes look more interesting now
 *   -wall of playground is now transparent (0.2)
 *
 *   Revision 1.11.4.9  2006/05/18 10:26:50  robot3
 *   -made the playground smaller (for shadowing issues)
 *   -changed camera homepos
 *
 *   Revision 1.11.4.8  2006/05/15 13:09:33  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.11.4.7  2006/05/11 12:51:25  robot3
 *   the zoo contains now passive boxes
 *
 *   Revision 1.11.4.6  2006/04/25 09:05:23  robot3
 *   caterpillar is now represented by a box
 *
 *   Revision 1.11.4.5  2006/04/11 13:28:18  robot3
 *   caterpillar is now in the zoo too
 *
 *   Revision 1.11.4.4  2006/03/28 09:55:12  robot3
 *   -main: fixed snake explosion bug
 *   -odeconfig.h: inserted cameraspeed
 *   -camermanipulator.cpp: fixed setbyMatrix,
 *    updateFactor
 *
 *   Revision 1.11.4.3  2006/01/12 15:17:30  martius
 *   *** empty log message ***
 *
 *   Revision 1.11.4.2  2005/11/16 11:27:38  martius
 *   invertmotornstep has configuration
 *
 *   Revision 1.11.4.1  2005/11/15 12:30:22  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.11  2005/11/14 13:02:39  martius
 *   new paramters
 *
 *   Revision 1.10  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include "simulation.h"

#include "odeagent.h"
#include "playground.h"

#include "passivesphere.h"
#include "passivebox.h"
#include "passivecapsule.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include "axisorientationsensor.h"

#include "hurlingsnake.h"
#include "schlangeservo2.h"
#include "caterpillar.h"
#include "nimm2.h"
#include "nimm4.h"
#include "sphererobot3masses.h"
#include "sliderwheelie.h"
#include "shortcircuit.h"

#include <selforg/discretecontrolleradapter.h>
#include <selforg/oneactivemultipassivecontroller.h>
#include <selforg/mutualinformationcontroller.h>


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    int numCater=0;
    int numSchlangeL=0;
    int numNimm2=1;
    int numNimm4=0;
    int numHurling=0;
    int numSphere=0;
    int numSliderWheele=0;


    setCameraHomePos(Pos(-19.15, 13.9, 6.9),  Pos(-126.1, -17.6, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)

    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("realtimefactor",1);
    // initialization

    Playground* playground =
      new Playground(odeHandle, osgHandle,osg::Vec3(18, 0.2, 1.0));
    playground->setColor(Color(0.88f,0.4f,0.26f,0.2f));
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    for(int i=0; i<5; i++){
      PassiveSphere* s =
	new PassiveSphere(odeHandle,
			  osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.2);
      s->setPosition(Pos(i*0.5-2, i*0.5, 1.0));
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);
    }

    for(int i=0; i<5; i++){
      PassiveBox* b =
	new PassiveBox(odeHandle,
			  osgHandle, osg::Vec3(0.2+i*0.1,0.2+i*0.1,0.2+i*0.1));
      b->setPosition(Pos(i*0.5-5, i*0.5, 1.0));
      b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
      b->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(b);
    }

    for(int i=0; i<5; i++){
      PassiveCapsule* c =
	new PassiveCapsule(odeHandle, osgHandle, 0.2f, 0.3f, 0.3f);
      c->setPosition(Pos(i-1, -i, 1.0));
      c->setColor(Color(0.2f,0.2f,1.0f,0.5f));
      c->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(c);
    }

    OdeAgent* agent;
    AbstractWiring* wiring;
    OdeRobot* robot;
    AbstractController *controller;

    //******* R A U P E  *********/
    for(int r=0; r < numCater ; r++) {
      CaterPillar* myCaterPillar;
      CaterPillarConf myCaterPillarConf = DefaultCaterPillar::getDefaultConf();
      myCaterPillarConf.segmNumber=3+r;
      myCaterPillarConf.jointLimit=M_PI/3;
      myCaterPillarConf.motorPower=0.2;
      myCaterPillarConf.frictionGround=0.01;
      myCaterPillarConf.frictionJoint=0.01;
      myCaterPillar =
	new CaterPillar ( odeHandle, osgHandle.changeColor(Color(1.0f,0.0,0.0)),
			  myCaterPillarConf, "Raupe" );//+ std::itos(r));
      ((OdeRobot*) myCaterPillar)->place(Pos(-5,-5+2*r,0.2));

      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit=2.0;
      controller = new InvertMotorSpace(15);
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( plotoptions );
      agent->init(controller, myCaterPillar, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(myCaterPillar);
      myCaterPillar->setParam("gamma",/*gb");
					global.obstacles.push_back(s)0.0000*/ 0.0);
    }


    //******* S C H L A N G E  (Long)  *********/
    for(int r=0; r < numSchlangeL ; r++) {
      SchlangeServo2* snake;
      SchlangeConf snakeConf = SchlangeServo2::getDefaultConf();
      snakeConf.segmNumber=6+r;
      snakeConf.frictionGround=0.01;

      snake = new SchlangeServo2 ( odeHandle, osgHandle, snakeConf, "SchlangeLong" + std::itos(r));
      ((OdeRobot*) snake)->place(Pos(4,4-r,0));
      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit=2.0;
      controller = new InvertMotorNStep(invertnconf);
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( std::list<PlotOption>() );
      agent->init(controller, snake, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(snake);
    }

/*****************************************************************************************************************/
/*****************************************************************************************************************/
/******************************************** N I M M  2 *********************************************************/

	  Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
	  nimm2conf.size = 1.6;
	  nimm2conf.singleMotor=false;
    for(int r=0; r < numNimm2; r++) {
	    robot = new Nimm2(odeHandle, osgHandle, nimm2conf, "Nimm2_" + std::itos(r));
	    //robot = new ShortCircuit(odeHandle,osgHandle,1,1);
      robot->place(Pos ((r-1)*5,5,0));
	    InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
	    controller = new InvertMotorNStep(invertnconf);
	    invertnconf.cInit = 0.1;
	    controller->setParam( "epsA",0);
	    controller->setParam( "epsC",0);
	    //         controller = new InvertMotorSpace(15);
	    //      controller->setParam("s4avg",10);
	    //controller = new SineController();
	    //  controller->setParam( "sinerate",1);
      //    controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( std::list<PlotOption>() );
	// create DiscreteControllerAdapter
	//	    DiscreteControllerAdapter* discretesizer = new DiscreteControllerAdapter(controller);
	//	    discretesizer->setIntervalCount(3);
	OneActiveMultiPassiveController* onamupaco = new OneActiveMultiPassiveController(controller,"main");
	    MutualInformationController* mic = new MutualInformationController(5);
	    mic->setParam("showF",0);
	    mic->setParam("showP",0);
	    onamupaco->addPassiveController(mic,"mi5");
	    mic = new MutualInformationController(10);
	    mic->setParam("showF",0);
	    mic->setParam("showP",0);
	    onamupaco->addPassiveController(mic,"mi10");
	    mic = new MutualInformationController(20);
	    onamupaco->addPassiveController(mic,"mi20");
	    mic = new MutualInformationController(50);
	    onamupaco->addPassiveController(mic,"mi50");
	    agent->init(onamupaco, robot, wiring);
	global.configs.push_back(controller);
	global.agents.push_back(agent);
    }


/******************************************** N I M M  2 *********************************************************/
/*****************************************************************************************************************/
/*****************************************************************************************************************/
	  
	  
    //******* N I M M  4 *********/
    for(int r=0; r < numNimm4; r++) {
      robot = new Nimm4(odeHandle, osgHandle, "Nimm4_" + std::itos(r));
      robot->place(Pos((r-1)*5,-3,0));
      controller = new InvertMotorSpace(20);
      controller->setParam("s4avg",10);
      controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( std::list<PlotOption>() );
      agent->init(controller, robot, wiring);
      global.agents.push_back(agent);
    }

    //****** H U R L I N G **********/
    for(int r=0; r < numHurling; r++) {
      HurlingSnake* snake;
      Color c;
      if (r==0) c=Color(0.8, 0.8, 0);
      if (r==1) c=Color(0,   0.8, 0);
      snake = new HurlingSnake(odeHandle, osgHandle.changeColor(c), "HurlingSnake_" + std::itos(r));
      ((OdeRobot*) snake)->place(Pos(r*5,-6,0.3));
      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit=1.5;
      controller = new InvertMotorNStep(invertnconf);
      controller->setParam("steps", 2);
      controller->setParam("adaptrate", 0.001);
      controller->setParam("nomupdate", 0.001);
      controller->setParam("factorB", 0);

      // deriveconf = DerivativeWiring::getDefaultConf();
      //     deriveconf.blindMotorSets=0;
      //     deriveconf.useId = true;
      //     deriveconf.useFirstD = true;
      //     deriveconf.derivativeScale = 50;
      //     wiring = new DerivativeWiring(deriveconf, new ColorUniformNoise(0.1));
      wiring = new One2OneWiring(new ColorUniformNoise(0.05));
      agent = new OdeAgent( std::list<PlotOption>() );
      agent->init(controller, snake, wiring);
			       global.configs.push_back(controller);
			       global.agents.push_back(agent);
    }

    //****** S P H E R E **********/
    for(int r=0; r < numSphere; r++) {
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
      Sphererobot3Masses* sphere1 =
	new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)),
				 conf, "Sphere" + std::itos(r), 0.2);
      ((OdeRobot*)sphere1)->place ( Pos( 0 , 0 , 0.1 ));
      controller = new InvertMotorSpace(15);
      controller->setParam("sinerate", 40);
      controller->setParam("phaseshift", 0.0);
      One2OneWiring* wiring2 = new One2OneWiring ( new ColorUniformNoise() );
      agent = new OdeAgent ( plotoptions );
      agent->init ( controller , sphere1 , wiring2 );
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
    }

    /******* S L I D E R - w H E E L I E *********/
    SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
    for(int r=0; r < numSliderWheele; r++) {
      mySliderWheelieConf.segmNumber=8;
      mySliderWheelieConf.jointLimit=M_PI/4;
      mySliderWheelieConf.motorPower=0.4;
      mySliderWheelieConf.frictionGround=0.8;
      mySliderWheelieConf.sliderLength=1;
      mySliderWheelieConf.segmLength=0.4;

      SliderWheelie* mySliderWheelie =
	new SliderWheelie(odeHandle, osgHandle, mySliderWheelieConf, "sliderWheelie" + std::itos(r));
      ((OdeRobot*) mySliderWheelie)->place(Pos(4-2*r,0,0.0));
      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit=1;
      controller = new InvertMotorNStep(invertnconf);
      controller->setParam("steps",2);
      controller->setParam("factorB",0);

      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = false;
      c.useFirstD = true;
      DerivativeWiring* wiring3 = new DerivativeWiring ( c , new ColorUniformNoise(0.1) );
      agent = new OdeAgent(plotoptions);
      agent->init(controller, mySliderWheelie, wiring3);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(mySliderWheelie);
    }


    showParams(global.configs);
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	default:
	  return false;
	  break;
	}
    }
    return false;
  }

};

int main (int argc, char **argv)
{
	/*	std::list<AbstrakteKlasse*> klassenList;
	KonkreteKlasse2* kk2= new KonkreteKlasse2();
	klassenList.push_back(kk2);
	kk2->method(4);
	for (std::list<AbstrakteKlasse*>::iterator i=klassenList.begin();i!=klassenList.end();i++) {
		(*i)->method(5);
	}*/
	 ThisSim sim;
	// run simulation
	return sim.run(argc, argv) ? 0 : 1;
}

