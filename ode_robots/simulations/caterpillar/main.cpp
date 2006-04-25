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
 *   Revision 1.1.2.1  2006-04-25 08:05:56  robot3
 *   created an own simulation for the caterpillar for testing purposes
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

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include "hurlingsnake.h"
#include "schlangeforce.h"
#include "caterpillar.h"
#include "nimm2.h"
#include "nimm4.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

// plotoptions is a list of possible online output, 
// if the list is empty no online gnuplot windows and no logging to file occurs.
// The list is modified with commandline options, see main() at the bottom of this file
list<PlotOption> plotoptions;

class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)

    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("controlinterval",1);
    // initialization
    
    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(25, 0.2, 1.5));
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
        
    OdeAgent* agent;
    AbstractWiring* wiring;
    OdeRobot* robot;
    AbstractController *controller;
    
    CaterPillar* myCaterPillar;
    CaterPillarConf myCaterPillarConf = DefaultCaterPillar::getDefaultConf();
    //******* R A U P E  *********/
    myCaterPillarConf.segmNumber=8;
    myCaterPillarConf.jointLimit=M_PI/3;
    myCaterPillarConf.motorPower=0.2;
    myCaterPillarConf.frictionGround=0.01;
    myCaterPillar = new CaterPillar ( odeHandle, osgHandle.changeColor(Color(1.0f,0.0,0.0)), myCaterPillarConf, "Raupe1");
    ((OdeRobot*) myCaterPillar)->place(Pos(-5,-5,0.2)); 
    
    InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
    invertnconf.cInit=2.0;
    controller = new InvertMotorNStep(invertnconf);    
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent = new OdeAgent( plotoptions );
    agent->init(controller, myCaterPillar, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(controller);
    global.configs.push_back(myCaterPillar);   
    myCaterPillar->setParam("gamma",/*gb");
      global.obstacles.push_back(s)0.0000*/ 0.0);
  

    //******* S C H L A N G E  (Long)  *********/
//     SchlangeForce* snake;
//     SchlangeConf snakeConf = SchlangeForce::getDefaultConf();
//     snakeConf.motorPower=0.5;
//     snakeConf.segmNumber=8;
//     snakeConf.jointLimit=M_PI/3;
//     snakeConf.frictionGround=0.1;
//     snake = new SchlangeForce ( odeHandle, osgHandle.changeColor(Color(0,0.5,0.8)), snakeConf, "Schlange2" );
//     ((OdeRobot*) snake)->place(Pos(0,0,0)); 
//     controller = new InvertMotorNStep(invertnconf);     
//     wiring = new One2OneWiring(new ColorUniformNoise(0.1));
//     agent = new OdeAgent( list<PlotOption>() );
//     agent->init(controller, snake, wiring);
//     global.agents.push_back(agent);
//     global.configs.push_back(controller);
//     global.configs.push_back(snake);   
//     snake->setParam("gamma",0.0000 0.0);
  

    //******* N I M M  2 *********/
    Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
    nimm2conf.size = 1.6;
    for(int r=0; r < 3; r++) {    
      robot = new Nimm2(odeHandle, osgHandle, nimm2conf);
      robot->place(Pos ((r-1)*5,5,0));
      //    controller = new InvertMotorNStep(10);   
      controller = new InvertMotorSpace(15);   
      controller->setParam("s4avg",10);
      //    controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( list<PlotOption>() );
      agent->init(controller, robot, wiring);
      global.configs.push_back(controller);
      global.agents.push_back(agent);        
    }
    
    //******* N I M M  4 *********/
    for(int r=0; r < 1; r++) {
      robot = new Nimm4(odeHandle, osgHandle);
      robot->place(Pos((r-1)*5,-3,0));
      controller = new InvertMotorSpace(20);
      controller->setParam("s4avg",10); 
      controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( list<PlotOption>() );
      agent->init(controller, robot, wiring);
      global.agents.push_back(agent);        
    }

    //****** H U R L I N G **********/
    for(int r=0; r < 2; r++) {
      HurlingSnake* snake;
      Color c;    
      if (r==0) c=Color(0.8, 0.8, 0);
      if (r==1) c=Color(0,   0.8, 0);
      snake = new HurlingSnake(odeHandle, osgHandle.changeColor(c));
      ((OdeRobot*) snake)->place(Pos(r*5,-6,0.3));
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
      agent = new OdeAgent( list<PlotOption>() );
      agent->init(controller, snake, wiring);
			       global.configs.push_back(controller);
			       global.agents.push_back(agent);     
    }
      showParams(global.configs);
  }
  
};


void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l] [-r seed]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile\n\t-r seed\trandom number seed ", progname);
}

int main (int argc, char **argv)
{ 
  // start with online windows (default: start without plotting and logging)
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger));
  
  // start with online windows and logging to file
  if(contains(argv, argc, "-l")) plotoptions.push_back(PlotOption(GuiLogger_File));
  
  // display help
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}
 
