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
 *   Revision 1.1.4.7  2006-03-31 16:11:19  fhesse
 *   tracing via trackrobot
 *
 *   Revision 1.1.4.6  2006/03/28 14:25:23  fhesse
 *   tracing added
 *
 *   Revision 1.1.4.5  2006/01/31 09:58:49  fhesse
 *   basically working now
 *
 *   Revision 1.1.4.4  2006/01/10 09:36:38  fhesse
 *   moved to osg
 *
 *   Revision 1.1.4.3  2005/12/16 16:32:32  fhesse
 *   command function for manual control added
 *
 *   Revision 1.1.4.2  2005/11/24 16:16:40  fhesse
 *   moved from main branch
 *
 *   Revision 1.2  2005/11/17 16:28:40  fhesse
 *   initial version
 *
 *   Revision 1.1  2005/11/11 15:35:54  fhesse
 *   preinitial version
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
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
#include "nimm4.h"
#include "arm2segm.h"
#include "muscledarm.h"

// used arena
//#include "playground.h"
#include "passivesphere.h"

// used controller
#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/invertnchannelcontroller.h>
// plotoptions is a list of possible online output, 
// if the list is empty no online gnuplot windows and no logging to file occurs.
// The list is modified with commandline options, see main() at the bottom of this file
list<PlotOption> plotoptions;


using namespace lpzrobots;

MuscledArm* arm;

class ThisSim : public Simulation {
public:


  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // initial camera position and viewpoint
    setCameraHomePos(Pos(-0.0707104, 0.092873, 3.64943),  Pos(89.9208, -85.8472, 0));
    // initialization
    global.odeConfig.noise=0.1;


    // passive sphere 
    // setPosition does not work anymore 
//     PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.1);
//     s1->setPosition(osg::Vec3(-0.7,0.9,0.1));
//     s1->setTexture("Images/dusty.rgb");
//     global.obstacles.push_back(s1);


    // muscled arm configuration
    MuscledArmConf  conf = MuscledArm::getDefaultConf();
    conf.jointAngleSensors=false;
    conf.jointAngleRateSensors=true;
    conf.muscleLengthSensors=false;
    conf.jointActuator=false;

    arm = new MuscledArm(odeHandle, osgHandle, conf);
    // set muscled arm parameters
    //arm->setParam("damping",20);
    //arm->setParam("factorMotors",5);


    ((OdeRobot*)arm)->place(Position(0,0,0));
    global.configs.push_back(arm);

    // create pointer to controller
    // push controller in global list of configurables
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.buffersize=30;
    //AbstractController *controller = new InvertMotorNStep(cc);
    AbstractController *controller = new InvertNChannelController(30);
    //AbstractController *controller = new   SineController();
    global.configs.push_back(controller);
  
    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent(plotoptions);
    agent->init(controller, arm, wiring);
    agent->setTrackOptions(TrackRobot(false, false, false, true, "55" ,50));
    agent->init_tracing(10000, 0.003);
    global.agents.push_back(agent);
    //-------------------------



    //       arm = new MuscledArm(odeHandle, osgHandle, conf);
    // //      arm->setParam("damping",20);
    // //      arm->setParam("factorMotors",5);


    //      ((OdeRobot*)arm)->place(Position(10,10,0));
    //      global.configs.push_back(arm);

    //      controller = new   SineController();
    //      global.configs.push_back(controller);
  
    //     // create pointer to one2onewiring
    //     wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    //     // create pointer to agent
    //     // initialize pointer with controller, robot and wiring
    //     // push agent in globel list of agents
    //     agent = new OdeAgent(plotoptions);
    //     agent->init(controller, arm, wiring);
    //     //  agent->setTrackOptions(TrackRobot(true, false, false,50));
    //     global.agents.push_back(agent);

    //-----------------------
    // switch off gravity
    global.odeConfig.setParam("gravity",0);
    //     global.odeConfig.setParam("realtimefactor",0);

    // show (print to console) params of all objects in configurable list 
    showParams(global.configs);
  }
};


// print command line options
void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile\n", progname);
}

//Funktion die eingegebene Befehle/kommandos verarbeitet
void command (const OdeHandle&, GlobalData& globalData, int cmd)
{
  //     //printp ( "Eingabe erfolgt %d (`%c')\n" , cmd , cmd );
  //     switch ( (char) cmd )
  //       {
  //       case '1' : arm->force_[0]+=0.5; break;
  //       case '!' : arm->force_[0]-=0.5; break;

  //       case '2' : arm->force_[1]+=0.5; break;
  //       case '@' : arm->force_[1]-=0.5; break;

  //       case '3' : arm->force_[2]+=0.5; break;
  //       case '#' : arm->force_[2]-=0.5; break;

  //       case '4' : arm->force_[3]+=0.5; break;
  //       case '$' : arm->force_[3]-=0.5; break;

  //       case '5' : arm->force_[4]+=0.5; break;
  //       case '%' : arm->force_[4]-=0.5; break;

  //       case '6' : arm->force_[5]+=0.5; break;
  //       case '^' : arm->force_[5]-=0.5; break;

  //       case 'q' : arm->force_[0]=0; break;
  //       case 'w' : arm->force_[1]=0; break;
  //       case 'e' : arm->force_[2]=0; break;
  //       case 'r' : arm->force_[3]=0; break;
  //       case 't' : arm->force_[4]=0; break;
  //       case 'y' : arm->force_[5]=0; break;

  //       case 'a' : for (int i=0; i<6; i++) arm->force_[i]=0; break;

  //       case 'z' : 
  // 	arm->force_[0]=0;
  // 	arm->force_[1]=0;
  // 	arm->force_[2]=0.5;
  // 	arm->force_[3]=0.5;
  // 	arm->force_[4]=-0.5;
  // 	arm->force_[5]=-0.5; 
  // 	break;
  //       }
  //     for (int i=0; i<6; i++){
  //       std::cout<<arm->force_[i]<<"  ";
  //     }
  //     std::cout<<"\n";
    
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

/*
  [Simulation Environment:  odeconfig.h,v - 1.12 ][1804289383]
  noise=              0.100000
  simstepsize=        0.010000
  realtimefactor=     0.000000
  drawinterval=       50.000000
  controlinterval=    5.000000
  gravity=            0.000000
  [muscledarm.cpp,v - 1.2 ][2031419189]
  factorMotors=       0.100000
  factorSensors=      4.000000
  damping=            1.000000
  [invertmotornstep.cpp,v - 1.19 ][240828263]
  epsA=               0.001299
  epsC=               0.000000
  adaptrate=          0.010000
  nomupdate=          0.010000
  desens=             0.000000
  s4delay=            1.000000
  s4avg=              5.000000
  steps=              4.000000
  zetaupdate=         0.000000
  logaE=              0.000000
  rootE=              0.000000
  relativeE=          0.000000
  factorB=            0.200000
  noiseB=             0.001000
*/

/*
  [Simulation Environment:  odeconfig.h,v - 1.12 ][1804289383]
  noise=              0.000000
  simstepsize=        0.010000
  realtimefactor=     0.000000
  drawinterval=       50.000000
  controlinterval=    5.000000
  gravity=            0.000000
  [ muscledarm.cpp,v - 1.2 ][715246296]
  factorMotors=       0.000000
  factorSensors=      10.000000
  damping=            10.000000
  [ sinecontroller.hpp,v - 1.4 ][394186711]
  sinerate=           400.000000
  phaseshift=         1.000000
*/



/*

Use random number seed: 1132245941

[Simulation Environment:  odeconfig.h,v - 1.12 ][1804289383]
noise=              0.100000
simstepsize=        0.010000
realtimefactor=     0.000000
drawinterval=       50.000000
controlinterval=    5.000000
gravity=            0.000000
[ muscledarm.cpp,v - 1.3 ][1577223465]
factorMotors=       2.000000
factorSensors=      4.000000
damping=            20.000000
[ invertmotornstep.cpp,v - 1.19 ][686205722]
epsA=               0.005501
epsC=               0.000000
adaptrate=          0.010000
nomupdate=          0.010000
desens=             0.000000
s4delay=            1.000000
s4avg=              1.000000
steps=              3.000000
zetaupdate=         0.000000
logaE=              0.000000
rootE=              0.000000
relativeE=          0.000000
factorB=            0.200000
noiseB=             0.001000




*/

