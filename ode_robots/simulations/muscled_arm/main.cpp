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
 *   Revision 1.6  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.5  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.4  2006/08/04 16:25:14  martius
 *   bugfixing
 *
 *   Revision 1.3  2006/07/14 12:23:50  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.4.9  2006/06/25 17:01:55  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.1.4.8  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.1.4.7  2006/03/31 16:11:19  fhesse
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
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>

// used robot
#include <ode_robots/nimm4.h>
#include <ode_robots/arm2segm.h>
#include <ode_robots/muscledarm.h>

// used arena
//#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>

// used controller
#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/invertnchannelcontroller.h>
#include <selforg/sox.h>

using namespace lpzrobots;

MuscledArm* arm;

class ThisSim : public Simulation {
public:


  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // initial camera position and viewpoint
    setCameraHomePos(Pos(0.602703, -0.643497, 1.96501),  Pos(52.2474, -56.2528, 0));
    setCameraMode(Static);
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

    arm = new MuscledArm(odeHandle, osgHandle, conf, "Arm");
    // set muscled arm parameters
    //arm->setParam("damping",20);
    arm->setParam("factorMotors",5);


    ((OdeRobot*)arm)->place(Position(0,0,0));
    global.configs.push_back(arm);

    // create pointer to controller
    // push controller in global list of configurables
    //    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    //    cc.buffersize=30;
    //AbstractController *controller = new InvertMotorNStep(cc);
    // AbstractController *controller = new InvertNChannelController(30);

    AbstractController *controller = new Sox(1.0);

    // AbstractController *controller = new   SineController(1);
    // controller->setParam("period",200);
    // controller->setParam("phaseshift",1);
    global.configs.push_back(controller);

    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, arm, wiring);
    agent->setTrackOptions(TrackRobot(false, false, false, true, "55" ,50));
    agent->setTraceLength(10000);
    agent->setTraceThickness(0.003);
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
    //     agent = new OdeAgent(global);
    //     agent->init(controller, arm, wiring);
    //     //  agent->setTrackOptions(TrackRobot(true, false, false,50));
    //     global.agents.push_back(agent);

    //-----------------------
    // switch off gravity
    global.odeConfig.setParam("gravity",0);
    //     global.odeConfig.setParam("realtimefactor",0);

    // show (print to console) params of all objects in configurable list

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
    //         arm->force_[0]=0;
    //         arm->force_[1]=0;
    //         arm->force_[2]=0.5;
    //         arm->force_[3]=0.5;
    //         arm->force_[4]=-0.5;
    //         arm->force_[5]=-0.5;
    //         break;
    //       }
    //     for (int i=0; i<6; i++){
    //       std::cout<<arm->force_[i]<<"  ";
    //     }
    //     std::cout<<"\n";

  }


  // add own key handling stuff here, just insert some case values
  // note: this is the normal signature (look above)
  //   virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  //   {
  //     if (down) { // only when key is pressed, not when released
  //       switch ( (char) key )
  //         {
  //         default:
  //           return false;
  //           break;
  //         }
  //     }
  //     return false;
  //   }

};

int main (int argc, char **argv)
{
  ThisSim sim;
  // run simulation
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

