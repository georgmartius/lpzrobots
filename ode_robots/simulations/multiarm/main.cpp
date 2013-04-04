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
 *   -handling of starting guilogger moved to simulation.cpp               *
 *    (is in internal simulation routine now)                              *
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off     *
 *   -CTRL-G now restarts the GuiLogger                                    *
 *                                                                         *
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
#include <selforg/selectiveone2onewiring.h>

// used robot
#include <ode_robots/arm.h>
#include <ode_robots/speedsensor.h>

// used arena
#include <ode_robots/playground.h>

// used passive spheres
#include <ode_robots/passivesphere.h>

// used controller
#include <selforg/sinecontroller.h>
#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorbigmodel.h>
#include <selforg/multilayerffnn.h>
#include <selforg/derbigcontroller.h>
#include <selforg/replaycontroller.h>

#include "multisat.h"

using namespace lpzrobots;

Arm* arm;

//#define REPLAY

#ifdef REPLAY
ReplayController* controller;
#else
InvertMotorNStep* controller;
#endif

// target of reaching task in euklidian coordinates
double target[]={-1,2,4};

class ThisSim : public Simulation
{
public:
  bool switchedToRL;
  MultiSat* multisat;

  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // initial camera position and viewpoint
    setCameraHomePos(Pos(13.6696, 9.12317, 5.54366),  Pos(119.528, -8.6947, 0)); // watch robot
    //setCameraHomePos(Pos(2.69124, 4.76157, 8.87839),  Pos(134.901, -47.8333, 0)); // control initial arm config
    //(Pos(4.51276, 15.1867, 4.2256),  Pos(157.899, -1.90167, 0)); // frontal, ganz drauf
    //

    switchedToRL=false;

    // initialization
    global.odeConfig.noise=0.1;

    Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(18, 0.2, 1.0));
    playground->setColor(Color(0.88f,0.4f,0.26f,0.2f));
    playground->setPosition(osg::Vec3(0,0,0.05));
    global.obstacles.push_back(playground);

    //                Color c(osgHandle.color);
    //    c.alpha() = 0.4;
    //                OsgHandle osgHandle_target = osgHandle.changeColor(c);
    //
    //                Sphere* targetSphere = new Sphere(0.5);
    //                targetSphere->init(odeHandle, 0, osgHandle_target);
    //                osg::Matrix tp = osg::Matrix::translate(target[0], target[1], target[2]);
    //                targetSphere->setPose(tp);
    //                FixedJoint* anker = new FixedJoint(global.environment /* fixation to ground*/, targetSphere);
    //                anker->init(odeHandle, osgHandle);

    //                // X-direction sphere
    //                PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
    //    s1->setPosition(osg::Vec3(4,0,0));
    //                s1->setTexture("Images/dusty.rgb");
    //                global.obstacles.push_back(s1);
    //
    //                // Y-direction sphere
    //                PassiveSphere* s2 = new PassiveSphere(odeHandle, osgHandle, 1.5);
    //    s2->setPosition(osg::Vec3(0,4,0));
    //                s2->setTexture("Images/dusty.rgb");
    //                global.obstacles.push_back(s2);

    ArmConf conf = Arm::getDefaultConf();
    // the arm will have joint sensors, position sensors and speed senors
    conf.useJointSensors=true;
    conf.withContext=true;
    conf.sensors.push_back(new SpeedSensor(5));
    OdeHandle armHandle = odeHandle;
    armHandle.substance.toFoam(3);
    arm = new Arm(armHandle, osgHandle, conf, "Arm");

    ((OdeRobot*)arm)->place(Position(-0.7,0.9,0.0));
    // fixation of cuboid base
    FixedJoint* anker = new FixedJoint(arm->getMainObject(), global.environment /* fixation to ground*/);
    anker->init(odeHandle, osgHandle);

    global.configs.push_back(arm);


#ifdef REPLAY
    const char* replayfilename="IMNS_Arm_nice_swing_30min.sel.log";
    controller = new ReplayController(replayfilename,true);
#else
    // PSEUDOLINEAR MODEL CONTROLLER
    InvertMotorNStepConf confi= InvertMotorNStep::getDefaultConf();
    confi.useSD=true;
    confi.someInternalParams=true;

    controller = new InvertMotorNStep(confi);
    controller->setParam("adaptrate", 0.000);
    controller->setParam("epsC", 0.03);
    controller->setParam("epsA", 0.1);
    controller->setParam("rootE", 0);
    controller->setParam("squashsize", 0.001); // maximal possible update in one step
    controller->setParam("steps", 1);
    controller->setParam("s4avg", 1); // zeitglaettung controllerinput ueber # steps
    controller->setParam("s4del", 1); // verzoegerung bis motor echt greift
#endif


    global.configs.push_back(controller);
    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    // create pointer to selectiveWiring
    //    AbstractWiring* wiring = new SelectiveOne2OneWiring(new ColorUniformNoise(0.1), new select_from_to(0,2));
//     std::list<PlotOption> l;
//     l.push_back(PlotOption(GuiLogger,Robot,5));
//    OdeAgent*  agent = new OdeAgent(l);
    OdeAgent*  agent = new OdeAgent(global);

    MultiSatConf msc = MultiSat::getDefaultConf();
    msc.controller = controller;
    msc.numContext = 6; // use first X sensors as context
    msc.numHidden = 3;
    msc.numSats = 32;
    msc.tauC = 500;
    msc.tauE1 = 2;
    msc.tauE2 = 20;
    msc.eps0 = 0.01;
    msc.penalty = 5;
    multisat = new MultiSat(msc);
    global.configs.push_back(multisat);
    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    agent->init(multisat, arm, wiring);

    //  agent->setTrackOptions(TrackRobot(true, false, false,50));
    global.agents.push_back(agent);

    //-----------------------
    // switch off/change gravity
    global.odeConfig.setParam("gravity",-3.00);
    //     global.odeConfig.setParam("realtimefactor",0);

    // show (print to console) params of all objects in configurable list

    //printf("===== started. =====\n");

    // transform target into shoulder centered coordinates
    //                arm->scaleShoulderCentered(target);
    printf("target shoulder centered = (%f, %f, %f)\n", target[0], target[1], target[2]);
  } //start-end

  // add distal teaching signal (desired behavior! not error)
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
  {
    //        double sineRate=30;
    //         double phaseShift=0.65;
    if(globalData.time > 60*60 && !switchedToRL){
      multisat->setParam("rlmode",1);
      printf("RL started\n");
      switchedToRL=true;
    }

  }; // addCallback-end


  //Funktion die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    char filename[1024];
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'n' :
          std::cout << "Please type a filename stem:";
          std::cin >> filename;
          if(multisat) multisat->storeSats(filename);
          break;
        default:
          return false;
          break;
        }
    }
    return false;
  }

  /*virtual*/ void bindingDescription(osg::ApplicationUsage & au) const
  {
    au.addKeyboardMouseBinding("Distal Teaching: i","increase error (nimm: forward)");
    au.addKeyboardMouseBinding("Distal Teaching: k","decrease error (nimm: backward)");
  }

  virtual void end(GlobalData& globalData)
  {
    //printf("TREFFER: %d\n", arm->getHitCounter());
  }

};
//}; // class-end

int main (int argc, char **argv)
{
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
