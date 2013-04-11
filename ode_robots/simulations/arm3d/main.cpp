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
 *   -handling of starting guilogger moved to simulation.cpp                   *
 *    (is in internal simulation routine now)                                   *
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off     *
 *   -CTRL-G now restarts the GuiLogger                                         *
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

// used robot
#include <ode_robots/arm.h>

// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>

// used controller
#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorbigmodel.h>
#include <selforg/multilayerffnn.h>
#include <selforg/derbigcontroller.h>

using namespace lpzrobots;

Arm* arm;

// #define BIGModel;

#ifdef BIGModel
InvertMotorBigModel* controller;
#else
InvertMotorNStep* controller;
#endif

// distal learning stuff
bool dteaching;
double* dteachingSignal;
int dteachingLen;

// target of reaching task in euklidian coordinates
double target[]={-1,2,4};

class ThisSim : public Simulation
{
public:

  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // initial camera position and viewpoint
    setCameraHomePos(Pos(13.6696, 9.12317, 5.54366),  Pos(119.528, -8.6947, 0)); // watch robot
    //setCameraHomePos(Pos(2.69124, 4.76157, 8.87839),  Pos(134.901, -47.8333, 0)); // control initial arm config
    //(Pos(4.51276, 15.1867, 4.2256),  Pos(157.899, -1.90167, 0)); // frontal, ganz drauf
    //

    // initialization
    global.odeConfig.noise=0.1;

    Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(18, 0.2, 1.0));
    playground->setColor(Color(0.88f,0.4f,0.26f,0.2f));
    playground->setPosition(osg::Vec3(0,0,0));
    global.obstacles.push_back(playground);

    //                Color c(osgHandle.color);
    //    c.alpha() = 0.4;
    //                OsgHandle osgHandle_target = osgHandle.changeColor(c);
    //
    //                Sphere* targetSphere = new Sphere(0.5);
    //                targetSphere->init(odeHandle, 0, osgHandle_target);
    //                osg::Matrix tp = osg::Matrix::translate(target[0], target[1], target[2]);
    //                targetSphere->setPose(tp);
    //                FixedJoint* anker = new FixedJoint(0 /* fixation to ground*/, targetSphere);
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
    conf.withContext=false;
    conf.useJointSensors=true;
    //    conf.sensors.push_back(new SpeedSensor(5));

    OdeHandle armHandle = odeHandle;
    armHandle.substance.toFoam(3);
    arm = new Arm(armHandle, osgHandle, conf, "Arm");

    ((OdeRobot*)arm)->place(Position(-0.7,0.9,0.01));
    // fixation of cuboid base
    FixedJoint* anker = new FixedJoint(arm->getMainObject(), global.environment /* fixation to ground*/);
    anker->init(odeHandle, osgHandle);

    global.configs.push_back(arm);

#ifdef BIGModel
    // BIGMODEL-CONTROLLER
    InvertMotorBigModelConf cc = InvertMotorBigModel::getDefaultConf();
    //        DerBigControllerConf cc = DerBigController::getDefaultConf();
    cc.someInternalParams=true;
    cc.useS=false;
    std::vector<Layer> layers;
    // Layer: size, factor_bias, act-fct (default: lin), dact-fct (default: lin)
    layers.push_back(Layer(25, 0.5 , FeedForwardNN::tanh));
    layers.push_back(Layer(10,0.5));
    // true -bypass (uebergeht nichtlin schicht)
    MultiLayerFFNN* net = new  MultiLayerFFNN(0.05, layers, true);
    //                MultiLayerFFNN* net = new MultiLayerFFNN(0.05, layers);
    cc.model = net;
    cc.modelInit = 1;
    cc.cInit = 1.5;
    controller = new InvertMotorBigModel(cc);
    //                controller = new DerBigController(cc);
    controller->setParam("adaptrate",0);
    controller->setParam("epsA",0.01);
    controller->setParam("epsC",0.05);
    controller->setParam("rootE",3);
    controller->setParam("teacher",0.5);
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

    //AbstractController* controller = new SineController();
    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent*  agent = new OdeAgent(global);
    agent->init(controller, arm, wiring);

    //  agent->setTrackOptions(TrackRobot(true, false, false,50));
    global.agents.push_back(agent);

    //-----------------------
    // switch off/change gravity
    global.odeConfig.setParam("gravity",-3);
    //     global.odeConfig.setParam("realtimefactor",0);

    // show (print to console) params of all objects in configurable list

    //printf("===== started. =====\n");

    dteaching=false;
    dteachingLen = arm->getSensorNumber();
    dteachingSignal = new double[dteachingLen];

    // transform target into shoulder centered coordinates
    //                arm->scaleShoulderCentered(target);
    // printf("target shoulder centered = (%f, %f, %f)\n", target[0], target[1], target[2]);
  } //start-end

  // add distal teaching signal (desired behavior! not error)
  /*virtual*/ void addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
  {
    //        double sineRate=30;
    //         double phaseShift=0.65;

    //                 double pos[3];
    //                 double lambda=0.1; // 1 - target is target for each timestep, 0<..<1 - intermediate targets
    //                 if(dteaching)
    //                 {
    //                         arm->getEndeffectorPosition(pos);
    //                         arm->scaleShoulderCentered(pos);
    //                         // reaching into the right direction (target-pos)
    //                         dteachingSignal[0]=(1-lambda)*pos[0]+lambda*target[0];
    //                         dteachingSignal[1]=(1-lambda)*pos[1]+lambda*target[1];
    //                         dteachingSignal[2]=(1-lambda)*pos[2]+lambda*target[2];
    //                         dteachingSignal[3]=0;
    //                         // Bemerkung: durch Clipping in InvertMotorNStep y-Sollsignale fast immer -1 oder 1!
    //                         // maybe therefore no reaching?
    //                         // printf("setdone (%d) %f %f %f %f\n", dteachingLen, dteachingSignal[0], dteachingSignal[1], dteachingSignal[2], dteachingSignal[3]);
    //                         controller->setSensorTeachingSignal(dteachingSignal, 4); // teaching signal and its length
    //                 }
  }; // addCallback-end

  // //Funktion die eingegebene Befehle/kommandos verarbeitet
  // /*virtual*/ bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  // {
  //         if (!down) return false;
  //         bool handled = false;
  //         switch ( key )
  //         {
  //                 case 'u' :
  //                         dteaching=!dteaching;Color(1.1,1,1.4);
  //                         if(dteaching)
  //                           arm->getMainPrimitive()->setColor(Color(0.6,0.5,0.9));
  //                         else{
  //                           arm->getMainPrimitive()->setColor(Color(1.1,1,1.4));
  //                         }
  //                         arm->resetHitCounter();
  //                         printf("Distal teaching %s.\n", dteaching ? "on" : "off");
  //                         break;
  //                 case 'i' :
  //                         dteachingSignal[0] = std::min(0.95, dteachingSignal[0]+0.1);
  //                         dteachingSignal[1] = std::min(0.95, dteachingSignal[1]+0.1);
  //                         dteachingSignal[2] = std::min(0.95, dteachingSignal[2]+0.1);
  //                         dteachingSignal[3] = std::min(0.95, dteachingSignal[3]+0.1);
  //                         controller->setSensorTeachingSignal(dteachingSignal, 4); // teaching signal and its length
  //                         printf("DistalTeachingSignal: %f, %f, %f, %f\n", dteachingSignal[0], dteachingSignal[1], dteachingSignal[2], dteachingSignal[3]);
  //                         handled = true;
  //                         break;
  //                 case 'k' :
  //                         dteachingSignal[0] = std::max(-0.95, dteachingSignal[0]-0.1);
  //                         dteachingSignal[1] = std::max(-0.95, dteachingSignal[1]-0.1);
  //                         dteachingSignal[2] = std::max(-0.95, dteachingSignal[2]-0.1);
  //                         dteachingSignal[3] = std::max(-0.95, dteachingSignal[3]-0.1);
  //                         controller->setSensorTeachingSignal(dteachingSignal, 4); // teaching signal and its length
  //                         printf("Distal  Teaching  Signal: %f, %f, %f, %f\n", dteachingSignal[0], dteachingSignal[1], dteachingSignal[2], dteachingSignal[3]);
  //                         handled = true;
  //                         break;
  //         }
  //         fflush(stdout);
  //   return handled;
  // }

  /*virtual*/ void bindingDescription(osg::ApplicationUsage & au) const
  {
    au.addKeyboardMouseBinding("Distal Teaching: i","increase error (nimm: forward)");
    au.addKeyboardMouseBinding("Distal Teaching: k","decrease error (nimm: backward)");
  }

  virtual void end(GlobalData& globalData)
  {
    //        printf("TREFFER: %d\n", arm->getHitCounter());
  }

};
//}; // class-end

int main (int argc, char **argv)
{
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
