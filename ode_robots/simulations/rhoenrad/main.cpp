/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *
 ***************************************************************************/
#include <stdio.h>
#include <selforg/noisegenerator.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>

// controller
//#include <selforg/sox.h>
#include "sox.h"

#include <selforg/motorbabbler.h>
//#include <selforg/derlininvert.h>
//#include <selforg/dercontroller.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include <ode_robots/joint.h>

// used arena
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivebox.h>

#include <ode_robots/addsensors2robotadapter.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/plattfussschlange.h>
#include <ode_robots/schlangeservo.h>
#include <ode_robots/schlangeservo2.h>
// used robot
#include "rhoenrad.h" // if robot is local

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:
  enum Grounds { Normal, Octa, Pit, Uterus, Stacked };

  // playground parameter
  const static double widthground = 1025.85;// 100; //1.3;
  const static double heightground = .8;// 1.2;
  const static double diamOcta = 2;
  const static double pitsize = 2;
  const static double pitheight = 2;
  const static double uterussize = 1;


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos (Pos(3.46321, 10.6081, 2.74255),  Pos(161.796, -3.69849, 0));

    int humanoids = 1;

    bool fixedInAir = true;
    reckturner = false;
    // Playground types
    addParameterDef("centerforce", &centerforce, .0);//2.0
    addParameterDef("forwardforce", &forwardforce, 0.0);
    center = osg::Vec3(20,20,3);
    forcepoint = 0;
    global.configs.push_back(this);

    fixator=0;
    reckLeft = reckRight = 0;
    reck = 0;

    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",5);//4);
    global.odeConfig.setParam("noise",0.0);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.004);//0.004);
    global.odeConfig.setParam("gravity", -6);
    //    global.odeConfig.setParam("cameraspeed", 250);
    //  int chessTexture = dsRegisterTexture("chess.ppm");


    //************************* SELECT PLAYGROUND HERE ******************?
    setupPlaygrounds(odeHandle, osgHandle, global,  Normal);



   for (int i=0; i< humanoids; i++){ //Several humans
     if (i>0) reckturner=false;

     // normal servos
     //RhoenradConf conf = Rhoenrad::getDefaultConf();
     // velocity servos
     RhoenradConf conf = Rhoenrad::getDefaultConfVelServos();
     conf.relWheelmass=4;
     conf.powerFactor = .25;// .95;//.65;//5;
     conf.onlyPrimaryFunctions = false;

     conf.useOrientationSensor=false;

     conf.jointLimitFactor = 1.1;


     OdeHandle skelHandle=odeHandle;
     // skelHandle.substance.toMetal(1);
    //     skelHandle.substance.toPlastic(.5);//TEST sonst 40
     // skelHandle.substance.toRubber(5.00);//TEST sonst 40
     Rhoenrad* human = new Rhoenrad(skelHandle, osgHandle,conf, "Humanoid");
     robot=human;
     human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)
                  //   *osg::Matrix::translate(-.2 +2.9*i,0,1));
                  *osg::Matrix::translate(.2*i,2*i,.841/*7*/ +2*i));
     global.configs.push_back(human);


     if( fixedInAir){
       Primitive* trunk = human->getMainPrimitive();

       fixator = new FixedJoint(trunk, global.environment);
       //       // fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) ,                    Axis(0,0,1), Axis(0,1,0));
       fixator->init(odeHandle, osgHandle);
     }

     // attention: This sox here is now called SoML in selforg/controller
     SoXConf sc = SoX::getDefaultConf();
     sc.useHiddenContr=true;
     sc.useHiddenModel=true;
     sc.someInternalParams=true;
     sc.useS=true;
     AbstractController* controller = new SoX(sc);
     controller->setParam("epsC",0.05);
     controller->setParam("epsA",0.05555555);
     controller->setParam("harmony",0.0);
     controller->setParam("s4avg",5.0);

     //     AbstractController* controller = new BasicController(cc);
     //   AbstractController* controller = new SineController(1<<14); // only motor 14
     // controller = new MotorBabbler();
     //AbstractController* controller = new SineController(0,SineController::Impulse);

     global.configs.push_back(controller);

     // create pointer to one2onewiring
     One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
     // DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
     // c.useId = true;
     // c.useFirstD = false;
     // DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise(.2) );

     OdeAgent* agent = new OdeAgent(global);
     agent->init(controller, human, wiring);
     //agent->startMotorBabblingMode(5000);
     //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
     global.agents.push_back(agent);
   }// Several humans end


  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control &&!pause){
      if(centerforce!=0){
        // force to center
        FOREACH(vector<OdeAgent*> , globalData.agents, a){
          Primitive* body = (*a)->getRobot()->getMainPrimitive();
          osg::Vec3 pos = body->getPosition();
          osg::Vec3 d = (center - pos);
          dBodyAddForce(body->getBody(), d.x()*centerforce, d.y()*centerforce,  d.z()*centerforce*10.8);//1.8
        }
      }
      if(forwardforce!=0){
        // force forwards
        FOREACH(vector<OdeAgent*> , globalData.agents, a){
          Primitive* body = (*a)->getRobot()->getMainPrimitive();
          osg::Matrix pose = body->getPose();
          // transform a local point ahead of robot into global coords
          // note that the internal corrd of the main primitive has z towards the front
          Pos point = (Pos(0,0,1)*pose );
          point.z()=pose.getTrans().z();  // only use x,y component (this can be commented out)
          Pos d = (point - pose.getTrans());
          d.normalize();

          dBodyAddForce(body->getBody(),
                        d.x()*forwardforce, d.y()*forwardforce, d.z()*forwardforce);
          if(!forcepoint){
            forcepoint = new Sphere(0.1);
            forcepoint->init(odeHandle, 0, osgHandle /*osgHandle.changeAlpha(0.4)*/,
                             Primitive::Geom | Primitive::Draw);
  }
          forcepoint->setPosition(point);
          forcepoint->update();
        }
      }
    }
   //  if(control &&!pause && centerforce!=0){
//       // force to center
//       FOREACH(vector<OdeAgent*> , globalData.agents, a){
//         Primitive* body = (*a)->getRobot()->getMainPrimitive();
//         osg::Vec3 pos = body->getPosition();
//         osg::Vec3 d = (center - pos);
//         dBodyAddForce(body->getBody(), d.x()*centerforce, d.y()*centerforce,  d.z()*centerforce*.8);
//       }
//     }
//       if(forwardforce!=0){
//         // force forwards
//         FOREACH(vector<OdeAgent*> , globalData.agents, a){
//           Primitive* body = (*a)->getRobot()->getMainPrimitive();
//           osg::Matrix pose = body->getPose();
//           // transform a local point ahead of robot into global coords
//           // note that the internal corrd of the main primitive has z towards the front
//           Pos point = (Pos(0,0,1)*pose );
//           point.z()=pose.getTrans().z();  // only use x,y component (this can be commented out)
//           Pos d = (point - pose.getTrans());
//           d.normalize();

//           dBodyAddForce(body->getBody(),
//                         d.x()*forwardforce, d.y()*forwardforce, d.z()*forwardforce);
//           if(!forcepoint){
//             forcepoint = new Sphere(0.1);
//             forcepoint->init(odeHandle, 0, osgHandle /*osgHandle.changeAlpha(0.4)*/,
//                              Primitive::Geom | Primitive::Draw);
//           }
//           forcepoint->setPosition(point);
//           forcepoint->update();
//         }
//       }
  };

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    Substance s;
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'x':
          if(fixator) delete fixator;
          fixator=0;
          return true;
          break;
        case 'i':
          if(playground) {
            s = playground->getSubstance();
            s.hardness*=1.5;
            cout << "hardness " << s.hardness << endl;
            playground->setSubstance(s);
          }
          return true;
          break;
        case 'j':
          if(playground) {
            s = playground->getSubstance();
            s.hardness/=1.5;
            cout << "hardness " << s.hardness << endl;
            playground->setSubstance(s);
          }
          return true;
          break;
        case 'r':
          if(robot) {
            Primitive* wheel = robot->getAllPrimitives().front();
            Axis a(0,0,100);
            wheel->applyTorque(Pos(wheel->toGlobal(a)));
          }
          return true;
          break;
        case 'R':
          if(robot) {
            Primitive* wheel = robot->getAllPrimitives().front();
            Axis a(0,0,-100);
            wheel->applyTorque(Pos(wheel->toGlobal(a)));
          }
          return true;
          break;
        default:
          return false;
          break;
        }
    }
    return false;
  }

  void setupPlaygrounds(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, Grounds ground){
    switch (ground){
    case Normal:
      {
        playground = new Playground(odeHandle, osgHandle,osg::Vec3(widthground, 0.208, heightground));
        playground->setColor(Color(1.,1.,1.,.99));
        //playground->setGroundTexture("Images/really_white.rgb");
        //        playground->setGroundTexture("Images/desert.jpg");
        playground->setGroundTexture("Images/sand.jpg");
        //playground->setGroundColor(Color(54.0/255,.5,54.0/255));
        playground->setPosition(osg::Vec3(0,0,.1));
        //      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(1.0875, 8.8, 1.3975));
        //       playground->setColor(Color(0.88f,0.4f,0.26f,1));
        // playground->setPosition(osg::Vec3(20,20,.5));
        Substance substance;
        substance.toRubber(5);
        //   substance.toMetal(1);
        playground->setGroundSubstance(substance);
        global.obstacles.push_back(playground);
        /*    double xboxes=0.0;
              double yboxes=0.0;*/
        double xboxes=0;//15;//19.0;
        double yboxes=0;//15;
        double boxdis=.9;//.45;//1.6;
        for (double j=0.0;j<xboxes;j++)
          for(double i=0.0; i<yboxes; i++) {
            double xsize= .6;//1.0;
            double ysize= .5;//.25;
            double zsize=.4;
            PassiveBox* b =
              new PassiveBox(odeHandle,
                             osgHandle, osg::Vec3(xsize,ysize,zsize),0.0);
            b->setPosition(Pos(20+boxdis*(i-(xboxes-1)/2.0),20+boxdis*(j-(yboxes-1)/2.0), 0.01));
            //         b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
            //         b->setTexture("Images/light_chess.rgb");
            global.obstacles.push_back(b);
          }
        break;
      }
    case Octa:
      {
        playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(diamOcta, 0.2,/*Height*/ 10), 12,false);
        playground->setTexture("Images/really_white.rgb");
        playground->setColor(Color(0.4,0.8,0.4,0.2));
        playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
        global.obstacles.push_back(playground);
        break;
      }
    case Pit:
      {
        // we stack two playgrounds in each other.
        // The outer one is hard and the inner one is softer
        int anzgrounds=2;
        Substance soft = Substance::getRubber(5);
        double thicknessSoft = 0.1;
        for (int i=0; i< anzgrounds; i++){
          OdeHandle myHandle = odeHandle;
          if(i==0){
            myHandle.substance = soft;
          }else{
            myHandle.substance.toMetal(1);
          }
          Playground* playground = new Playground(myHandle, osgHandle,
                                                  osg::Vec3(pitsize+2*thicknessSoft*i, thicknessSoft + 12*i, pitheight),
                                                  1, i==(anzgrounds-1));
          if(i==(anzgrounds-1)){ // set ground also to the soft substance
            playground->setGroundSubstance(soft);
          }
          if(i==0) this->playground=playground;
          playground->setColor(Color(0.5,0.1,0.1,i==0 ? 0 : .99)); // inner wall invisible
          playground->setPosition(osg::Vec3(0,0,thicknessSoft)); // playground positionieren und generieren
          global.obstacles.push_back(playground);
        }

        break;
      }
    case Uterus:
      {
        // we stack two playgrounds in each other.
        // The outer one is hard (and invisible) and the inner one is soft
        int anzgrounds=2;
        // this is the utterus imitation: high slip, medium roughness, high elasticity, soft
        Substance uterus(0.2/*roughness*/, 0.1 /*slip*/,
                         .5 /*hardness*/, 0.95 /*elasticity*/);
        double thickness = 0.4;
        for (int i=0; i< anzgrounds; i++){
          OdeHandle myHandle = odeHandle;
          if(i==0){
            myHandle.substance = uterus;
          }else{
            myHandle.substance.toMetal(.2);
          }
          Playground* playground = new Playground(myHandle, osgHandle,
                                                  osg::Vec3(uterussize+2*thickness*i,
                                                            i==0 ? thickness : .5, pitheight),
                                                  1, i==0);
          playground->setTexture("Images/dusty.rgb");
          if(i==0){ // set ground also to the soft substance
            playground->setGroundSubstance(uterus);
          }
          if(i==0) this->playground=playground;
          playground->setColor(Color(0.5,0.1,0.1,i==0? .2 : 0)); // outer ground is not visible (alpha=0)
          playground->setPosition(osg::Vec3(0,0,i==0? thickness : 0 )); // playground positionieren und generieren
          global.obstacles.push_back(playground);
        }

        break;
      }
    case Stacked:
      {
        int anzgrounds=2;
        for (int i=0; i< anzgrounds; i++){
          playground = new Playground(odeHandle, osgHandle, osg::Vec3(10+4*i, .2, .95+0.15*i), 1, i==(anzgrounds-1));
          OdeHandle myhandle = odeHandle;
          //      myhandle.substance.toFoam(10);
          // playground = new Playground(myhandle, osgHandle, osg::Vec3(/*base length=*/50.5,/*wall = */.1, /*height=*/1));
          playground->setPosition(osg::Vec3(0,0,0.2)); // playground positionieren und generieren

          global.obstacles.push_back(playground);
        }
        break;
      }
    }

  }

  Joint* fixator;
  Joint* reckLeft;
  Joint* reckRight;
  Primitive* reck;
  //  Playground* playground;
  AbstractGround* playground;
  OdeRobot* robot;
  double hardness;
  bool reckturner;
  osg::Vec3 center;
  double centerforce;
  double forwardforce;

  Primitive* forcepoint;


};


int main (int argc, char **argv)
{
  ThisSim sim;
  sim.setCaption("lpzrobots Simulator             playfulmachines.com");
  return sim.run(argc, argv) ? 0 : 1;

}

