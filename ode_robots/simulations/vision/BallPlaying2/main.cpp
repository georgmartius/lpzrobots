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
 ***************************************************************************/
#include <stdio.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>
#include <selforg/invertmotorspace.h>
#include <selforg/crossmotorcoupling.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/selectivenoisewiring.h>
#include <selforg/semox.h>

#include "soxignorenull.h"

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>


// used arena
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h> // arena
#include <ode_robots/complexplayground.h>

// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

#include <ode_robots/camera.h>
#include <ode_robots/imageprocessors.h>
#include <ode_robots/camerasensors.h>

#include <ode_robots/fourwheeled.h>

#include <ode_robots/operators.h>

#include <osg/Light>
#include <osg/LightSource>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

template<typename T>
std::vector<T> mkVector(const T* v, int len){
  return std::vector<T>(v,v+len);
}

int id=0; // id of simulation  for logfile


// make sure you change also the strings below
enum Version {V1=0, _1r1b, Change, ChangeCtrl, test, V2, V3, V4, V5, V6, Arena4, VMax};
const char* VersionStrings[] = {"V1", "1r1b", "change", "changectrl", "test", "V2", "V3", "V4", "V5", "V6", "Arena4"};

class ThisSim : public Simulation, public Inspectable {
public:
  int motionIndex1    ;
  int motionIndex2    ;
  int posIndex1       ;
  int posIndex2       ;
  int sizeIndex       ;
  int sizeChangeIndex ;

  ThisSim(Version v, double teaching, double sizefactor, int changeEvery)
    : version(v), teaching(teaching), sizefactor(sizefactor), changeEvery(changeEvery) {
    motionIndex1    =0;
    motionIndex2    =0;
    posIndex1       =0;
    posIndex2       =0;
    sizeIndex       =0;
    sizeChangeIndex =0;
  }

  enum Env {Round, Corridor, Bone, Arena};

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    int numSeeing4wheeled = 2;
    int numBlindRobots    = 0;
    bool seeingAreRed     = false;
    bool blind_straight   = true;
    bool useBottom        = false;

    int numBalls          = 0;
    double ballSize       = 0.3;

    Env arena           = Round;
    double radiusRound    = 8;
    double radiusCorr     = 10;
    double bonefactor     = .8;

    double learningrate   = 0.05; // epsC and epsA

    setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));
    setCameraHomePos(Pos(-0.50487, -20.9638, 8.85769),  Pos(-0.789815, -21.674, 0));
    setCameraMode(Static);

    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("noise",0.03);
    if(global.odeConfig.realTimeFactor==0)
      global.odeConfig.setParam("fps",1);

    addParameterDef("attraction", &attraction, 0.000);
    addParameterDef("friction", &friction, 0.1);

    addParameter("sizefactor", &sizefactor); // is set in contructor
    addParameterDef("posfactor", &posfactor, 0.5);
    addParameterDef("sizesetpoint", &sizeSetPoint, sqrt(2));
    addParameterDef("possetpoint",   &posSetPoint, 0);
    addParameterDef("possetpoint2",   &posSetPoint2, 0);

    switch(version){
    case Change:
      ballSize=0.35;
    case V1:
      // V1:  Corridor (10), 1x 4wheeled, 5 Balls (0.85 teaching clip)
      setCameraHomePos(Pos(-7.38466, 10.1764, 3.17434),  Pos(-96.9417, -12.6582, 0));
      arena             = Corridor;
      numSeeing4wheeled = 1;
      numBalls          = 5;
      seeingAreRed      = true;
      break;
    case ChangeCtrl:
      ballSize=0.35;
      arena             = Corridor;
      numBlindRobots    = 1;
      numSeeing4wheeled = 0;
      numBalls          = 5;
      blind_straight    = false;
      break;
    case _1r1b:
      // 1r1b: Corridor (10), 1x 4wheeled, 1 Ball
      setCameraHomePos(Pos(-7.38466, 10.1764, 3.17434),  Pos(-96.9417, -12.6582, 0));
      arena             = Corridor;
      numSeeing4wheeled = 1;
      numBalls          = 1;
      break;
    case test:
      // 1r1b: Corridor (10), 1x 4wheeled, 1 Ball
      setCameraHomePos(Pos(-7.38466, 10.1764, 3.17434),  Pos(-96.9417, -12.6582, 0));
      arena             = Corridor;
      numSeeing4wheeled = 1;
      numBalls          = 1;
      learningrate      = 0.02;
      sizefactor        = 0.1;
      break;
    case V2:
      // V2:  Round (8), 1x 4wheeled, 3 Balls (0.9 teaching clip)
      numSeeing4wheeled = 1;
      sizeSetPoint      = 1;
      numBalls          = 3;
      break;
    case V3:
      // V3:  Round (8), 3x 4wheeled, 1 Balls (0.9 teaching clip) sizesetpoint = 1
      numSeeing4wheeled = 3;
      numBalls          = 1;
      break;
    case V4:
      // V4:  Round (8), 2x 4wheeled red, 1xblind (0.9 teaching clip) sizesetpoint = .5
      numSeeing4wheeled = 2;
      numBlindRobots    = 1;
      seeingAreRed      = true;
      sizeSetPoint      = .5;
      numBalls          = 0;
      break;
    case V5:
      // V5:  Round (6), 4x 4wheeled, sizesetpoint = .3
      numSeeing4wheeled = 10; // epsC=0.01, gamma=0.2
      numBlindRobots    = 1;
      seeingAreRed      = true;
      sizeSetPoint      = .4;
      numBalls          = 0;
      radiusRound       =16;
      useBottom         = true;
      break;
    case V6:
      // V6:  Bone, 1x 4wheeled, 3 balls
      arena             = Bone;
      numSeeing4wheeled = 1;
      seeingAreRed      = false;
      sizeSetPoint      = 2;
      numBalls          = 4;
      learningrate      = 0.02;
      break;
    case Arena4:
      // V6:  Arena, 1x 4wheeled, 4 balls
      arena             = Arena;
      numSeeing4wheeled = 1;
      seeingAreRed      = true;
      numBalls          = 4;
      learningrate      = 0.02;
      friction          = 0.2;
      posfactor         = 1;
      break;
    default:
      fprintf(stderr, "Unknown Variant of the simulation!");
      exit(1);
    }

    pos.set(numSeeing4wheeled+numBlindRobots,2);
    vel.set(numSeeing4wheeled+numBlindRobots,2);
    addInspectableMatrix("pos", &pos, false, "position of the robots");
    addInspectableMatrix("vel", &vel, false, "velocity of the robots");
    distance.set(numSeeing4wheeled+numBlindRobots,1);
    if(numBalls>0){
      balls.set(numBalls,2);
      addInspectableMatrix("balls",   &balls, false, "position of the balls");
      addInspectableMatrix("distance", &distance, false, "distance to closest ball (for each robot)");
    }


    global.configs.push_back(this);

    OdeHandle wallHandle = odeHandle;
    wallHandle.substance.toSnow(.3);
    switch(arena){
    case Round:
      {
        OctaPlayground* playground = new OctaPlayground(wallHandle, osgHandle,
                                                        osg::Vec3(radiusRound, 0.2, 1), 12);
        playground->setPosition(osg::Vec3(0,0,0.1));
        playground->setGroundSubstance(Substance(0.4,0.000,40,0.5));
        global.obstacles.push_back(playground);
      }
      break;
    case Corridor:
      {
        // outer ground
        OctaPlayground* outer = new OctaPlayground(wallHandle, osgHandle,
                                                   osg::Vec3(radiusCorr+2, 0.2, 1), 12);
        outer->setTexture("");
        outer->setColor(Color(0.1,0.4,0.1,0.2));
        outer->setGroundSubstance(Substance(0.4,0.000,40,0.5));
        outer->setPosition(osg::Vec3(0,0,0.1));
        global.obstacles.push_back(outer);
        // inner walls (without ground
        OctaPlayground* inner = new OctaPlayground(wallHandle, osgHandle,
                                                   osg::Vec3(radiusCorr-2, 0.2, 1), 12, false);
        inner->setColor(Color(0.1,0.4,0.1));
        inner->setTexture("");
        inner->setPose(osg::Matrix::rotate(M_PI/12,0,0,1) * osg::Matrix::translate(0,0,0.1));
        global.obstacles.push_back(inner);
      }
      break;
    case Bone:
      {
        AbstractGround* playground = new ComplexPlayground(wallHandle, osgHandle,
                                                           "bone.fig", bonefactor, 0.03);
        playground->setGroundSubstance(Substance(0.6,0.005,40,0.5));
        playground->setPosition(osg::Vec3(0,0,0.1));
        global.obstacles.push_back(playground);
      }
      break;
    case Arena:
      {
        AbstractGround* playground = new ComplexPlayground(wallHandle, osgHandle,
                                                           "arena.fig", bonefactor, 0.04);
        //playground->setGroundSubstance(Substance(0.4,0.000,40,0.5));
        playground->setGroundSubstance(Substance(0.6,0.005,40,0.5));
        playground->setPosition(osg::Vec3(0,0,0.1));

        global.obstacles.push_back(playground);
      }
      break;
    }


    // add passive spheres as obstacles
    for (int i=0; i< numBalls; i++){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(1,1,0)), ballSize);
      // s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      switch(arena){
      case Round: s1->setPosition(osg::Vec3(i%5,-2+i/5,1));
        break;
      case Corridor:
        {
          double sep = 1; // 3.0;
          s1->setPosition(osg::Vec3(sin(i/sep)*radiusCorr,cos(i/sep)*radiusCorr,1));
          break;
        }
      case Bone: s1->setPosition(osg::Vec3((i/4)*bonefactor,(-5+(i%4)*5)*bonefactor,1));
        break;
      case Arena: s1->setPosition(osg::Vec3((-3+(i%4)*2)*bonefactor,(8+i/4)*bonefactor,1));
        break;
      }
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }



    /// FOURWHEELED
    for(int i=0; i<numSeeing4wheeled; i++){
      FourWheeledConf fwc = FourWheeled::getDefaultConf();
      fwc.twoWheelMode = true;
      fwc.useBumper    = false;
      fwc.useButton    = useBottom;
      fwc.force        = 5;
      OdeRobot* robot = new FourWheeled(odeHandle, osgHandle,
                                        fwc, "4W_" + string(VersionStrings[version]) + "_CamRobot_" +
                                        itos(teaching*1000) + "_"  + itos(i) + "_" + itos(::id));

      CameraConf camcfg = Camera::getDefaultConf();
      camcfg.width  = 256;
      camcfg.height = 128;
      camcfg.fov    = 120;
      camcfg.camSize = 0.08;
      camcfg.processors.push_back(new HSVImgProc(false,1));
      // filter only Yellow color
      camcfg.processors.push_back(new ColorFilterImgProc(true, .5,
                                  HSVImgProc::Yellow-10, HSVImgProc::Yellow+10,100));
      Camera* cam = new Camera(camcfg);
      MotionCameraSensorConf mc = MotionCameraSensor::getDefaultConf();
      mc.values = MotionCameraSensor::Position |
        MotionCameraSensor::Size | MotionCameraSensor::SizeChange;
      mc.dims   = Sensor::XY;
      mc.avg    = 1;
      mc.window = false;
      mc.border = 0.25;
      mc.sizeExponent = 0.5; // 4th root

      auto camSensor = std::make_shared<MotionCameraSensor>(mc);
      motionIndex1    = 2;
      motionIndex2    = 3;
      posIndex1       = 4;
      posIndex2       = 5;
      sizeIndex       = 6;
      sizeChangeIndex = 7;

      camSensor->setInitData(cam, odeHandle, osgHandle, osg::Matrix::rotate(-M_PI/2,0,0,1)
                             * osg::Matrix::translate(0.2,0, 0.40) );
      robot->addSensor(camSensor);
      if(seeingAreRed)
        robot->setColor(Color(1,0,0));
      else
        robot->setColor(Color(1,.7,0));
      if(arena == Bone || arena == Arena)
        robot->place(osg::Vec3(0,-i-2,0.1));
      else if(arena==Corridor)
        robot->place(osg::Vec3(sin(i/2.0+.5)*radiusCorr,cos(i/2.0+.5)*radiusCorr,0.3));
      else
        robot->place(osg::Matrix::rotate(M_PI, 0,0,1)
                       * osg::Matrix::translate(3-2*(i/5),-4+2*(i%5),0.3));


      SoxIgnoreNullConf cc = SoxIgnoreNull::getDefaultConf();
      cc.initFeedbackStrength = 1.2;
      cc.useExtendedModel     = true;
      cc.useTeaching          = true;
      cc.onlyMainParameters   = false;
      SoxIgnoreNull *sox      = new SoxIgnoreNull(cc);
      AbstractController* controller = sox;
      //      controller = new SineController();
      //      controller->setParam("period", 200);
      //      controller->setParam("phaseshift", 0.2);

      controller->setParam("gamma", teaching);
      //  controller->setParam("rootE",3);
      controller->setParam("epsC",learningrate);
      controller->setParam("epsA",learningrate);
      controller->setParam("damping",0.0001);
      controller->setParam("causeaware",0.1);
      controller->setParam("pseudo",3);
      controller->setParam("sense",2);
      controller->setParam("factorS",0.1);
      controller->setParam("factorB",0.1);


      double noise[] = {1,1};
      AbstractWiring* wiring = new SelectiveNoiseWiring(new WhiteNormalNoise(),
                                                        mkVector(noise,2));

      //    AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
      OdeAgent* agent;
      if(i==0)
        agent = new OdeAgent( global);
      else
        agent = new OdeAgent( global, PlotOption(NoPlot), 1.0);
      agent->addInspectable(this);
      agent->init(controller, robot, wiring);

      agent->addOperator(new LimitOrientationOperator(Axis(1,0,0), Axis(0,0,1), M_PI/3.0, 100));

      global.configs.push_back(controller);
      global.agents.push_back(agent);
    }



    for(int i=0; i<numBlindRobots; i++){
      // this robot has no camera
      FourWheeledConf fwc = FourWheeled::getDefaultConf();
      fwc.twoWheelMode = true;
      fwc.useBumper    = false;
      fwc.force        = 5;
      OdeRobot* robot = new FourWheeled(odeHandle, osgHandle, fwc,
                                          "BlindRobot_" + itos(i) + "_" + itos(::id));
      robot->setColor(Color(1,1,0));
      if(arena==Corridor){
        robot->place(osg::Vec3(sin(i/2.0+.5)*radiusCorr,cos(i/2.0+.5)*radiusCorr,0.3));
      }else{
        robot->place(Pos(-3,-4+2*i,0.3));
      }
      AbstractController* controller;
      if(blind_straight) {
        //      AbstractController *controller = new InvertMotorSpace(10);
        SeMoX *semox = new SeMoX();
        CrossMotorCoupling* ctrl = new CrossMotorCoupling(semox, semox, 0.0);
        std::list<int> perm;
        perm += 1;
        perm += 0;
        ctrl->setCMC(CrossMotorCoupling::getPermutationCMC(perm));
        ctrl->setParam("gamma_teach",0.003);
        controller=ctrl;
      } else {
        SoxIgnoreNullConf cc = SoxIgnoreNull::getDefaultConf();
        cc.initFeedbackStrength = 1.2;
        cc.useExtendedModel     = false;
        cc.useTeaching          = false;
        cc.onlyMainParameters   = false;
        SoxIgnoreNull *sox      = new SoxIgnoreNull(cc);
        controller = sox;

        //  controller->setParam("rootE",3);
        controller->setParam("epsC",learningrate);
        controller->setParam("epsA",learningrate);
        controller->setParam("damping",0.0001);
        controller->setParam("sense",2);
        controller->setParam("factorB",0.1);
      }

      One2OneWiring* wiring = new One2OneWiring(new WhiteNormalNoise());
      OdeAgent* agent = new OdeAgent(global);
      if(i==0 && numSeeing4wheeled==0){
        agent->addInspectable(this);
      }

      agent->init(controller, robot, wiring);
      agent->addOperator(new LimitOrientationOperator(Axis(1,0,0), Axis(0,0,1), M_PI/3.0, 100));

      global.agents.push_back(agent);
    }


  }

  virtual void addCallback(GlobalData& global, bool draw, bool pause, bool control) {
    if(control){
      FOREACH(OdeAgentList, global.agents, a){
        SoxIgnoreNull* sox = dynamic_cast<SoxIgnoreNull*>((*a)->getController());
        if(sox){
          matrix::Matrix desired = sox->getLastSensorValues();
          if(!motionIndex1 || desired.val(motionIndex1,0)==0){ // no position (or perfectly at center)
            // no teaching if no input
          }else{
            if(posIndex1){
              // position: \dot pos = -(pos) // set point is 0
              desired.val(motionIndex1,0) +=
                - (desired.val(posIndex1,0)-posSetPoint
                   + 0.1*desired.val(motionIndex1,0)) * posfactor;
              desired.val(posIndex1,0)  = posSetPoint;
            }
            if(motionIndex2 && posIndex2){
              desired.val(motionIndex2,0) +=
                - (desired.val(posIndex2,0)-posSetPoint2
                   + 0.1*desired.val(motionIndex2,0)) * posfactor;
              desired.val(posIndex2,0)  = posSetPoint2;
            }
            if(sizeIndex && sizeChangeIndex){
              // size: \dot size = -(size - 2.0) // set point is 2.0
              desired.val(sizeChangeIndex,0) +=
                - (desired.val(sizeIndex,0)-sizeSetPoint)* sizefactor;
              desired.val(sizeIndex,0)  = sizeSetPoint;
            }
          }
          sox->setSensorTeaching(desired);
        }
      }
    }
    if(!pause){

      // ball friction
      int i=0;
      FOREACH(ObstacleList, globalData.obstacles, o){
        PassiveSphere* s = dynamic_cast<PassiveSphere*>(*o);
        if(s){
          Pos avel = s->getMainPrimitive()->getAngularVel();
          s->getMainPrimitive()->applyTorque(-avel*friction);
          // save for stat
          Pos spos = s->getMainPrimitive()->getPosition();
          balls.val(i,0) = spos.x();
          balls.val(i,1) = spos.y();
          i++;
        }
      }

      //stats and
      // optionally move all balls to the robots
      int r=0;
      FOREACHC(OdeAgentList,globalData.agents, a){
        Pos rpos = (*a)->getRobot()->getPosition();
        /// some stats:
        distance.val(r,0)=100;
        pos.val(r,0) = rpos.x();
        pos.val(r,1) = rpos.y();
        Position rvel = (*a)->getRobot()->getSpeed();
        vel.val(r,0) = rvel.x;
        vel.val(r,1) = rvel.y;
        FOREACH(ObstacleList, globalData.obstacles, o){
          PassiveSphere* s = dynamic_cast<PassiveSphere*>(*o);
          if(s){
            Pos spos = s->getMainPrimitive()->getPosition();
            if(attraction > 0)
              s->getMainPrimitive()->applyForce((rpos-spos)*attraction);
            // save for stat
            if((rpos-spos).length() < distance.val(r,0)) distance.val(r,0) = (rpos-spos).length();
          }
        }
        r++;
      }

      // change camera
      if(version==Change){
        int every=changeEvery;
        if(global.sim_step == int(1*every*60/global.odeConfig.simStepSize)){
          changeCameraPose(global, osg::Matrix::rotate(M_PI/4,0,0,1));
        } else if(global.sim_step == int(2*every*60/global.odeConfig.simStepSize)){
          changeCameraPose(global, osg::Matrix::rotate(-M_PI/4,0,0,1));
          changeCameraPose(global, osg::Matrix::rotate(3*M_PI/2,0,0,1));
        } else if(global.sim_step == int(3*every*60/global.odeConfig.simStepSize)){
          changeCameraPose(global, osg::Matrix::rotate(-3*M_PI/2,0,0,1));
          changeCameraPose(global, osg::Matrix::rotate(M_PI,0,1,0)
                           *osg::Matrix::translate(0.0,0.2, 0));
        } else if(global.sim_step == int(5*every*60/global.odeConfig.simStepSize)){
          changeCameraPose(global, osg::Matrix::rotate(-M_PI,0,1,0)
                           *osg::Matrix::translate(0.0,-0.2, 0));
          changeCameraPose(global, osg::Matrix::rotate(M_PI,0,0,1));
        } else if(global.sim_step == int(6*every*60/global.odeConfig.simStepSize)){
          changeCameraPose(global, osg::Matrix::rotate(-M_PI,0,0,1));
        }
      }
    }
  }

  virtual void changeCameraPose(GlobalData& global, osg::Matrix posefactor){
    FOREACH(OdeAgentList, global.agents, a){
      auto robot = (*a)->getRobot();
      if(robot){
        auto ss = robot->getAttachedSensors();
        if(ss.size()>0){
          auto cs = dynamic_pointer_cast<CameraSensor>((*ss.begin()).first);
              if(cs){
                osg::Matrix pose = cs->getPose();
                cs->setPose(posefactor*pose);
              }
        }
      }
    }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key ) {
      case 't':
        changeCameraPose(global, osg::Matrix::rotate(M_PI/4,0,0,1));
        return true;
      case 'i':
        changeCameraPose(global, osg::Matrix::rotate(M_PI/2,0,0,1));
        return true;
      case 'I':
        changeCameraPose(global, osg::Matrix::rotate(M_PI,0,0,1));
        return true;
      case 'k':
        changeCameraPose(global, osg::Matrix::rotate(M_PI,0,1,0)*osg::Matrix::translate(0.0,0.2, 0));
        return true;
      case 'K':
        changeCameraPose(global, osg::Matrix::rotate(M_PI,0,1,0)*osg::Matrix::translate(0.0,-0.2, 0));
        return true;
      }
    }
    return false;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Sim: i","rotate cam 90deg");
    au.addKeyboardMouseBinding("Sim: I","rotate cam 180deg");
    au.addKeyboardMouseBinding("Sim: k/K","flip cam front/back (and up/down)");
  }

  virtual void end(GlobalData& globalData){
  }

  virtual void usage() const {
    printf("  *********** SPECIFIC Options ************ \n");
    printf("\t-v Variant\tVariant of the simulation V1 - V5 and\n\
\t\t1r1b (1 robot 1 ball) and change (changing cam)) (read the sourcecode)\n");
    printf("\t-t gamma_teach\tteaching strength (Def: 0.02)\n");
    printf("\t-sf factor\tfactor of guiding for size (Def: 1)\n");
    printf("\t-i  id of the simulation (to name the log files (Def: 0)\n");
  };


  osg::LightSource* makeLights(osg::StateSet* stateset)
  {
    // create a directional light (infinite distance place at 45 degrees)
    osg::Light* myLight = new osg::Light;
    myLight->setLightNum(1);
    myLight->setPosition(osg::Vec4(1.0,1.0,1.0,0.0f));
    myLight->setDirection(osg::Vec3(-1.0, -1.0, -1.0));
    myLight->setAmbient(osg::Vec4(.9f,.9f,.9f,.9f));
    myLight->setDiffuse(osg::Vec4(.7f,.7f,.7f,.7f));
    myLight->setConstantAttenuation(1.0f);

    osg::LightSource* lightS = new osg::LightSource;
    lightS->setLight(myLight);
    lightS->setLocalStateSetModes(osg::StateAttribute::ON);

    lightS->setStateSetModes(*stateset,osg::StateAttribute::ON);

    return lightS;
  }
  paramval sizeSetPoint;
  paramval posSetPoint;
  paramval posSetPoint2;

  Version version;
  double teaching;

  paramval attraction;
  paramval friction;
  paramval sizefactor;
  paramval posfactor;

  int    changeEvery;


  // stats
  matrix::Matrix pos, vel, distance;
  matrix::Matrix balls;
};


int main (int argc, char **argv)
{
  Version version   = V1;
  double teaching   = 0.1;
  double sizefactor = 1.0;
  int    changeEvery = 5;
  int index;
  index= Simulation::contains(argv, argc, "-v");
  if( index > 0 && argc > index){
    int pos = Simulation::contains((char**)VersionStrings, VMax , argv[index]);
    if(pos > 0)
      version=(Version)(pos-1);
  }
  index= Simulation::contains(argv, argc, "-t");
  if( index > 0 && argc > index){
    teaching=atof(argv[index]);
  }
  index= Simulation::contains(argv, argc, "-e");
  if( index > 0 && argc > index){
    changeEvery=atof(argv[index]);
  }
  index= Simulation::contains(argv, argc, "-sf");
  if( index > 0 && argc > index){
    sizefactor=atof(argv[index]);
  }
  index= Simulation::contains(argv, argc, "-i");
  if( index > 0 && argc > index){
    id=atoi(argv[index]);
  }
  ThisSim sim(version, teaching, sizefactor, changeEvery);
  return sim.run(argc, argv) ? 0 : 1;

}
