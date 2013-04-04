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
 *
 ***************************************************************************/
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>


#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/onecontrollerperchannel.h>
#include <selforg/forceboostwiring.h>

#include <selforg/sos.h>
#include <selforg/sox.h>
#include <selforg/soml.h>

#include <ode_robots/axisorientationsensor.h>

#include <ode_robots/schlangeservo2.h>
#include <ode_robots/caterpillar.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/hexapod.h>
#include <ode_robots/skeleton.h>

#include "environment.h"
#include <ode_robots/operators.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

// generates controller for splitcontrol
struct ControlGen : public ControllerGenerator {
  ControlGen(double eps)
    :eps(eps) {}
  virtual ~ControlGen(){}
  virtual AbstractController* operator()( int index) {
    AbstractController* c;
    c= new Sos(0.01);
    c->setParam("epsC",eps);
    c->setParam("epsA",.1);
    return c;
  }
  double eps;
};

// predicate that matches agents that have the same name prefix
struct agent_match_prefix : public unary_function<const OdeAgent*, bool> {
    agent_match_prefix(string nameprefix)
      : nameprefix(nameprefix) {
      len=nameprefix.length();
    }
    bool operator()(const OdeAgent* a) {
      if(!a || !a->getRobot()) return false;
      return nameprefix.compare(0,len, a->getRobot()->getName(),0,len) == 0;
    }

    string nameprefix;
    int len;
  };

class ThisSim : public Simulation {
public:

  Env env;
  double lastRobotCreation;
  bool useMultilayer;

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    setGroundTexture("Images/whiteground.jpg");
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    addParameterDef("multilayer",&useMultilayer, false, "use multilayer controller (SoML)");

    int numHexapods      = 0;
    int numSphericals    = 0;
    int numSnakes        = 0;
    int numHumanoids     = 1;
    int numSliderWheelie = 0;
    int numLongVehicle   = 0;
    int numCaterPillars   = 0;

    setCameraHomePos(Pos(-0.282677, 28.654, 8.41382),  Pos(-178.667, -18.1136, 0));
    setCameraMode(Static);
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)

    global.odeConfig.setParam("noise",0.01);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("realtimefactor",1);

    // environemnt type
    env.type=Env::Normal;
    env.create(odeHandle, osgHandle, global);

    env.numSpheres  = 0;
    env.numBoxes    = 0;
    env.numCapsules = 0;
    env.placeObstacles(odeHandle, osgHandle, global);

    // So wird das mit allen Robotern aussehen, createXXX, siehe unten
    for(int r=0; r < numHexapods ; r++) {
      createHexapod(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,1+1*r));
    }
    for(int r=0; r < numSphericals; r++) {
      createSpherical(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,0+1*r));
    }
    for(int r=0; r < numSnakes; r++) {
      createSnake(odeHandle, osgHandle, global, osg::Matrix::translate(4,4-r,0.2));
    }
    for(int r=0; r < numHumanoids; r++) {
      createHumanoid(odeHandle, osgHandle, global, osg::Matrix::translate(1,-1,1+1.5*r));
    }
    for(int r=0; r < numSliderWheelie; r++) {
      createArmband(odeHandle, osgHandle, global, osg::Matrix::translate(5+r,0,0.5));
    }
    for(int r=0; r < numLongVehicle; r++) {
      createLongVehicle(odeHandle, osgHandle, global, osg::Matrix::translate(2,-4-r,0.5));
    }
    for(int r=0; r < numCaterPillars; r++) {
      createCaterPillar(odeHandle, osgHandle, global, osg::Matrix::translate(-4-r,5+r,0.5));
    }


    lastRobotCreation=-.5;
  }

  bool removeRobot(GlobalData& global, const string& nameprefix){
    OdeAgentList::reverse_iterator i =
      find_if(global.agents.rbegin(), global.agents.rend(), agent_match_prefix(nameprefix));
    if(i!=global.agents.rend()){
      printf("Removed robot %s\n", (*i)->getRobot()->getName().c_str());
      OdeAgent* a = *i;
      global.agents.erase(i.base()-1);
      removeElement(global.configs, a);
      delete a;
      return true;
    }
    return false;
  }

  OdeAgent* createHexapod(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                          GlobalData& global, osg::Matrix pose){
    // find robot and do naming
    string name("Hexapod");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    HexapodConf myHexapodConf        = Hexapod::getDefaultConf();
    myHexapodConf.coxaPower          = 1.5;
    myHexapodConf.tebiaPower         = 0.8;
    myHexapodConf.coxaJointLimitV    = .9; // M_PI/8;  // angle range for vertical dir. of legs
    myHexapodConf.coxaJointLimitH    = 1.3; //M_PI/4;
    myHexapodConf.tebiaJointLimit    = 1.8; // M_PI/4; // +- 45 degree
    myHexapodConf.percentageBodyMass = .5;
    myHexapodConf.useBigBox          = false;
    myHexapodConf.tarsus             = true;
    myHexapodConf.numTarsusSections  = 1;
    myHexapodConf.useTarsusJoints    = true;


    OsgHandle rosgHandle=osgHandle.changeColorSet(num);
    OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance.toRubber(20);
    OdeRobot* robot = new Hexapod(rodeHandle, rosgHandle, myHexapodConf, name);
    robot->place(osg::Matrix::rotate(M_PI*0,1,0,0)*pose);

    AbstractController* controller = new Sox(1.2, false);
    controller->setParam("epsC",0.3);
    controller->setParam("epsA",0.05);
    controller->setParam("Logarithmic",1);
    AbstractWiring* wiring = new ForceBoostWiring(new ColorUniformNoise(0.1),0.05);
    //    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent*       agent  = new OdeAgent( global, PlotOption(NoPlot));
    agent->init(controller, robot, wiring);

    // add an operator to keep robot from falling over
    agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1),
                                                    M_PI*0.5, 10));

    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }

  OdeAgent* createSpherical(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                            GlobalData& global, osg::Matrix pose){

    // find robot and do naming
    string name("Spherical");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    OdeRobot* sphere =
      new Sphererobot3Masses ( odeHandle, osgHandle.changeColor("Green"),
                               conf, name, 0.2);
    sphere->place(pose);
    AbstractController* controller = new Sos();
    One2OneWiring*      wiring     = new One2OneWiring ( new WhiteUniformNoise() );
    OdeAgent*           agent      = new OdeAgent ( global, PlotOption(NoPlot));
    agent->init ( controller, sphere, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }

  OdeAgent* createSnake(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                        GlobalData& global, osg::Matrix pose, string name = "Snake"){
    // find robot and do naming
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);
    SchlangeConf snakeConf = SchlangeServo2::getDefaultConf();
    snakeConf.segmNumber   = 6+2*num;
    snakeConf.motorPower   = 5;
    snakeConf.useServoVel  = true;

    OdeRobot*    robot     = new SchlangeServo2 ( odeHandle, osgHandle, snakeConf, name);
    robot->place(pose);

    AbstractController* controller = new Sox(1.2);
    AbstractWiring*     wiring     = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent*           agent      = new OdeAgent( global, PlotOption(NoPlot) );
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }

  OdeAgent* createHumanoid(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                           GlobalData& global, osg::Matrix pose){
    // find robot and do naming
    string name("Humanoid");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    // normal servos
    //SkeletonConf conf   = Skeleton::getDefaultConf();
    // velocity servos
    SkeletonConf conf = Skeleton::getDefaultConfVelServos();

    conf.massfactor  = 1;
    conf.relLegmass  = 1;
    conf.relFeetmass = 1;
    conf.relArmmass  = 1;       //1.0;

    // conf.ankleJointLimit = 0.001; //!
    // conf.pelvisPower     = 20;

    conf.powerFactor  = 1;

    conf.useBackJoint     = true;
    conf.jointLimitFactor = 1.4;

    // conf.irSensors = true;

    OdeHandle skelHandle=odeHandle;
    OsgHandle skelOsgHandle=osgHandle.changeColorSet(num);

    // skelHandle.substance.toMetal(1);
    // skelHandle.substance.toPlastic(.5);//TEST sonst 40
    // skelHandle.substance.toRubber(5.00);//TEST sonst 40
    Skeleton* human = new Skeleton(skelHandle, skelOsgHandle, conf, name);
    // // additional sensor
    // std::list<Sensor*> sensors;
    // // sensors.push_back(new AxisOrientationSensor(AxisOrientationSensor::OnlyZAxis));
    // AddSensors2RobotAdapter* human =
    //   new AddSensors2RobotAdapter(skelHandle, osgHandle, human0, sensors);
    // global.configs.push_back(human0);

    human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)*pose);
    //   *osg::Matrix::translate(-.2 +2.9*i,0,1));
    //                 *osg::Matrix::translate(.2*i+20,2*i+20,.841/*7*/ +2*i));

    AbstractController* controller;
    if(useMultilayer){
      SoMLConf sc = SoML::getDefaultConf();
      sc.useHiddenContr        = true;
      sc.useHiddenModel        = true;
      sc.hiddenContrUnitsRatio = 1.0;
      sc.hiddenModelUnitsRatio = 1.0;
      sc.useS = false;
      controller = new SoML(sc);
    }else{
      controller = new Sox();
    }

    controller->setParam("epsC",0.3);
    controller->setParam("epsA",0.1);
    controller->setParam("Logarithmic",1);
    controller->setParam("sense",1);
    //controller->setParam("harmony",0);
    controller->setParam("causeaware",0.01);

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent*      agent  = new OdeAgent(global);
    agent->init(controller, human, wiring);
    // add an operator to keep robot up
    LiftUpOperatorConf lc = LiftUpOperator::getDefaultConf();
    lc.height = 1.5;
    lc.force  = 10;
    // lc.intervalMode = true;
    // agent->addOperator(new LiftUpOperator(lc));

    // like a bungee
    agent->addOperator(new PullToPointOperator(Pos(0,0,5),50,true,
                                               PullToPointOperator::Z,
                                               0, 0.1));

    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }

  OdeAgent* createArmband(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                          GlobalData& global, osg::Matrix pose){
    // SPLIT CONTROLLED ARMBAND

    // find robot and do naming
    string name("SliderArmband");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    SliderWheelieConf mySliderWheelieConf  = SliderWheelie::getDefaultConf();
    mySliderWheelieConf.segmNumber         = 12;
    mySliderWheelieConf.segmLength         = .4;
    mySliderWheelieConf.segmDia            = .1; // thickness and width(*8) of segments
    mySliderWheelieConf.segmLength         = .6;
    mySliderWheelieConf.segmDia            = .2; // thickness and width(*8) of segments
    mySliderWheelieConf.jointLimitIn       = M_PI/3;
    mySliderWheelieConf.frictionGround     = 0.5;
    mySliderWheelieConf.motorPower         = 5;
    mySliderWheelieConf.motorDamp          = 0.05;
    mySliderWheelieConf.sliderLength       = 0.5;

    OdeRobot* robot = new SliderWheelie(odeHandle, osgHandle.changeColor(.5,.1,.2),
                                        mySliderWheelieConf, name);
    robot->place(Pos(0,0,2.0));

    //controller = new Sos(1.0);
    AbstractController * controller =
      new OneControllerPerChannel(new ControlGen(1),"OnePerJoint");
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05));
    OdeAgent* agent = new OdeAgent(global);
    // only the first controller is exported to guilogger and Co
    agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[0]);
    // think about configureable stuff since it clutters the console
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }


  OdeAgent* createLongVehicle(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                              GlobalData& global, osg::Matrix pose){
    // find robot and do naming
    string name("LongVehicle");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
    nimm2conf.size = 1;
    nimm2conf.force = 5;
    nimm2conf.speed=20;
    //      nimm2conf.speed=15;
    nimm2conf.cigarMode=true;
    nimm2conf.cigarLength= 3.0;
    nimm2conf.singleMotor=false;
    nimm2conf.boxMode=true;
    nimm2conf.boxWidth=1.5;
    //      nimm2conf.visForce =true;
    nimm2conf.bumper=true;

    OdeHandle odeHandleR = odeHandle;
    OdeRobot* robot = new Nimm2(odeHandleR, osgHandle, nimm2conf, name);
    robot->setColor(Color(.1,.1,.8));
    robot->place(pose);
    AbstractController* controller = new Sos();
    AbstractWiring* wiring = new One2OneWiring(new WhiteNormalNoise());
    OdeAgent* agent = new OdeAgent(global, PlotOption(NoPlot));
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }

  OdeAgent* createCaterPillar(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                              GlobalData& global, osg::Matrix pose){
    // find robot and do naming
    string name("CaterPillar");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    CaterPillarConf myCaterPillarConf = DefaultCaterPillar::getDefaultConf();
    myCaterPillarConf.segmNumber=7+num;
    myCaterPillarConf.jointLimit=M_PI/3;
    myCaterPillarConf.motorPower=5;
    myCaterPillarConf.frictionJoint=0.01;
    CaterPillar* robot=
      new CaterPillar ( odeHandle, osgHandle.changeColor(Color(.1f,.5,0.1)),
                        myCaterPillarConf, name );
    robot->place(pose);

    AbstractController* controller = new Sos();
    AbstractWiring* wiring = new One2OneWiring(new WhiteNormalNoise());
    OdeAgent* agent = new OdeAgent(global, PlotOption(NoPlot));
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      // lower case (create robot) but last creation less than a second ago
      if(key >= 'a' && key <='z' && lastRobotCreation > (global.time-1)) {
        printf("You have to wait one second between robot creations.\n");
        return false;
      }

      switch ( (char) key )
        {
        case 'k': // test
          env.widthground=15;
          env.create(odeHandle, osgHandle, global,true);
        case 'b':
          createSpherical(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,2)); break;
        case 'B':
          removeRobot(global, "Spherical"); break;
        case 'x':
          createHexapod(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,2)); break;
        case 'X':
          removeRobot(global, "Hexapod"); break;
        case 's':
          createSnake(odeHandle, osgHandle, global, osg::Matrix::translate(4,4,2)); break;
        case 'S':
          removeRobot(global, "Snake"); break;
        case 'i': // put Snake in center (useful in pit-mode
          createSnake(odeHandle, osgHandle, global,
                      osg::Matrix::translate(0,0,2),"PitSnake"); break;
        case 'I':
          removeRobot(global, "PitSnake"); break;
        case 'u':
          createHumanoid(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,2)); break;
        case 'U':
          removeRobot(global, "Humanoid"); break;
        case 'a':
          createArmband(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,2)); break;
        case 'A':
          removeRobot(global, "SliderArmband"); break;
        case 'l':
          createLongVehicle(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,2)); break;
        case 'L':
          removeRobot(global, "LongVehicle"); break;
        default:
          return false;
          break;
        }
      if(key >= 'a' && key <='z') { // lower case -> created robot
        lastRobotCreation = global.time;
      }
      return true;
    }
    return false;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Sim: b/B","add/remove Spherical");
    au.addKeyboardMouseBinding("Sim: x/X","add/remove Hexapod");
    au.addKeyboardMouseBinding("Sim: s/S","add/remove Snake");
    au.addKeyboardMouseBinding("Sim: i/I","add/remove Snake in Pit");
    au.addKeyboardMouseBinding("Sim: u/U","add/remove Humanoid");
    au.addKeyboardMouseBinding("Sim: a/A","add/remove SliderArmband");
    au.addKeyboardMouseBinding("Sim: d/D","add/remove Dog");
    au.addKeyboardMouseBinding("Sim: l/L","add/remove LongVehicle");
  }



};



int main (int argc, char **argv)
{

  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}

