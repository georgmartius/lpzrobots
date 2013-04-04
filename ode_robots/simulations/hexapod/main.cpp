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
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/complexplayground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/operators.h>
#include <ode_robots/boxpile.h>

#include <selforg/invertmotornstep.h>
#include <selforg/semox.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/feedbackwiring.h>
#include <selforg/stl_adds.h>
#include <selforg/soml.h>

#include <selforg/derinf.h>

#include "sox.h"
//#include <selforg/sox.h>
#include <ode_robots/hexapod.h>
//#include "hexapod.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

const int segmnum   = 13;
double    teacher   = 0;
bool      track     = false;
bool      tracksegm = false;
int       k         = 0;
int       bars      = 0;

//bool useSineController = true;
bool useSineController = false;

class ThisSim : public Simulation {
public:
  StatisticTools stats;

  AbstractController* controller;
  //  SeMoX* controller;
  //InvertMotorNStep* controller;
  OdeRobot* vehicle;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-6.32561, 5.12705, 3.17278),  Pos(-130.771, -17.7744, 0));


    global.odeConfig.setParam("noise", 0.05);
    global.odeConfig.setParam("controlinterval", 2);
    global.odeConfig.setParam("cameraspeed", 250);
    global.odeConfig.setParam("gravity", -6);
    setParam("UseQMPThread", false);

    // use Playground as boundary:
    AbstractGround* playground =
      new Playground(odeHandle, osgHandle, osg::Vec3(8, 0.2, 1), 1);
    //     // playground->setColor(Color(0,0,0,0.8));
    playground->setGroundColor(Color(2,2,2,1));
    playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    Boxpile* boxpile = new Boxpile(odeHandle, osgHandle);
    boxpile->setColor("wall");
    boxpile->setPose(ROTM(M_PI/5.0,0,0,1)*TRANSM(0, 0,0.2));
    global.obstacles.push_back(boxpile);


    //     global.obstacles.push_back(playground);
    // double diam = .90;
    // OctaPlayground* playground3 = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter*/4.0*diam, 5,/*Height*/ .3), 12,
    //                                                  false);
    // //  playground3->setColor(Color(.0,0.2,1.0,1));
    // playground3->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    // global.obstacles.push_back(playground3);

    controller=0;

    //    addParameter("gamma_s",&teacher);
    global.configs.push_back(this);

    for(int i=0; i< bars; i++){
      PassiveBox* b = new PassiveBox(odeHandle, osgHandle.changeColor(Color(0.,0.,0.)),
                                     osg::Vec3(1,10,0.3+i*.1),10);
      b->setPosition(osg::Vec3(10+i*7,0,0));
      global.obstacles.push_back(b);
    }

    /*******  H E X A P O D  *********/
    int numhexapods = 1;
    for ( int ii = 0; ii< numhexapods; ii++){

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
    //    myHexapodConf.numTarsusSections = 2;

    OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance.toRubber(20);


    vehicle = new Hexapod(rodeHandle, osgHandle.changeColor("Green"),
                          myHexapodConf, "Hexapod_" + std::itos(teacher*10000));

    // on the top
    vehicle->place(osg::Matrix::rotate(M_PI*1,1,0,0)*osg::Matrix::translate(0,0,1.5+ 2*ii));
    // normal position
    //    vehicle->place(osg::Matrix::translate(0,0,0));

//     InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
//     cc.cInit=1.0;
//     cc.useS=false;
    //    cc.someInternalParams=true;
//     InvertMotorNStep *semox = new InvertMotorNStep(cc);
//     semox->setParam("steps", 1);
//     semox->setParam("continuity", 0.005);
//     semox->setParam("teacher", teacher);

    SoMLConf sc = SoML::getDefaultConf();
    sc.useHiddenContr=true;
    sc.useHiddenModel=false;
    sc.someInternalParams=false;
    sc.useS=false;
    SoML* soml = new SoML(sc);
    soml->setParam("epsC",0.105);
    soml->setParam("epsA",0.05);

    Sox* sox = new Sox(1.2, false);
    sox->setParam("epsC",0.105);
    sox->setParam("epsA",0.05);
    sox->setParam("Logarithmic",1);


    SeMoXConf cc = SeMoX::getDefaultConf();
    //cc.cInit=.95;
    cc.cInit=.99;
    cc.modelExt=false;
    cc.someInternalParams=true;
    SeMoX* semox = new SeMoX(cc);

    DerInfConf dc = DerInf::getDefaultConf();
    dc.cInit=.599;
    dc.someInternalParams=false;
    AbstractController* derinf = new DerInf(dc);
    derinf->setParam("epsC",0.1);
    derinf->setParam("epsA",0.05);

    AbstractController* sine = 0;
    if(useSineController){
      // sine = new SineController(~0, SineController::Sine);
      sine = new SineController(~0, SineController::Impulse);
      // //     // //     // motorpower 20
      sine->setParam("period", 30);
      sine->setParam("phaseshift", 0.5);
      sine->setParam("amplitude", 0.5);
    }

    semox->setParam("epsC", 0.1);
    semox->setParam("epsA", 0.1);
    semox->setParam("rootE", 3);
    semox->setParam("s4avg", 1);
    semox->setParam("gamma_cont", 0.005);

    semox->setParam("gamma_teach", teacher);


    if(useSineController){
      controller = sine;
    }else{
      //      controller = semox;
      controller = sox;
     //  controller = soml;
      // controller = derinf;
    }

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    // the feedbackwiring feeds here 75% of the motor actions as inputs and only 25% of real inputs
//     AbstractWiring* wiring = new FeedbackWiring(new ColorUniformNoise(0.1),
//                                                 FeedbackWiring::Motor, 0.75);
    //global.plotoptions.push_back(PlotOption(GuiLogger,Robot,5));
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    // add an operator to keep robot from falling over
    agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1), M_PI*0.5, 30));
    if(track){
      TrackRobotConf c = TrackRobot::getDefaultConf();
      c.displayTrace = true;
      c.scene        = "";
      c.interval     = 1;
      c.trackSpeed   = false;
      c.displayTraceThickness = 0.01;
      agent->setTrackOptions(TrackRobot(c));
    }
    if(tracksegm){
      TrackRobotConf c   = TrackRobot::getDefaultConf();
      Color      col = osgHandle.getColor("joint");
      c.displayTrace = true;
      c.scene        = "segm";
      c.interval     = 1;
      c.displayTraceThickness = 0.02;
      col.alpha()    = 0.5;
      agent->addTracking(5, TrackRobot(c), col);
      agent->addTracking(8, TrackRobot(c), col);
    }

    global.agents.push_back(agent);
    global.configs.push_back(agent);
    //agent->startMotorBabblingMode(5000);

    // this->getHUDSM()->setColor(Color(1.0,1.0,0));
    // this->getHUDSM()->setFontsize(18);
    // this->getHUDSM()->addMeasure(teacher,"gamma_s",ID,1);

  }
  }
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control && controller){
      if(teacher){
        Teachable* contr = dynamic_cast<Teachable*>(controller);
        if(contr){
          // calculate teaching signal
          matrix::Matrix teaching = contr->getLastMotorValues(); // initialize with last motor values (essentially no teaching)
          // TODO: change teaching matrix here
          contr->setMotorTeaching(teaching);
        }
      }
    }

  }

  // // overloaded from configurable
  // virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren){
  //   bool rv = Configurable::setParam(key,val);
  //   if(key=="gamma_s"){
  //     controller->setParam("gamma_teach", teacher);
  //   }
  //   return rv;
  // }

};


int main (int argc, char **argv)
{
  int index = Simulation::contains(argv,argc,"-guide");
  if(index >0 && argc>index){
    teacher=atof(argv[index]);
  }
  index = Simulation::contains(argv,argc,"-bars");
  if(index >0 && argc>index){
    bars=atoi(argv[index]);
  }
  track = Simulation::contains(argv,argc,"-track") != 0;
  tracksegm = Simulation::contains(argv,argc,"-tracksegm") != 0;

  ThisSim sim;
  sim.setGroundTexture("Images/green_velour_wb.rgb");
  sim.setCaption("lpzrobots Simulator Homeokinesis -  One-Layer Controller");
  return sim.run(argc, argv) ? 0 :  1;
}



