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
 *   Revision 1.8  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.7  2011/05/30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.6  2010/07/05 16:47:34  martius
 *   hashset transition to tr1
 *   new pid function for velocity servos, which work now fine
 *
 *   Revision 1.5  2010/01/27 10:20:47  martius
 *   added blinking. This simulation was used to make the movies,
 *    armband_{weak|medium|strong}_guidance and the ...change_direction
 *
 *   Revision 1.4  2010/01/26 10:53:53  martius
 *   k can be changed on console (fixed)
 *   cmd command for teaching is called -guided
 *
 *   Revision 1.3  2010/01/26 10:50:17  martius
 *   k can be changed on the console
 *
 *   Revision 1.2  2010/01/26 10:26:45  martius
 *   added bars (optional)
 *   cmc in extra function
 *
 *   Revision 1.1  2009/08/07 14:39:52  martius
 *   guidance of SO with Cross Motor Couplings
 *
 *   Revision 1.7  2009/03/30 18:51:43  martius
 *   corrections and stuff
 *
 *   Revision 1.6  2009/03/28 18:08:12  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2009/03/27 22:31:00  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2009/03/27 18:38:33  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2009/03/27 13:37:09  martius
 *   parameters
 *
 *   Revision 1.2  2009/03/26 20:24:56  martius
 *   changed color and ground
 *
 *   Revision 1.1  2009/03/26 18:35:12  martius
 *   working version
 *
 *   Revision 1.2  2009/03/19 18:51:40  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2009/03/18 17:46:14  martius
 *   Teaching stuff
 *
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/complexplayground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

#include <selforg/invertmotornstep.h>
#include <selforg/semox.h>
#include <selforg/crossmotorcoupling.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/feedbackwiring.h>
#include <selforg/stl_adds.h>

#include <ode_robots/sliderwheelie.h>
// #include <ode_robots/wheelie.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
const int segmnum = 13;
bool useSym = false;
double teacher = 0;
int change = 0;  // do not change direction by default
bool track = false;
int k=0;
int bars=0;
int stairs=0;

// wiring to use motor values as sensors as well
class MotorAsSensors : public One2OneWiring {
public:
  MotorAsSensors(NoiseGenerator* noise, int plotMode=Controller)
    : One2OneWiring(noise,plotMode, 0) {
  }
  virtual ~MotorAsSensors() {}
protected:
  virtual bool initIntern(){
    One2OneWiring::initIntern();
    csensornumber = rsensornumber + rmotornumber;
    noisenumber = csensornumber;
    return true;
  }

  virtual bool wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                 sensor* csensors, int csensornumber,
                                 double noise){
    One2OneWiring::wireSensorsIntern(rsensors, rsensornumber, csensors, csensornumber, noise);
    for(int i=0; i< rmotornumber; i++){
      // copy the motor action from previous timestep
      csensors[i + rsensornumber] = mCmotors.val(i,0) + noisevals[i];
    }
//     for(int i=0; i< rsensornumber; i++){
//       // copy the motor action from previous timestep
//       csensors[i  + rmotornumber] = rsensors[i]  + noisevals[i];
//     }
    return true;
  }

};


class ThisSim : public Simulation {
public:
  StatisticTools stats;
  double k_double;
  int blink;

  CrossMotorCoupling* controller;
  //  SeMoX* controller;
  //InvertMotorNStep* controller;
  OdeRobot* vehicle;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-0.0114359, 6.66848, 0.922832),  Pos(178.866, -7.43884, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("noise", 0.05);
    global.odeConfig.setParam("controlinterval", 1);
    global.odeConfig.setParam("cameraspeed", 250);

    // use Playground as boundary:
//    playground = new Playground(odeHandle, osgHandle, osg::Vec3(8, 0.2, 1), 1);
//     // playground->setColor(Color(0,0,0,0.8));
//     playground->setGroundColor(Color(2,2,2,1));
//     playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
//     global.obstacles.push_back(playground);
    controller=0;

    addParameter("k",&k);
    addParameter("gamma_s",&teacher);
    global.configs.push_back(this);

    for(int i=0; i< bars; i++){
      PassiveBox* b = new PassiveBox(odeHandle, osgHandle.changeColor(Color(0.,0.,0.)),
                                     osg::Vec3(1,10,0.3+i*.1),0.0);
      b->setPosition(osg::Vec3(10+i*7,0,0));
      global.obstacles.push_back(b);
    }

    double h = 0.;
    RandGen rgen;
    rgen.init(2);
    for(int i=0; i<stairs; i++){
      do{
        h+=(rgen.rand()-.5)*0.6; // values between (-.25.25)
      }while(h<0);
      PassiveBox* b = new PassiveBox(odeHandle, osgHandle.changeColor(Color(0.3,0.3,0.3)),
                                     osg::Vec3(1,10,h),0.0);
      b->setPosition(osg::Vec3(5+i,0,0));
      global.obstacles.push_back(b);
    }

    /******* S L I D E R - w H E E L I E *********/
    SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
    mySliderWheelieConf.segmNumber   = segmnum;
    mySliderWheelieConf.motorPower   = 5;
    mySliderWheelieConf.jointLimitIn = M_PI/3;
//     mySliderWheelieConf.frictionGround=0.5;
//    mySliderWheelieConf.segmLength=1.4;
    mySliderWheelieConf.sliderLength = 0;
    mySliderWheelieConf.motorType    = SliderWheelieConf::CenteredServo;
    mySliderWheelieConf.showCenter   = false;
    vehicle = new SliderWheelie(odeHandle, osgHandle.changeColor(Color(1,222/255.0,0)),
                                mySliderWheelieConf, "sliderWheelie_" + std::itos(teacher*10000));

    vehicle->place(Pos(0,0,.1));
    global.configs.push_back(vehicle);

//     InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
//     cc.cInit=1.0;
//     cc.useS=false;
//     cc.someInternalParams=true;
//     InvertMotorNStep *semox = new InvertMotorNStep(cc);
//     semox->setParam("steps", 1);
//     semox->setParam("continuity", 0.005);
//     semox->setParam("teacher", teacher);

    SeMoXConf cc = SeMoX::getDefaultConf();
    //cc.cInit=.95;
    cc.cInit=.5;
    cc.modelExt=false;
    //    cc.someInternalParams=true;
    cc.someInternalParams=false;
    SeMoX* semox = new SeMoX(cc);

//     AbstractController* controller = new SineController(~0, SineController::Sine);   // local variable!
// //     // motorpower 20
//     controller->setParam("period", 300);
//     controller->setParam("phaseshift", 0.3);

    if(useSym){
      semox->setParam("epsC", 0.1);
      semox->setParam("epsA", 0.1);
    }else{
      semox->setParam("epsC", 0.1);
      semox->setParam("epsA", 0.1);
    }
    semox->setParam("rootE", 3);
    semox->setParam("s4avg", 1);
    semox->setParam("gamma_cont", 0.005);

    semox->setParam("gamma_teach", teacher);

    //controller=semox;
    controller = new CrossMotorCoupling( semox, semox, 0.4);

    //        AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    //AbstractWiring* wiring = new MotorAsSensors(new ColorUniformNoise(0.1));
    AbstractWiring* wiring = new FeedbackWiring(new ColorUniformNoise(0.1),
                                                 FeedbackWiring::Motor, 0.75);
    //global.plotoptions.push_back(PlotOption(GuiLogger,Robot,5));
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    if(track) agent->setTrackOptions(TrackRobot(true,false,false, false,
                                                 change != 0 ? std::itos(change).c_str() : "uni", 50));
    global.agents.push_back(agent);
    global.configs.push_back(controller);

    this->getHUDSM()->setColor(Color(1.0,1.0,0));
    this->getHUDSM()->setFontsize(18);
    this->getHUDSM()->addMeasure(teacher,"gamma_s",ID,1);
    this->getHUDSM()->addMeasure(k_double,"k",ID,1);

    setCMC(k);
    blink=0;


  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control && controller){
      if(useSym && change>0){
        int newk= int(globalData.time/(change*60))%2 == 0 ? 0 : 1; // turn around every n minutes
        if(k!=newk) blink=400;
        setCMC(newk);
      }
      // let the display blink
      if(blink>0){
        blink--;
        HUDStatisticsManager::WindowStatistic* ws = this->getHUDSM()->getMeasureWS("k");
        if(ws){
          ws->getText()->setColor( Color(1.0,1.0,
              int(globalData.time*2)%2 == 0 || blink == 0 ? 0.0 : 1.0));
        }
      }

    }

  }

  virtual void setCMC(int _k){
    k=_k;
    k_double=k;
    std::list<int> perm;
    int len  = controller->getMotorNumber();
    for(int i=0; i<len; i++){
      perm.push_back((i+k+(len)/2)%len);
    }
    CMC cmc = controller->getPermutationCMC(perm);
    controller->setCMC(cmc);
  }

  // overloaded from configurable
  virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren){
    bool rv = Configurable::setParam(key,val);
    if(key=="k"){
      if(change>0){
        fprintf(stderr, "Automatic change mode is on. Manual changes of k will be overwritten!\n");
      }
      setCMC(k);
      blink=400;
    }
    if(key=="gamma_s"){
      controller->setParam("gamma_teach", teacher);
    }
    return rv;
  }

  virtual void usage() const {
    printf("Additional Usage: ... [-guide Teach] [-k K] [-change Min] [-bars NUM] [-stairs NUM] [-notrack]\n");
    printf("\t-guide Teach\tguidance factor (def: 0)\n");
    printf("\t-k K\tconnection setup (def: 0)\n");
    printf("\t-change Min\tchange every Min minutes the connection setup (def: 5)\n");
    printf("\t-bars NUM\tNumber of bars with increasing size (def: 0)n");
    printf("\t-stairs NUM\tNumber of bars in the random staircase (def: 0)n");
    printf("\t-notrack\t do not track the robot\n");
  };

};


int main (int argc, char **argv)
{
  int index = Simulation::contains(argv,argc,"-guide");
  if(index >0 && argc>index){
    teacher=atof(argv[index]);
    useSym = 1;
  }
  index = Simulation::contains(argv,argc,"-k");
  if(index >0 && argc>index){
    k=atoi(argv[index]);
  }
  index = Simulation::contains(argv,argc,"-change");
  if(index >0 && argc>index){
    change=atoi(argv[index]);
  }
  index = Simulation::contains(argv,argc,"-bars");
  if(index >0 && argc>index){
    bars=atoi(argv[index]);
  }
  index = Simulation::contains(argv,argc,"-stairs");
  if(index >0 && argc>index){
    stairs=atoi(argv[index]);
  }
  track = Simulation::contains(argv,argc,"-notrack") == 0;

  ThisSim sim;
  sim.setGroundTexture("Images/red_velour_wb.rgb");
  sim.setCaption("lpzrobots Simulator               Martius et al, 2009");
  return sim.run(argc, argv) ? 0 :  1;
}



