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
 *   Revision 1.3  2010-10-13 12:41:44  martius
 *   changed to new version of hexapod now which is in lpzrobots
 *
 *   Revision 1.2  2010/08/03 12:51:17  martius
 *   hexapod adapted Velocity servos
 *
 *   Revision 1.1  2010/07/06 08:36:30  martius
 *   hexapod of Guillaume improved and included
 *
 *   Revision 1.4  2010/07/05 16:45:55  martius
 *   hexapod tuned
 *
 *   Revision 1.3  2010/07/02 15:54:26  martius
 *   robot tuned
 *   parameters for guidance experimented
 *
 *   Revision 1.2  2010/07/02 06:39:21  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2010/07/02 06:12:55  martius
 *   initial version with hexapod made by Guillaume
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
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/feedbackwiring.h>
#include <selforg/stl_adds.h>
#include <selforg/sox.h>

#include <ode_robots/hexapod.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
const int segmnum = 13;
double teacher = 0;
bool track = false;
int k=0;
int bars=0;

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
    setCameraHomePos(Pos(-0.0114359, 6.66848, 0.922832),  Pos(178.866, -7.43884, 0));

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

    addParameter("gamma_s",&teacher);
    global.configs.push_back(this);

    for(int i=0; i< bars; i++){
      PassiveBox* b = new PassiveBox(odeHandle, osgHandle.changeColor(Color(0.,0.,0.)), 
				     osg::Vec3(1,10,0.3+i*.1),10);
      b->setPosition(osg::Vec3(10+i*7,0,0));      
      global.obstacles.push_back(b);    
    }
      
    /*******  H E X A P O D  *********/
    HexapodConf myHexapodConf = Hexapod::getDefaultConf();
    // myHexapodConf.coxaPower=0.8;
    // myHexapodConf.tebiaPower=1;
    myHexapodConf.useBigBox=true;
    myHexapodConf.tarsus=true;
    myHexapodConf.numTarsusSections = 2;
    myHexapodConf.useTarsusJoints = true;


    OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance.toRubber(20);

    
    vehicle = new Hexapod(rodeHandle, osgHandle.changeColor(Color(1,1,1)), 
			  myHexapodConf, "Hexapod_" + std::itos(teacher*10000));

    // on the top
    vehicle->place(osg::Matrix::rotate(M_PI*0,1,0,0)*osg::Matrix::translate(0,0,3));
    // normal position
    //    vehicle->place(osg::Matrix::translate(0,0,0));
    global.configs.push_back(vehicle);

//     InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();    
//     cc.cInit=1.0;
//     cc.useS=false;
//     cc.someInternalParams=true;
//     InvertMotorNStep *semox = new InvertMotorNStep(cc);  
//     semox->setParam("steps", 1);
//     semox->setParam("continuity", 0.005);
//     semox->setParam("teacher", teacher);

    SoXConf sc = SoX::getDefaultConf();
    sc.useHiddenContr=true;
    sc.useHiddenModel=false;
    sc.someInternalParams=false;
    sc.useS=false;
    SoX* sox = new SoX(sc);
    sox->setParam("logaE",0);

    SeMoXConf cc = SeMoX::getDefaultConf();    
    //cc.cInit=.95;
    cc.cInit=.99;
    cc.modelExt=false;
    cc.someInternalParams=true;
    SeMoX* semox = new SeMoX(cc);  

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
    }

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    // the feedbackwiring feeds here 75% of the motor actions as inputs and only 25% of real inputs
//     AbstractWiring* wiring = new FeedbackWiring(new ColorUniformNoise(0.1),
// 						FeedbackWiring::Motor, 0.75);
    //global.plotoptions.push_back(PlotOption(GuiLogger,Robot,5));
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    if(track) agent->setTrackOptions(TrackRobot(true,false,false, false, ""));
    global.agents.push_back(agent);
    global.configs.push_back(controller);

    this->getHUDSM()->setColor(Color(1.0,1.0,0));
    this->getHUDSM()->setFontsize(18);    
    this->getHUDSM()->addMeasure(teacher,"gamma_s",ID,1);

    showParams(global.configs);
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

  // overloaded from configurable
  virtual bool setParam(const paramkey& key, paramval val){
    bool rv = Configurable::setParam(key,val);
    if(key=="gamma_s"){
      controller->setParam("gamma_teach", teacher); 
    }
    return rv;
  }

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
  track = Simulation::contains(argv,argc,"-notrack") == 0;

  ThisSim sim;
  sim.setGroundTexture("Images/green_velour_wb.rgb");
  sim.setCaption("lpzrobots Simulator               de Chambrier, Martius 2010");
  return sim.run(argc, argv) ? 0 :  1;
}

 
 
