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
 *   Revision 1.11  2011-11-02 09:11:49  martius
 *   capcolbug patch added
 *   use proper include pathes for user installation and library compilation
 *
 *   Revision 1.10  2011/10/27 12:11:18  der
 *   sox added
 *   sox added
 *
 *   Revision 1.9  2011/10/12 13:41:04  der
 *   *** empty log message ***
 *
 *   Revision 1.8  2011/05/30 21:57:16  martius
 *   store and restore from console improved
 *   console width automatically adapted
 *
 *   Revision 1.7  2011/05/30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.6  2011/01/31 11:31:10  martius
 *   renamed sox to soml
 *
 *   Revision 1.5  2010/11/05 13:54:05  martius
 *   store and restore for robots implemented
 *
 *   Revision 1.4  2010/10/20 13:18:38  martius
 *   parameters changed,
 *   motor babbling added
 *
 *   Revision 1.3  2010/10/13 12:41:44  martius
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
#include <ode_robots/octaplayground.h>
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
#include <selforg/soml.h>
#include <selforg/derinf.h>


#include "sox.h"
#include <ode_robots/hexapod.h>
//#include "hexapod.h"

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
    global.odeConfig.setParam("controlinterval", 2);
    global.odeConfig.setParam("cameraspeed", 250);
    global.odeConfig.setParam("gravity", -4);
    setParam("UseQMPThread", false);
    //setupPlaygrounds(odeHandle, osgHandle, global,  Normal);
    // use Playground as boundary:
//    playground = new Playground(odeHandle, osgHandle, osg::Vec3(8, 0.2, 1), 1);
//     // playground->setColor(Color(0,0,0,0.8)); 
//     playground->setGroundColor(Color(2,2,2,1)); 
//     playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
//     global.obstacles.push_back(playground); 
    int diam = 80;
    OctaPlayground* playground3 = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter*/4*diam, 5*diam,/*Height*/ .2), 12,false);
    //  playground3->setColor(Color(.0,0.2,1.0,1));
    playground3->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground3);
   
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

    HexapodConf myHexapodConf = Hexapod::getDefaultConf();
    myHexapodConf.coxaPower= .8;//1.0;//1.3;//2.0;
    myHexapodConf.tebiaPower= .5;//1.2;//1.6;
    myHexapodConf.coxaJointLimitV =.6;// M_PI/8;  ///< angle range for vertical direction of legs
    myHexapodConf.coxaJointLimitH = 1;//M_PI/4;
    myHexapodConf.tebiaJointLimit = 1.5;// M_PI/4; // +- 45 degree
    myHexapodConf.percentageBodyMass=.5;
    // if ( ii =0 )
    myHexapodConf.useBigBox=true;
    myHexapodConf.tarsus=true;
    myHexapodConf.numTarsusSections = 1;
    myHexapodConf.useTarsusJoints = true;

    OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance.toRubber(20);

    
    vehicle = new Hexapod(rodeHandle, osgHandle.changeColor(Color(1,1,1)), 
			  myHexapodConf, "Hexapod_" + std::itos(teacher*10000));

    // on the top
    vehicle->place(osg::Matrix::rotate(M_PI*0,1,0,0)*osg::Matrix::translate(0,0,1+ 2*ii));
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

    // SoMLConf sc = SoML::getDefaultConf();
    // sc.useHiddenContr=true;
    // sc.useHiddenModel=true;
    // sc.someInternalParams=false;
    // sc.useS=false;
    // SoML* sox = new SoML(sc);
    // sox->setParam("epsC",0.105);
    // sox->setParam("epsA",0.05);

    Sox* sox = new Sox(1.2, false);
    sox->setParam("epsC",0.105);
    sox->setParam("epsA",0.05);
    sox->setParam("Logarithmic",1);
    //  sox->setParam("osceps",10);


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
      // controller = derinf; 
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

  ThisSim sim;
  sim.setGroundTexture("Images/green_velour_wb.rgb");
  sim.setCaption("lpzrobots Simulator Homeokinesis -  Multilayer NN");
  return sim.run(argc, argv) ? 0 :  1;
}

 
 
