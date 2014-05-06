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
 *   Revision 1.2  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.1  2010/03/03 14:56:42  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2010/01/25 13:05:32  martius
 *   *** empty log message ***
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
#include <ode_robots/passivebox.h>

#include <selforg/statistictools.h>
#include <selforg/statisticmeasure.h>

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
bool useSym = true;
double teacher = 0.00;
int change = 120;  // every x second change direction
bool track = false;


class ThisSim : public Simulation {
public:
  StatisticTools stats;

  CrossMotorCoupling* controller;
  //  SeMoX* controller;
  //InvertMotorNStep* controller;
  OdeRobot* vehicle;
  double D;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    D=0;

    setCameraHomePos(Pos(-1.55424, 10.0881, 1.58559),  Pos(-170.16, -7.29053, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("noise", 0.05);
    global.odeConfig.setParam("controlinterval", 1);
    //    global.odeConfig.setParam("gravity", 0);

    for(int i=0; i< 2; i++){
      PassiveBox* b = new PassiveBox(odeHandle, osgHandle.changeColor(Color(0.,0.,0.)),
                                     osg::Vec3(1,10,0.3+i*.1),10);
      b->setPosition(osg::Vec3(30+i*7,0,0));
      global.obstacles.push_back(b);
    }


    controller=0;

    /******* S L I D E R - w H E E L I E *********/
    SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
    mySliderWheelieConf.segmNumber   = segmnum;
    mySliderWheelieConf.motorPower   = 5;
    mySliderWheelieConf.jointLimitIn = M_PI/3;
//     mySliderWheelieConf.frictionGround=0.5;
//    mySliderWheelieConf.segmLength=1.4;
    mySliderWheelieConf.sliderLength = 0;
    mySliderWheelieConf.motorType    = SliderWheelieConf::CenteredServo;
    //mySliderWheelieConf.drawCenter   = false;
    vehicle = new SliderWheelie(odeHandle, osgHandle.changeColor(Color(1,222/255.0,0)),
                                mySliderWheelieConf, "sliderWheelie_" + std::itos(teacher*10000));

    vehicle->place(Pos(0,0,2));
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
    cc.cInit=1.2;
    cc.modelExt=false;
    cc.someInternalParams=true;
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

    //    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    AbstractWiring* wiring = new FeedbackWiring(new ColorUniformNoise(0.1),
                                                FeedbackWiring::Motor, 0.75);
    //global.plotoptions.push_back(PlotOption(GuiLogger,Robot,5));
    OdeAgent* agent = new OdeAgent(global);
    agent->addCallbackable(&stats);
    agent->init(controller, vehicle, wiring);
    if(track) agent->setTrackOptions(TrackRobot(true,false,false, false,
                                                 change < 50 ? std::itos(change).c_str() : "uni", 50));
    global.agents.push_back(agent);
    global.configs.push_back(controller);

    this->getHUDSM()->setColor(Color(1.0,1.0,0));
    this->getHUDSM()->setFontsize(18);
    this->getHUDSM()->addMeasure(teacher,"Gamma_s",ID,1);
    this->getHUDSM()->addMeasure(D,"D",ID,1);

//     if(useSym){
//       int k= 0;
//       std::list<int> perm;
//       int len  = controller->getMotorNumber();
//       for(int i=0; i<len; i++){
//         perm.push_back((i+k+(len)/2)%len);
//       }
//       CMC cmc = controller->getPermutationCMC(perm);
//       controller->setCMC(cmc);
//     }



  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control && controller){
      if(useSym && globalData.time > 45){
        int k= int(globalData.time/(change))%2 == 0 ? 0 : 1; // turn around every n minutes
        teacher=0.001;
        controller->setParam("gamma_teach", teacher);
        D = 2*k-1;
        std::list<int> perm;
        int len  = controller->getMotorNumber();
        for(int i=0; i<len; i++){
           perm.push_back((i+k+(len)/2)%len);
        }
        CMC cmc = controller->getPermutationCMC(perm);
        controller->setCMC(cmc);
      }
      if(useSym && globalData.time > 90){
        teacher=0.005;
        controller->setParam("gamma_teach", teacher);
      }
    }

  };

};


int main (int argc, char **argv)
{

  ThisSim sim;
  sim.setGroundTexture("Images/red_velour_wb.rgb");
  sim.setCaption("lpzrobots Simulator               Martius et al, 2009");
  return sim.run(argc, argv) ? 0 :  1;
}



