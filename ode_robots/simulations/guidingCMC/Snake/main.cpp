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
 *   Revision 1.6  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.5  2010/09/30 17:07:08  martius
 *   tests and vision experiments improved
 *
 *   Revision 1.4  2010/01/26 09:58:55  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2009/11/23 13:49:20  martius
 *   some testing
 *
 *   Revision 1.2  2009/08/07 14:39:52  martius
 *   guidance of SO with Cross Motor Couplings
 *
 *   Revision 1.1  2009/08/05 23:25:23  martius
 *   new simulations of Georg
 *
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/complexplayground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/semox.h>
#include <selforg/crossmotorcoupling.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include <selforg/feedbackwiring.h>
#include <selforg/stl_adds.h>

#include <ode_robots/schlangeservo.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

const int segmNum=14;
bool useSym = false;
double teacher = 0;
int change = 5;  // every x minutes change direction
bool track = true;
int k=8;


class ThisSim : public Simulation {
public:
  Joint* fixator;

  CrossMotorCoupling* controller;
  OdeRobot* vehicle;
  matrix::Matrix teaching;

  ThisSim()
  {
  }

  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    setCameraHomePos(Pos(-19.7951, -12.3665, 16.4319),  Pos(-51.7826, -26.772, 0));

    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("gravity", -9);
    global.odeConfig.setParam("controlinterval",2);
    //    global.odeConfig.setParam("realtimefactor",4);

    //****************/
    SchlangeConf conf = Schlange::getDefaultConf();
    //conf.motorPower=3;
    conf.frictionJoint=0.05;
    conf.frictionRatio=0.01;

    //     conf.motorPower=5;
    conf.motorPower=5;
    //    conf.frictionJoint=0.01;
    //    conf.segmNumber=16;
    conf.segmNumber=segmNum;
    conf.useServoVel=false;
    //     conf.jointLimit=conf.jointLimit*3;
    // conf.sensorFactor=5;     unused

    OdeHandle snakeHandle(odeHandle);
    //    snakeHandle.substance.toPlastic(0.1);
    snakeHandle.substance.toPlastic(.5);
    vehicle = new SchlangeServo ( snakeHandle, osgHandle.changeColor(Color(0.9, 0.85, 0.05)),
                                  conf, "Schlange1D_" + std::itos(teacher*10000));

    vehicle->place(Pos(0,0,2));

//     Primitive* head = vehicle->getMainPrimitive();
//     fixator = new BallJoint(head, global.environment, head->getPosition());
//     fixator->init(odeHandle, osgHandle);


    SeMoXConf cc = SeMoX::getDefaultConf();
    cc.someInternalParams=true;
    cc.modelExt=true;
    cc.cInit=1.0;
    SeMoX* semox = new SeMoX(cc);
    //semox->setParam("adaptrate", 0.0001);
    //    semox->setParam("nomupdate", 0.0001);
    semox->setParam("epsC", 0.02);
    semox->setParam("epsA", 0.05);
    semox->setParam("rootE", 0);
    semox->setParam("gamma_cont", 0.001);
    semox->setParam("gamma_teach", teacher);
    semox->setParam("discountS", 0.005);
    semox->setParam("s4avg", 1);
    semox->setParam("s4del", 2); // ?

    controller = new CrossMotorCoupling( semox, semox);
    // controller = new SineController();

    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    // DerivativeWiringConf wc = DerivativeWiring::getDefaultConf();
//     wc.useId=true;
//     wc.useSecondD=true;
//     wc.eps=0.25;
//     wc.derivativeScale=5;
//     AbstractWiring* wiring = new DerivativeWiring(wc, new WhiteUniformNoise());

    //    AbstractWiring* wiring = new FeedbackWiring(new ColorUniformNoise(0.1),
    //                                                  FeedbackWiring::Motor, 0.75);

    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);

    if(track) agent->setTrackOptions(TrackRobot(true,false,false, false,
                                                 change < 50 ? std::itos(change).c_str() : "uni", 50));
    global.agents.push_back(agent);
    global.configs.push_back(controller);
    global.configs.push_back(vehicle);

    if(useSym){
      std::list<int> perm;
      int len  = controller->getMotorNumber();
      for(int i=0; i<len; i++){
        perm.push_back((i+k)%len);
      }
      CMC cmc = controller->getPermutationCMC(perm);
      controller->setCMC(cmc);
    }


  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
//     if(control && controller) {
//       if(useSym){
//         int k= int(globalData.time/(change*60))%4+1; //  == 0 ? 0 : 1; // turn around every 10 minutes
//         std::list<int> perm;
//         int len  = controller->getMotorNumber();
//         for(int i=0; i<len; i++){
//            perm.push_back((i+k)%len);
//         }
//         CMC cmc = controller->getPermutationCMC(perm);
//         controller->setCMC(cmc);

//         matrix::Matrix m = controller->getLastMotorValues();
//         teaching = m;
//         int len = m.getM();
//         for(int i=0; i<len; i++){
//           double l = m.val((i+k+(len)/2)%len,0);
//           if(fabs(l)>0.4){
//             teaching.val(i,0) = l;
//           }
//         }
//         controller->setMotorTeaching(teaching);
//       }
//    }
  }

  //Funktion die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;
    bool handled = false;
    switch ( key )
      {
      case 'x':
        if(fixator) delete (fixator);
        fixator=0 ;
        handled = true;
        break;
      case 'i':
        k++;
        std::cout << "connection: " << k << std::endl;
        handled = true;
        break;
      case 'k':
        k = max(0,k-1);
        std::cout << "connection: " << k << std::endl;
        handled = true;
        break;
      }
    fflush(stdout);

    std::list<int> perm;
    int len = controller->getMotorNumber();
    for(int i=0; i<len; i++){
      perm.push_back((i+k)%len);
    }
    CMC cmc = controller->getPermutationCMC(perm);
    controller->setCMC(cmc);

    return handled;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Simulation: x","delete fixation");
  }

};


int main (int argc, char **argv)
{
  int index = Simulation::contains(argv,argc,"-sym");
  if(index >0 && argc>index){
    teacher=atof(argv[index]);
    useSym = 1;
  }
  index = Simulation::contains(argv,argc,"-change");
  if(index >0 && argc>index){
    change=atoi(argv[index]);
  }
  track = Simulation::contains(argv,argc,"-notrack") == 0;

  ThisSim sim;
  sim.setGroundTexture("Images/red_velour_wb.rgb");
  sim.setCaption("lpzrobots Simulator               Martius et al, 2009");

  return sim.run(argc, argv) ? 0 :  1;
}



