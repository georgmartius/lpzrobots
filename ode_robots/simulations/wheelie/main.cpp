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
 *   $Log$
 *   Revision 1.17  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.16  2010/03/17 09:33:16  martius
 *   removed memory leaks and some small bugs
 *   valgrind suppression file is updated
 *
 *   Revision 1.15  2008/09/16 14:55:20  martius
 *   different params and cmdline options
 *
 *   Revision 1.14  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.13  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.12  2007/03/16 10:58:01  martius
 *   test of substances
 *
 *   Revision 1.11  2006/09/21 22:11:28  martius
 *   make opt fixed
 *
 *   Revision 1.10  2006/09/21 11:49:55  martius
 *   low gravity ground higher
 *
 *   Revision 1.9  2006/09/21 11:44:02  martius
 *   powerratio
 *
 *   Revision 1.8  2006/09/21 10:21:59  robot8
 *   - parameters of structure changed
 *
 *   Revision 1.7  2006/09/21 09:54:46  martius
 *   params
 *
 *   Revision 1.6  2006/09/21 09:38:12  robot8
 *   *** empty log message ***
 *
 *   Revision 1.5  2006/09/21 08:16:16  martius
 *   test
 *
 *   Revision 1.4  2006/09/20 12:56:36  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2006/09/20 09:14:47  robot8
 *   - wheelie robots updated:
 *   -added biger chain  elements on every 4th position of the robots primitive, starting with the second primitive
 *   - normal wheelies Hinge Joints became less limited so they have more possibilities to move
 *
 *   Revision 1.2  2006/07/14 12:23:55  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.5  2006/06/26 20:36:17  robot5
 *   Initial value of variables modified.
 *
 *   Revision 1.1.2.4  2006/06/20 07:17:35  robot3
 *   -changed some behaviour of wheelie
 *   -added cvs log
 *
 *
 ***************************************************************************/
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include <ode_robots/sliderwheelie.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
double velScale = 4;
double powerValue = 4;
double powerRatio = 0.5;
double eps       = 0.01;

class ThisSim : public Simulation {
public:
  AbstractController *controller;
  OdeRobot* robot;
  int useReinforcement;
  double totalReinforcement;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {

    useReinforcement      = 1;
    totalReinforcement    = 0;
    bool useSliderWheelie = true;

    setCameraHomePos(Pos(-3.90752, 9.63146, 3.31768),  Pos(172.39, -10.7938, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)

    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("gravity",-3); // normally at -9.81
    //    global.odeConfig.setParam("realtimefactor",1);
    // initialization

//     Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(30, 0.2, 1));
//     playground->setPosition(osg::Vec3(0,0,0.1)); // playground positionieren und generieren
//     global.obstacles.push_back(playground);

//     for(int i=0; i<5; i++){
//       PassiveSphere* s =
//         new PassiveSphere(odeHandle,
//                           osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.2);
//       s->setPosition(Pos(i*0.5-2, i*0.5, 1.0));
//       s->setTexture("Images/dusty.rgb");
//       global.obstacles.push_back(s);
//     }

    OdeAgent *agent;
    AbstractWiring *wiring;

    if(useSliderWheelie){
      SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
      /******* S L I D E R - w H E E L I E *********/
      mySliderWheelieConf.segmNumber=12;
      mySliderWheelieConf.jointLimitIn=M_PI/3;
      mySliderWheelieConf.motorPower=powerValue; // 2
      mySliderWheelieConf.powerRatio=powerRatio; // 2;
      mySliderWheelieConf.frictionGround=0.5;
      mySliderWheelieConf.sliderLength=0.5;
      mySliderWheelieConf.segmLength=1.4;
      robot = new SliderWheelie(odeHandle, osgHandle, mySliderWheelieConf, "sliderWheelie1");
      ((OdeRobot*) robot)->place(Pos(-5,-3,3.0));
      InvertMotorNStepConf sliderinvertnconf = InvertMotorNStep::getDefaultConf();
      //      sliderinvertnconf.cInit=0.1;
      sliderinvertnconf.cInit=1;
      // sliderinvertnconf.useSD=true;
      controller = new InvertMotorNStep(sliderinvertnconf);
      //controller = new SineController();
      controller->setParam("noiseY",0);
      controller->setParam("epsC",eps); // 0.01
      controller->setParam("epsA",eps);
      controller->setParam("adaptrate",0.00);
      controller->setParam("logaE",3);
      controller->setParam("steps",1);
      controller->setParam("factorB",0);

      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = false;
      c.useFirstD = true;
      //wiring = new DerivativeWiring ( c , new ColorUniformNoise(0.1) );
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent(global);
      agent->init(controller, robot, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(robot);
    }



  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    static Position lastPos = robot->getPosition();
    if(useReinforcement==1 && control){ // speed reinforcement
      Position pos = robot->getPosition();
      Position speed = (pos - lastPos) *
        (globalData.odeConfig.controlInterval / globalData.odeConfig.simStepSize);
      lastPos=pos;
      double vel = sqrt(speed.x*speed.x + speed.y*speed.y);
      //        matrix::Matrix m(3,1, speed.toArray());
      //c->setReinforcement(tanh(sqrt(m.map(sqr).elementSum())/4 - 1));
      double reinf = tanh(vel/velScale - 1);
      totalReinforcement += reinf;
      InvertMotorNStep* c = dynamic_cast<InvertMotorNStep*>(controller);
      if(c){
        c->setReinforcement(reinf);
      }
    }
  }

  void end(GlobalData& globalData) {
    FILE* f;
    f = fopen("result","w");
    if(!f) return;
    //    fprintf(f,"#C power powerRatio eps velScale seed reinf_ps\n");
    fprintf(f,"%f %f %f %f %li %f\n",
            powerValue, powerRatio, eps, velScale,
            globalData.odeConfig.getRandomSeed(),
            totalReinforcement/globalData.time);
    fclose(f);
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        default:
          return false;
          break;
        }
    }
    return false;
  }

};


int main (int argc, char **argv)
{
  ThisSim sim;
  int index = ThisSim::contains(argv,argc,"--vals");
  if(index >0 && argc>index+3) {
    powerRatio = atof(argv[index++]);
    powerRatio = atof(argv[index++]);
    eps        = atof(argv[index++]);
    velScale   = atof(argv[index++]);
    printf("Params: P %f, PF %f, eps %f, VS %f\n",
           powerValue, powerRatio, eps, velScale);
  }

  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
