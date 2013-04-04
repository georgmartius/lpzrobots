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
 *   Revision 1.19  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.18  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.17  2006/07/14 12:23:53  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.16.4.5  2006/05/15 13:11:30  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.16.4.4  2006/02/08 16:16:11  martius
 *   parameter tuning
 *
 *   Revision 1.16.4.3  2006/02/01 18:35:16  martius
 *   schlangeservo2
 *
 *   Revision 1.16.4.2  2006/01/05 11:41:01  fhesse
 *   moved to osg
 *                                                    *
 *                                                                         *
 ***************************************************************************/


#include <selforg/noisegenerator.h>

#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>

#include <ode_robots/schlangeservo.h>
#include <ode_robots/schlangeservo2.h>
#include <ode_robots/schlangeforce.h>
#include <ode_robots/schlangevelocity.h>


using namespace lpzrobots;

class ThisSim : public Simulation {
public:


  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    setCameraHomePos(Pos(-10.7854, -7.41751, 5.92078),  Pos(-50.6311, -5.00218, 0));

    // conf for schlange1 and schlange2
    SchlangeConf conf = Schlange::getDefaultConf();
    conf.motorPower   = 0.1;
    conf.jointLimit   = conf.jointLimit*1.5;
    //conf.segmDia      = 0.2;
    conf.segmNumber   = 7;


    // SchlangeServo
    SchlangeServo2* schlange1 =
      new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
                           conf, "Servo");
    ((OdeRobot*)schlange1)->place(Pos(2,2,5));
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.cInit=2;
    AbstractController *controller1 = new InvertMotorNStep(cc);
    controller1->setParam("adaptrate",0);
    controller1->setParam("epsC",0.001);
    controller1->setParam("epsA",0.001);
    controller1->setParam("rootE",1);
    controller1->setParam("s4avg",10);
    controller1->setParam("steps",2);
    //    AbstractController *controller1 = new SineController();
    AbstractWiring* wiring1 = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent* agent1 = new OdeAgent(global);
    agent1->init(controller1, schlange1, wiring1);
    global.agents.push_back(agent1);
    global.configs.push_back(controller1);
    global.configs.push_back(schlange1);


    //SchlangeForce
//     conf.motorPower   = 0.2;
//     conf.jointLimit   = conf.jointLimit*1.5;
//     //conf.segmDia      = 0.2;
//     conf.segmNumber   = 5;
//      SchlangeForce* schlange2 =
//        new SchlangeForce ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
//                            conf, "Force");
//      ((OdeRobot*)schlange2)->place(Pos(3,3,5));
//      InvertMotorNStepConf cc2 = InvertMotorNStep::getDefaultConf();
//      cc2.cInit=0.01;
//      AbstractController *controller2 = new InvertMotorNStep(cc2);
//      //     AbstractController *controller2 = new SineController();
//      AbstractWiring* wiring2 = new One2OneWiring(new ColorUniformNoise(0.1));
//      OdeAgent* agent2 = new OdeAgent(global);
//      agent2->init(controller2, schlange2, wiring2);
//      global.agents.push_back(agent2);
//      global.configs.push_back(controller2);
//      global.configs.push_back(schlange2);
//      controller2->setParam("adaptrate",0);
//      controller2->setParam("epsC",0.005);
//      controller2->setParam("epsA",0.001);
//      controller2->setParam("rootE",1);
//      controller2->setParam("s4avg",10);
//      controller2->setParam("steps",2);

//      controller2->setParam("phaseshift", 0.3);

    //SchlangeVelocity
//      SchlangeConf conf3 = SchlangeVelocity::getDefaultConf();
//      conf3.jointLimit=conf3.jointLimit*3;
//      conf3.segmDia    = 0.2;
//      conf3.segmNumber = 2;
//      SchlangeVelocity* schlange3 =
//        new SchlangeVelocity ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
//                               conf3, "Velocity");
//      ((OdeRobot*)schlange3)->place(Pos(2,2,0));
//      AbstractController *controller3 = new SineController();
//      AbstractWiring* wiring3 = new One2OneWiring(new ColorUniformNoise(0.1));
//      OdeAgent* agent3 = new OdeAgent(global);
//      agent3->init(controller3, schlange3, wiring3);
//      global.agents.push_back(agent3);
//      global.configs.push_back(controller3);
//      global.configs.push_back(schlange3);


     global.odeConfig.setParam("controlinterval",1);
     //     global.odeConfig.setParam("gravity", -9.81);
     global.odeConfig.setParam("gravity", 0);
     global.odeConfig.setParam("noise", 0.05);


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
  return sim.run(argc, argv) ? 0 : 1;
}




