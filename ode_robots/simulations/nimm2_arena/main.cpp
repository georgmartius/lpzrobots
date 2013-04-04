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
 *   Revision 1.11  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.10  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.9  2007/12/11 14:11:12  martius
 *   chain of robots
 *
 *   Revision 1.8  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.7  2006/08/04 16:25:14  martius
 *   bugfixing
 *
 *   Revision 1.6  2006/07/14 12:23:50  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.5.4.5  2006/06/25 21:57:41  martius
 *   robot names with numbers
 *
 *   Revision 1.5.4.4  2006/06/25 17:01:55  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.5.4.3  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.5.4.2  2006/01/18 16:46:56  martius
 *   moved to osg
 *
 *   Revision 1.1.2.3  2006/01/17 17:02:47  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.2  2006/01/13 12:33:16  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.1  2006/01/13 12:24:06  martius
 *   env for external teaching input to the controller
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertnchannelcontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/stl_adds.h>

#include <vector>

#include <ode_robots/nimm2.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;


AbstractController* controller;
motor teaching[2];
bool connectRobots;

class ThisSim : public Simulation {
public:
  list<Joint*> joints;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    int number_x=3;
    int number_y=1;
    connectRobots = true;
    double distance = 0.3;

    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.05;
    //    global.odeConfig.setParam("gravity", 0);
    global.odeConfig.setParam("controlinterval", 5);

    // use Playground as boundary:
    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(111, 0.2, 1), 12);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    // for(int i=0; i<50; i++){
//       PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5);
//       s->setPosition(osg::Vec3(-4+(i/10),-4+(i%10),1));
//       global.obstacles.push_back(s);
//     }

    OdeRobot* nimm2;
    AbstractController* contrl;
    AbstractWiring* wiring;
    OdeAgent* agent;
    vector<OdeRobot*> robots(number_x);
    for (int i=-0; i<number_y; i++){
      for (int j=-0; j<number_x; j++){
        //      nimm2 = new Nimm2(odeHandle);
        Nimm2Conf conf = Nimm2::getDefaultConf();
        conf.speed=20;
        conf.force=3.0;
        conf.bumper=true;
        conf.cigarMode=true;
        wiring = new One2OneWiring(new ColorUniformNoise(0.1));
        if ((i==0) && (j==0)) {
          controller = new InvertMotorNStep();
          //          controller = new InvertMotorSpace(10);
          agent = new OdeAgent(global);
          nimm2 = new Nimm2(odeHandle, osgHandle, conf, "Nimm2Yellow");
          nimm2->setColor(Color(1.0,1.0,0));
          global.configs.push_back(controller);
          agent->init(controller, nimm2, wiring);
          controller->setParam("adaptrate", 0.000);
          //    controller->setParam("nomupdate", 0.0005);
          controller->setParam("epsC", 0.05);
          controller->setParam("epsA", 0.01);
          controller->setParam("epsC", 0.05);
          controller->setParam("rootE", 0);
          controller->setParam("steps", 2);
          controller->setParam("s4avg", 5);
          controller->setParam("s4del", 5);
          //          controller->setParam("factorB",0);
        } else {
          contrl = new InvertNChannelController(10);
          agent = new OdeAgent(global,PlotOption(NoPlot));
          nimm2 = new Nimm2(odeHandle, osgHandle, conf, "Nimm2_" + std::itos(i) + "_" + std::itos(j));
          agent->init(contrl, nimm2, wiring);
          contrl->setParam("adaptrate", 0.000);
          //    controller->setParam("nomupdate", 0.0005);
          contrl->setParam("epsC", 0.005);
          contrl->setParam("epsA", 0.001);
          contrl->setParam("rootE", 0);
          contrl->setParam("steps", 2);
          contrl->setParam("s4avg", 5);
          contrl->setParam("factorB",0);
        }
        nimm2->place(Pos(j*(2.5+distance),i*1.26,0));
        global.agents.push_back(agent);
        robots[j]=nimm2;
      }
      if(connectRobots)
        for(int j=0; j<number_x-1; j++){
          Joint* joint = new BallJoint(robots[j]->getMainPrimitive(),
                                       robots[j+1]->getMainPrimitive(),
                                       Pos((j+0.5)*(2.5+distance),i*1.26,0.30)
                                       );
          joint->init(odeHandle,osgHandle,true,distance/2);
          joints.push_back(joint);
        }
    }


  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(draw && connectRobots){
      FOREACH(list<Joint*>, joints,j){
        (*j)->update();
      }
    }
  };


  //Funktion die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;
    bool handled = false;
    FILE* f;
    switch ( key )
      {
      case 's' :
        f=fopen("controller","wb");
        controller->store(f) && printf("Controller stored\n");
        fclose(f);
        handled = true; break;
      case 'l' :
        f=fopen("controller","rb");
        controller->restore(f) && printf("Controller loaded\n");
        handled = true; break;
        fclose(f);
      }
    fflush(stdout);
    return handled;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Teachung: t","toggle mode");
    au.addKeyboardMouseBinding("Teaching: u","forward");
    au.addKeyboardMouseBinding("Teaching: j","backward");
    au.addKeyboardMouseBinding("Simulation: s","store");
    au.addKeyboardMouseBinding("Simulation: l","load");
  }


};

int main (int argc, char **argv)
{
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}

