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
 *   Revision 1.10  2009/08/05 23:25:57  martius
 *   adapted small things to compile with changed Plotoptions
 *
 *   Revision 1.9  2009/02/02 16:08:13  martius
 *   minor changes
 *
 *   Revision 1.8  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.7  2007/11/07 13:26:59  martius
 *   tidy up
 *
 *   Revision 1.6  2007/03/26 13:15:51  martius
 *   new makefile with readline support
 *
 *   Revision 1.5  2007/02/23 15:14:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2007/02/12 13:17:41  martius
 *   new teaching interface
 *
 *   Revision 1.3  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.2  2006/07/14 12:23:53  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.9  2006/06/25 17:01:56  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.1.2.8  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.1.2.7  2006/03/29 15:10:22  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.6  2006/02/08 16:15:27  martius
 *   teaching via keys
 *
 *   Revision 1.1.2.5  2006/01/31 16:45:50  martius
 *   neuronviz output
 *
 *   Revision 1.1.2.4  2006/01/18 16:47:06  martius
 *   *** empty log message ***
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
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/invertmotornstep.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include <ode_robots/nimm2.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:

  InvertMotorNStep*controller;
  OdeRobot* vehicle;
  motor teaching[2];
  int useTeaching;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-5.44372, 7.37141, 3.31768),  Pos(-142.211, -21.1623, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("noise", 0.05);
    global.odeConfig.setParam("controlinterval", 1);
    //    global.odeConfig.setParam("gravity", 0);

    // use Playground as boundary:
    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(100, 0.2, 1), 2);
    // playground->setColor(Color(0,0,0,0.8));
    playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    for(int i=0; i<0; i++){ //20
      PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5,10);
      s->setPosition(osg::Vec3(-4+2*(i/5),-4+2*(i%5),2));
      global.obstacles.push_back(s);
    }

    Nimm2Conf c = Nimm2::getDefaultConf();
    c.sphereWheels=false;
    c.sphereWheels=false;

    vehicle = new Nimm2(odeHandle, osgHandle, c, "Nimm2");
    //OdeRobot* vehicle = new Nimm4(odeHandle, osgHandle);
    vehicle->place(Pos(0,0,0.6));

    // create pointer to controller
    // push controller in global list of configurables
    //  AbstractController *controller = new InvertNChannelController(10);
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.cInit=1.0;
    cc.useS=false;

    useTeaching=0;

    controller = new InvertMotorNStep(cc);
    controller->setParam("adaptrate", 0.000);
    //    controller->setParam("nomupdate", 0.0005);
    controller->setParam("epsC", 0.1);
    controller->setParam("epsA", 0.1);
    controller->setParam("rootE", 0);
    controller->setParam("steps", 1);
    controller->setParam("s4avg", 1);

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(controller);


  }

  //Funktion die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;
    bool handled = false;
    // FILE* f;
    //    double m;
    //    motor motors[2];
    switch ( key )
      {
      case 'u' :
        teaching[0] = std::min(0.95, teaching[0]+0.1);
        teaching[1] = std::min(0.95, teaching[1]+0.1);
        printf("Teaching Signal: %f, %f\n", teaching[0], teaching[1]);
        handled = true;
        break;
      case 'j' :
        teaching[0] = std::max(-0.95, teaching[0]-0.1);
        teaching[1] = std::max(-0.95, teaching[1]-0.1);
        printf("Teaching Signal: %f, %f\n", teaching[0], teaching[1]);
        handled = true;
        break;
      case 'i' :
        useTeaching = (useTeaching+1)%3;
        printf("%s\n", useTeaching==0 ? "Teaching disabled" :
               (useTeaching ==1 ? "enabled motor teaching" : "enabled sensor teaching"));
        handled = true;
        break;
      }
    fflush(stdout);
    return handled;
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control && useTeaching==1){
      controller->setMotorTeachingSignal(teaching, 2);
    }
    if(control && useTeaching==2){
      controller->setSensorTeachingSignal(teaching, 2);
    }

  };


  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Teaching: u","forward");
    au.addKeyboardMouseBinding("Teaching: j","backward");
    au.addKeyboardMouseBinding("Distal Teaching: i","forward");
    au.addKeyboardMouseBinding("Distal Teaching: k","backward");
  }

};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 :  1;
}



