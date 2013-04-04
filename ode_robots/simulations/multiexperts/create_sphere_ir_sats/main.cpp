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
 *   Revision 1.3  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.2  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.1  2007/08/24 11:59:44  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2007/08/06 14:25:57  martius
 *   new version without gating network
 *
 *   Revision 1.2  2007/06/18 08:11:22  martius
 *   nice version with many agents
 *
 *   Revision 1.1  2007/06/14 07:59:30  martius
 *   nice scanning of initial parameters
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/ffnncontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/matrix.h>
#include <selforg/replaycontroller.h>
#include "multisat.h"

#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/barrel2masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;
using namespace std;


class ThisSim : public Simulation {
public:
  AbstractController *controller;
  MultiSat* multisat;
  AbstractWiring* wiring;
  OdeAgent* agent;
  OdeRobot* sphere;
  Sphererobot3MassesConf conf;
  const char* replayfilename;

  ThisSim(const char* replayname) : replayfilename(replayname) {}

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-0.497163, 11.6358, 3.67419),  Pos(-179.213, -11.6718, 0));
    // initialization
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("realtimefactor",1);

    //    global.odeConfig.setParam("realtimefactor",0);
//     global.odeConfig.setParam("drawinterval",2000);

    sphere=0;

    //****************
    conf = Sphererobot3Masses::getDefaultConf();
    conf.pendularrange  = 0.15;
    conf.motorpowerfactor  = 150;
    conf.motorsensor=false;
    //    conf.spheremass  = 1;
    conf.spheremass  = 0.3;
    //    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    conf.irAxis1=true;
    conf.irAxis2=true;
    conf.irAxis3=true;
    // conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel));

    // create new sphere
    sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(0,0.0,2.0)),
                                      conf, "sat_log", 0.3);

    sphere->place(Pos(0,0,0.1));

    controller = new ReplayController(replayfilename,true);

    MultiSatConf msc = MultiSat::getDefaultConf();
    msc.controller = controller;
    msc.numContext = 0;
    msc.numHidden = 6;
    msc.numSats   = 1;
    msc.penalty   = 10.0;
    msc.eps0      = 0.005;
    msc.deltaMin  = 1/500.0;
    //      msc.numSomPerDim = 3;
    msc.tauE1     = 40;
    msc.tauE2     = 1000;
    msc.tauW     = 10000;

    msc.useDerive=false;
    multisat = new MultiSat(msc);


    wiring = new One2OneWiring ( new ColorUniformNoise(0.20) );
    agent = new OdeAgent ( plotoptions );
    agent->init ( multisat , sphere , wiring );
    global.agents.push_back ( agent );



  }


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    char filename[256];
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'y' : dBodyAddForce ( sphere->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
        case 'Y' : dBodyAddForce ( sphere->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
        case 'x' : dBodyAddTorque ( sphere->getMainPrimitive()->getBody() , 0 , 10 , 0 ); break;
        case 'X' : dBodyAddTorque ( sphere->getMainPrimitive()->getBody() , 0 , -10 , 0 ); break;
        case 'S' : controller->setParam("sinerate", controller->getParam("sinerate")*1.2);
          printf("sinerate : %g\n", controller->getParam("sinerate"));
          break;
        case 's' : controller->setParam("sinerate", controller->getParam("sinerate")/1.2);
          printf("sinerate : %g\n", controller->getParam("sinerate"));
          break;
        case 'n' :
          std::cout << "Please type a filename stem:";
          std::cin >> filename;
          if(multisat) multisat->storeSats(filename);
          break;
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
  if(argc<2){
    printf("Provide network name!\n");
    return 1;
  }else{
    ThisSim sim(argv[1]);
    // run simulation
    return sim.run(argc, argv) ? 0 : 1;
  }
}

