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
 *   Revision 1.1  2007/08/06 14:25:57  martius
 *   new version without gating network
 *
 *   Revision 1.8  2007/07/19 15:44:32  martius
 *   new multisat version without gating
 *
 *   Revision 1.7  2007/06/22 14:25:07  martius
 *   *** empty log message ***
 *
 *   Revision 1.6  2007/06/21 16:31:54  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2007/06/18 08:11:22  martius
 *   nice version with many agents
 *
 *   Revision 1.4  2007/06/14 08:01:45  martius
 *   Pred error modulation by distance to minimum works
 *
 *   Revision 1.3  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.2  2007/05/23 14:07:34  martius
 *   *** empty log message ***
 *
 *   Revision 1.14  2007/03/26 13:15:51  martius
 *   new makefile with readline support
 *
 *   Revision 1.13  2007/02/23 19:36:42  martius
 *   useSD
 *
 *   Revision 1.12  2007/02/23 15:14:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.11  2007/01/26 12:07:08  martius
 *   orientationsensor added
 *
 *   Revision 1.10  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.9  2006/12/01 16:19:05  martius
 *   barrel in use
 *
 *   Revision 1.8  2006/11/29 09:16:09  martius
 *   modell stuff
 *
 *   Revision 1.7  2006/07/14 12:23:52  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.6.4.6  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.6.4.5  2006/05/15 12:29:43  robot3
 *   handling of starting the guilogger moved to simulation.cpp
 *   (is in internal simulation routine now)
 *
 *   Revision 1.6.4.4  2006/03/29 15:10:22  martius
 *   *** empty log message ***
 *
 *   Revision 1.6.4.3  2006/02/17 16:47:55  martius
 *   moved to new system
 *
 *   Revision 1.15.4.3  2006/01/12 15:17:39  martius
 *   *** empty log message ***
 *
 *   Revision 1.15.4.2  2006/01/10 20:33:50  martius
 *   moved to osg
 *
 *   Revision 1.15.4.1  2005/11/15 12:30:17  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.15  2005/11/09 14:54:46  fhesse
 *   nchannelcontroller used
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/ffnncontroller.h>
#include "multisat_checksats.h"
#include <selforg/replaycontroller.h>

#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>

#include <ode_robots/forcedsphere.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/barrel2masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>
#include <ode_robots/replayrobot.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

char* file;

class ThisSim : public Simulation {
public:
  OdeRobot* sphere1;
  AbstractController *controller;
  MultiSatCheck *multisat;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    int num_barrels=0;
    int num_spheres=1;
    controller=0;
    multisat=0;
    sphere1=0;

    setCameraHomePos(Pos(-0.497163, 11.6358, 3.67419),  Pos(-179.213, -11.6718, 0));
    // initialization
    global.odeConfig.setParam("noise",0.05);
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",2);


    /* * * * BARRELS * * * */
    for(int i=0; i< num_barrels; i++){
      //****************
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      conf.pendularrange  = 0.15;
      conf.motorsensor=false;
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection, Sensor::X | Sensor::Y));
      //      conf.addSensor(new SpeedSensor(5, SpeedSensor::Translational, Sensor::X ));
      conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel, Sensor::Z ));
      conf.irAxis1=false;
      conf.irAxis2=false;
      conf.irAxis3=false;
      conf.spheremass   = 0.3; // 1
      sphere1 = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)),
                                    conf, "Multi4_4h_Barrel", 0.4);
      sphere1->place ( osg::Matrix::rotate(M_PI/2, 1,0,0));

      InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
      cc.cInit=0.5;
      cc.useSD=true;
      AbstractController* controller = new InvertMotorNStep(cc); // selforg controller
      //AbstractController* controller = new SineController();
      //controller = new FFNNController("models/barrel/controller/nonoise.cx1-10.net", 10, true);
      MultiSatCheckConf msc = MultiSatCheck::getDefaultConf();
      msc.controller = controller;
      msc.numContext = 1;
      msc.numHidden = 4;
      msc.numSats = 4;
      multisat = new MultiSatCheck(msc);

      controller->setParam("steps", 2);
      //    controller->setParam("adaptrate", 0.001);
      controller->setParam("adaptrate", 0.0);
      controller->setParam("nomupdate", 0.005);
      //      controller->setParam("epsC", 0.03);
      //      controller->setParam("epsA", 0.05);
      controller->setParam("epsC", 0.02);
      controller->setParam("epsA", 0.03);
      controller->setParam("rootE", 3);
      controller->setParam("logaE", 0);

//       DerivativeWiringConf dc = DerivativeWiring::getDefaultConf();
//       dc.useId=true;
//       dc.useFirstD=false;
//       AbstractWiring* wiring = new DerivativeWiring(dc,new ColorUniformNoise());
//      AbstractWiring* wiring = new SelectiveOne2OneWiring(new ColorUniformNoise(), new select_from_to(0,1));
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.2));
      //      OdeAgent* agent = new OdeAgent ( PlotOption(File, Robot, 1) );
      OdeAgent* agent = new OdeAgent ( plotoptions );
      agent->addInspectable(controller); // add selforg controller to list of inspectables
      agent->init ( multisat , sphere1 , wiring );
      //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
      global.configs.push_back ( multisat );
    }


    /* * * * SPHERES * * * */
    for(int i=0; i< num_spheres; i++){
      bool replay=true;
      global.odeConfig.setParam("noise", replay ? 0 : 0.1);
      //****************
      const char* replayfilename="../Sphere_reinforce_axis_rot.sel.log";
      //const char* replayfilename="Sphere_slow_07-07-18.sel.log";
      //   const char* replayfilename="Sphere_long_rich_07-07-18.sel.log";
      //      const char* replayfilename="../Sphere_long_rich_2_07-07-31.sel.log";
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      conf.pendularrange  = 0.15;
      conf.motorpowerfactor  = 150;
      conf.spheremass  = 1;
      conf.motorsensor=false;
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
      //      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::Axis));
      conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel));

      sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(0,0.0,2.0)),
                                         conf, "Multi20_2h_Sphere_satcheck", 0.3);
      sphere1->place ( osg::Matrix::translate(0,0,0.2));

      if(!replay){
        InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
        //      DerControllerConf cc = DerController::getDefaultConf();
        cc.cInit=1.0;
        //      cc.useSD=true;
        //controller = new DerController(cc);
        controller = new InvertMotorNStep(cc);
      }else
        controller = new ReplayController(replayfilename,true);

      MultiSatCheckConf msc = MultiSatCheck::getDefaultConf();
      msc.controller = controller;
      msc.numContext=3;
      msc.numSats=20;
      msc.useDerive=false;
      multisat = new MultiSatCheck(msc);

      AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise(0.20) );
      OdeAgent* agent = new OdeAgent ( plotoptions );
      agent->init ( multisat , sphere1 , wiring );
      global.configs.push_back ( multisat );
      global.agents.push_back ( agent );

      FILE* f =  fopen(file,"rb");
      multisat->restore(f);
      fclose(f);

    }


  }

};

int main (int argc, char **argv)
{
  ThisSim sim;
  // run simulation
  if(argc<2) {
    std::cerr << "Provide multisat controller to load\n";
    exit(1);
  }
  file = argv[1];
  return sim.run(argc, argv) ? 0 : 1;
}

