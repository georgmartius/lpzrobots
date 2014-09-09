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
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

//#include <selforg/pimax.h>
#include <selforg/sox.h>
#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/ffnncontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>

#include <ode_robots/barrel2masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>

#include "roscontroller.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;


class ThisSim : public Simulation {
public:
  AbstractController *controller;
  OdeRobot* sphere1;
  int useReinforcement;
  Sensor* sensor;

  double friction;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    int num_barrels=1;
    int num_barrels_test=0;

    sensor=0;
    setCameraMode(Follow);

    bool normalplayground=false;

    //    setCameraHomePos(Pos(-0.497163, 11.6358, 3.67419),  Pos(-179.213, -11.6718, 0));
    setCameraHomePos(Pos(-2.60384, 13.1299, 2.64348),  Pos(-179.063, -9.7594, 0));
    // initialization
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("realtimefactor",5);

    // add a new parameter to be configured on the console
    global.odeConfig.addParameterDef("friction", &friction, 0.1, "rolling friction coefficient");


    if(normalplayground){
      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(20, 0.01, 0.01 ), 1);
      playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
      playground->setGroundTexture("Images/really_white.rgb");
      playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.1));
      playground->setPosition(osg::Vec3(0,0,0.05));
      global.obstacles.push_back(playground);
    }


    /* * * * BARRELS * * * */
    for(int i=0; i< num_barrels; i++){
      //****************
      Sphererobot3MassesConf conf = Barrel2Masses::getDefaultConf();
      conf.pendularrange  = 0.3;//0.15;
      conf.motorpowerfactor  = 200;//150;
      conf.motorsensor=false;
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection, Sensor::X | Sensor::Y));
      conf.spheremass   = 1;

      conf.addSensor(new SpeedSensor(10, SpeedSensor::Translational, Sensor::X ));
      conf.irAxis1=false;
      conf.irAxis2=false;
      conf.irAxis3=false;
      conf.axesShift = 0;
      // conf.axesShift = conf.diameter/2 - conf.pendularrange/2;
      sphere1 = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)),
                                    conf, "Barrel1", 0.4);

      sphere1->place (osg::Matrix::rotate(M_PI/2, 1,0,0)*osg::Matrix::translate(0,0,0.2));

      // InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
      // cc.cInit=1;
      // //    cc.useSD=true;
      // controller = new InvertMotorNStep(cc);

      // controller->setParam("steps", 2);
      // controller->setParam("adaptrate", 0.0);
      // controller->setParam("nomupdate", 0.00);
      // controller->setParam("epsC", 0.03);
      // controller->setParam("epsA", 0.05);
      // controller->setParam("rootE", 0);
      // controller->setParam("logaE", 0);
      //controller = new PiMax();
      //controller->setParam("epsC", 0.001);

      // controller = new Sox();
      // controller->setParam("epsC", 0.5);
      controller = new ROSController("Test");

      AbstractWiring* wiring = new SelectiveOne2OneWiring(new ColorUniformNoise(), new select_from_to(0,1));
      //      OdeAgent* agent = new OdeAgent ( PlotOption(File, Robot, 1) );
      OdeAgent* agent = new OdeAgent ();
      agent->init ( controller , sphere1 , wiring );
      //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
    }


    /* * * * TEST BARRELS * * * */
    for(int i=0; i< num_barrels_test; i++){
      global.odeConfig.setParam("realtimefactor",1);
      //****************
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      conf.pendularrange  = 0.15;
      conf.motorsensor=true;
      conf.irAxis1=false;
      conf.irAxis2=false;
      conf.irAxis3=false;
      conf.spheremass   = 1;
      sphere1 = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)),
                                    conf, "Barrel1", 0.4);
      sphere1->place ( osg::Matrix::rotate(M_PI/2, 1,0,0));

      controller = new SineController();
      controller->setParam("sinerate", 15);
      controller->setParam("phaseshift", 0.45);

      //       DerivativeWiringConf dc = DerivativeWiring::getDefaultConf();
      //       dc.useId=true;
      //       dc.useFirstD=false;
      //       AbstractWiring* wiring = new DerivativeWiring(dc,new ColorUniformNoise());
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise());
      OdeAgent* agent = new OdeAgent ();
      agent->init ( controller , sphere1 , wiring );
      //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
    }



  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(!pause){
      if(friction>0){
        OdeRobot* robot = globalData.agents[0]->getRobot();
        Pos vel = robot->getMainPrimitive()->getAngularVel();
        robot->getMainPrimitive()->applyTorque(-vel*friction);
      }
    }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'y' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
        case 'Y' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
        case 'x' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 10 , 0 ); break;
        case 'X' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , -10 , 0 ); break;
        case 'S' : controller->setParam("sinerate", controller->getParam("sinerate")*1.2);
          printf("sinerate : %g\n", controller->getParam("sinerate"));
          break;
        case 's' : controller->setParam("sinerate", controller->getParam("sinerate")/1.2);
          printf("sinerate : %g\n", controller->getParam("sinerate"));
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
  ThisSim sim;
  sim.setCaption("Spherical Robot (lpzrobots Simulator)   Martius,Der 2007");
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}

