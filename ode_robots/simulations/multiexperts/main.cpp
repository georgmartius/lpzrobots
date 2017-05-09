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
 *   Revision 1.15  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
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
#include "multisat.h"
#include <selforg/replaycontroller.h>

#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>

//#include <ode_robots/forcedsphere.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/barrel2masses.h>
#include <ode_robots/fourwheeled.h>

#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>
#include <ode_robots/replayrobot.h>



// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  OdeRobot* robot;
  AbstractController *controller;
  MultiSat *multisat;
  AbstractGround* playground;
  int useReinforcement;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    int num_barrels=0;
    int num_spheres=1;
    useReinforcement=0;

    int num_4wheel=0;
    //    useReinforcement=4;

    controller=0;
    multisat=0;
    robot=0;

    setCameraHomePos(Pos(-0.497163, 11.6358, 3.67419),  Pos(-179.213, -11.6718, 0));
    // initialization
    global.odeConfig.setParam("noise",0.05);
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",2);

    bool longsquarecorridor=false;

    playground=0;
    if(longsquarecorridor){
      playground = new Playground(odeHandle, osgHandle,osg::Vec3(500, 0.2, 1.2 ),
                                  5000/500, false);
      playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
      playground->setGroundTexture("Images/dusty.rgb");
      playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.6));
      playground->setPosition(osg::Vec3(0,0,0.1));
      playground->setTexture("");
      global.obstacles.push_back(playground);
    }

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
      robot = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)),
                                   conf, "Multi4_4h_Barrel", 0.4);
      robot->place ( osg::Matrix::rotate(M_PI/2, 1,0,0));

      InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
      cc.cInit=0.5;
      cc.useSD=true;
      AbstractController* controller = new InvertMotorNStep(cc); // selforg controller
      //AbstractController* controller = new SineController();
      //controller = new FFNNController("models/barrel/controller/nonoise.cx1-10.net", 10, true);
      MultiSatConf msc = MultiSat::getDefaultConf();
      msc.controller = controller;
      //      msc.numContext = 1;
      msc.numHidden = 4;
      msc.numSats = 4;
      multisat = new MultiSat(msc);

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
      OdeAgent* agent = new OdeAgent ( global );
      agent->addInspectable(controller); // add selforg controller to list of inspectables
      agent->init ( multisat , robot , wiring );
      //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
      global.configs.push_back ( multisat );
    }


    /* * * * SPHERES * * * */
    for(int i=0; i< num_spheres; i++){
      bool replay=true;
      global.odeConfig.setParam("noise", replay ? 0 : 0.05);
      //****************
      // const char* replayfilename="Sphere_reinforce_axis_rot.sel.log";
      const char* replayfilename="Sphere_slow_07-07-18.sel.log";
      //   const char* replayfilename="Sphere_long_rich_07-07-18.sel.log";
      // const char* replayfilename="Sphere_long_rich_2_07-07-31.sel.log";
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      conf.pendularrange  = 0.30;
      conf.brake          = 0.1;
      //      conf.pendularrange  = 0.15;
      //      conf.motorpowerfactor  = 150;
      // conf.spheremass  = 1;
      //      conf.spheremass  = 0.3;
      conf.motorsensor=false;
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
      //      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::Axis));
      conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel));

      robot = new Sphererobot3Masses(odeHandle, osgHandle.changeColor(Color(0,0.0,2.0)),
                                     conf, "Multi20_h_Sphereslow_v5_noy_sqrt", 0.3);
      //        conf, "Multi16_2h_Sphere_long_rich2", 0.3);
      robot->place ( osg::Matrix::translate(0,0,0.2));

      if(!replay){
        InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
        //      DerControllerConf cc = DerController::getDefaultConf();
        cc.cInit=1.0;
        //      cc.useSD=true;
        //controller = new DerController(cc);
        controller = new InvertMotorNStep(cc);
      }else
        controller = new ReplayController(replayfilename,true);

      MultiSatConf msc = MultiSat::getDefaultConf();
      msc.controller = controller;
      msc.numContext = 3;
      msc.numHidden = 4;
      msc.numSats   = 48;
      msc.eps0      = 0.1;
      //      msc.numSomPerDim = 3;
      msc.tauE1     = 50;
      msc.tauE2     = 500;
      msc.tauW      = 2000;
      //      msc.lambda_w  = 0.05;
      msc.lambda_w  = 0.2;

      msc.useDerive=false;
      msc.useY=false;
      multisat = new MultiSat(msc);

      //controller = new FFNNController("models/barrel/controller/nonoise.cx1-10.net", 10, true);
      controller->setParam("steps", 1);
      //    controller->setParam("adaptrate", 0.001);
      controller->setParam("adaptrate", 0.0);
      controller->setParam("nomupdate", 0.005);
      controller->setParam("epsC", 0.1);
      controller->setParam("epsA", 0.2);
      controller->setParam("rootE", 3);
      controller->setParam("logaE", 0);
      controller->setParam("sinerate", 15);
      controller->setParam("phaseshift", 0.45);

      // AbstractWiring* wiring = new SelectiveOne2OneWiring(new ColorUniformNoise(0.2), new select_from_to(0,2));
      //OdeAgent* agent = new OdeAgent ( PlotOption(GuiLogger, Robot, 5) );
      //     agent->init ( controller , robot , wiring );
      global.configs.push_back ( controller );

      AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise(0.20) );
      OdeAgent* agent = new OdeAgent ( global );
      agent->addInspectable(controller);
      agent->init ( multisat , robot , wiring );
      global.configs.push_back ( multisat );
      //      agent->setTrackOptions(TrackRobot(true, true, false, true, "ZSens", 50));
      global.agents.push_back ( agent );
    }

    /* * * * 4 Wheeled * * * */
    for(int i=0; i< num_4wheel; i++){
      bool replay=false;
      global.odeConfig.setParam("noise", replay ? 0 : 0.05);
      //****************
      const char* replayfilename="4Wheel_1.sel.log";

      FourWheeledConf fwc = FourWheeled::getDefaultConf();
      //       fwc.irFront=true;
      //       fwc.irBack=true;
      //       fwc.irSide=true;
      OdeRobot* nimm4 = new FourWheeled ( odeHandle, osgHandle, fwc, "Multi8_2h_Nimm4_nogat_noy_sqrt");
      nimm4->addSensor(std::make_shared<SpeedSensor>(SpeedSensor(1,SpeedSensor::TranslationalRel, Sensor::Z)));
      nimm4->addSensor(std::make_shared<SpeedSensor>(SpeedSensor(1,SpeedSensor::RotationalRel, Sensor::X)));
      robot = nimm4;
      robot->place ( osg::Matrix::translate(0,0,0.2));

      if(!replay){
        InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
        //      DerControllerConf cc = DerController::getDefaultConf();
        cc.cInit=1.0;
        //      cc.useSD=true;
        //controller = new DerController(cc);
        controller = new InvertMotorNStep(cc);
      }else
        controller = new ReplayController(replayfilename,true);

      MultiSatConf msc = MultiSat::getDefaultConf();
      msc.controller = controller;
      msc.numContext = 2;
      msc.numHidden = 2;
      msc.numSats   = 8;
      msc.eps0      = 0.01;
      //      msc.numSomPerDim = 3;
      msc.tauE1     = 20;
      msc.tauE2     = 200;
      msc.tauW     = 2000;

      msc.useDerive=false;
      msc.useY=false;
      multisat = new MultiSat(msc);

      //controller = new FFNNController("models/barrel/controller/nonoise.cx1-10.net", 10, true);
      controller->setParam("steps", 1);
      controller->setParam("adaptrate", 0.0);
      controller->setParam("epsC", 0.05);
      controller->setParam("epsA", 0.05);
      controller->setParam("rootE", 0);
      controller->setParam("logaE", 0);

      global.configs.push_back ( controller );

      AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise(0.20) );
      OdeAgent* agent = new OdeAgent ( global );
      agent->addInspectable(controller);
      agent->init ( multisat , robot , wiring );
      global.configs.push_back ( multisat );
      //      agent->setTrackOptions(TrackRobot(true, true, false, true, "ZSens", 50));
      global.agents.push_back ( agent );
    }



  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
//     InvertMotorNStep* c = dynamic_cast<InvertMotorNStep*>(controller);
//     if(c){
//       matrix::Matrix m(3,1, dBodyGetLinearVel( sphere->getMainPrimitive()->getBody()));
//       c->setReinforcement(m.map(abs).elementSum()/3 - 1);
//     }
    // if(useReinforcement==4){ // for FourWheelie
    //   InvertMotorNStep* c = dynamic_cast<InvertMotorNStep*>(controller);
    //   if(c && sensor){
    //      double dat[1];
    //     sensor->get(dat, 1);
    //      c->setReinforcement(fabs(dat[0]));
    //   }
    // }


  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    char filename[256];
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'y' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
        case 'Y' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
          //        case 'x' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , 10 , 0 ); break;
          //        case 'X' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , -10 , 0 ); break;
        case 'x' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , 0, 30); break;
        case 'X' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , 0,-30); break;
        case 'n' :
          std::cout << "Please type a filename stem:";
          std::cin >> filename;
          if(multisat) multisat->storeSats(filename);
          break;
        case 'l' :
          if(controller){
            std::cout << "Controller filename: ";
            std::cin >> filename;
            {
              FILE* f = fopen(filename,"rb");
              if(f && controller->restore(f)){
                printf("Controller restored\n");
                controller->setParam("epsC",0);
              }
              else printf("Error occured while restoring contoller\n");
            }
          }
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
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}

// with this seed the sphere does all sorts of things
//Use random number seed: 1181308527

// nice regular behaving sphere
// Use random number seed: 1181566764
