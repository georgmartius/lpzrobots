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
 *   Revision 1.5  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.4  2007/08/24 11:59:44  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2007/06/14 08:01:45  martius
 *   Pred error modulation by distance to minimum works
 *
 *   Revision 1.2  2007/06/11 08:26:49  martius
 *   sphere
 *
 *   Revision 1.1  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
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

#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/barrel2masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;

class BarrelNetControl : public FFNNController {
public:
  BarrelNetControl(const std::string& networkfilename):
    FFNNController(networkfilename, 1, false) {
  }

  virtual matrix::Matrix assembleNetworkInputXY(matrix::Matrix* xbuffer, matrix::Matrix* ybuffer) const {
    int tp = t+buffersize;
    Matrix m(xbuffer[tp%buffersize]);
    for(int i=1; i<=history; i++){
      m=m.above(xbuffer[(tp-i)%buffersize].above(ybuffer[(tp-i)%buffersize]));
    }
    return m;
  }

  virtual matrix::Matrix assembleNetworkOutput(const matrix::Matrix& output) const {
    return output.rows(number_sensors, number_sensors + number_motors);
  }
};

class SatNetControl : public FFNNController {
public:
  SatNetControl(const std::string& networkfilename):
    FFNNController(networkfilename, 1, false) {
  }

  virtual matrix::Matrix assembleNetworkInputXY(matrix::Matrix* xbuffer, matrix::Matrix* ybuffer) const {
    int tp = t+buffersize;
    Matrix m(xbuffer[tp%buffersize]);
    for(int i=1; i<=history; i++){
      m=m.above(xbuffer[(tp-i)%buffersize].above(ybuffer[(tp-i)%buffersize]));
    }
    return m;
  }

  virtual matrix::Matrix assembleNetworkOutput(const matrix::Matrix& output) const {
    return output.rows(number_sensors, number_sensors + number_motors);
  }
};

class SatNetControl_NoY : public FFNNController {
public:
  SatNetControl_NoY(const std::string& networkfilename):
    FFNNController(networkfilename, 1, true, 0) {
  }

  virtual matrix::Matrix assembleNetworkInputX(matrix::Matrix* xbuffer, matrix::Matrix* ybuffer) const {
    int tp = t+buffersize;
    Matrix m(xbuffer[tp%buffersize]);
    for(int i=1; i<=history; i++){
      m=m.above(xbuffer[(tp-i)%buffersize]);
    }
    return m;
  }

  virtual matrix::Matrix assembleNetworkOutput(const matrix::Matrix& output) const {
    return output.rows(number_sensors, number_sensors + number_motors);
  }
};




class ThisSim : public Simulation {
public:
  AbstractController *controller;
  OdeRobot* sphere1;
  const char* networkfilename;

  ThisSim(const char* netname) : networkfilename(netname) {}

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    int num_barrels=0;
    int num_spheres=1;

    setCameraHomePos(Pos(-0.497163, 11.6358, 3.67419),  Pos(-179.213, -11.6718, 0));
    // initialization
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",2);

    /* * * * BARRELS * * * */
    for(int i=0; i< num_barrels; i++){
      //****************
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      conf.pendularrange  = 0.15;
      conf.motorsensor=false;
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection, Sensor::X | Sensor::Y));
      conf.addSensor(new SpeedSensor(10, SpeedSensor::Translational, Sensor::X ));
      conf.irAxis1=false;
      conf.irAxis2=false;
      conf.irAxis3=false;
      conf.spheremass   = 1;
      sphere1 = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)),
                                    conf, "BarrelNet", 0.4);
      sphere1->place ( osg::Matrix::rotate(M_PI/2, 1,0,0));

      controller = new BarrelNetControl(networkfilename);

      // AbstractWiring* wiring = new DerivativeWiring(dc,new ColorUniformNoise());
      AbstractWiring* wiring = new SelectiveOne2OneWiring(new ColorUniformNoise(), new select_from_to(0,1));
      OdeAgent* agent = new OdeAgent ( PlotOption(File, Robot, 1) );
      agent->init ( controller , sphere1 , wiring );
      //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
    }


    /* * * * SPHERES * * * */
    for(int i=0; i< num_spheres; i++){
      //****************
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      conf.pendularrange  = 0.15;
      conf.motorpowerfactor  = 150;
      conf.motorsensor=false;
      //    conf.spheremass  = 1;
      conf.spheremass  = 0.3;
      //conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
      //      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::Axis));
      conf.irAxis1=true;
      conf.irAxis2=true;
      conf.irAxis3=true;
      //      conf.spheremass   = 1;
      sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(0,0.0,2.0)),
                                         conf, "Sphere_satctrl", 0.3);
      sphere1->place ( Pos(0,0,0.2));

      controller = new SatNetControl(networkfilename);

      One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise(0.20) );
      OdeAgent* agent = new OdeAgent ( plotoptions );
      agent->init ( controller , sphere1 , wiring );
      //      agent->setTrackOptions(TrackRobot(true, true, false, true, "ZSens", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
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
  if(argc<2){
    printf("Provide network name!\n");
    return 1;
  }else{
    ThisSim sim(argv[1]);
    // run simulation
    return sim.run(argc, argv) ? 0 : 1;
  }
}

