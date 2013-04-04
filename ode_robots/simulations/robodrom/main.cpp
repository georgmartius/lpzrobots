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
 *   Revision 1.23  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.22  2011/04/28 09:46:49  martius
 *   cleanup
 *
 *   Revision 1.21  2011/02/13 20:30:44  martius
 *   parameters and eliptic version
 *
 *   Revision 1.20  2011/02/11 18:16:41  martius
 *   single pot replication
 *
 *   Revision 1.19  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.18  2007/01/26 12:07:08  martius
 *   orientationsensor added
 *
 *   Revision 1.17  2006/09/20 12:56:32  martius
 *   *** empty log message ***
 *
 *   Revision 1.16  2006/07/14 12:23:51  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.15.4.7  2006/06/23 09:05:55  robot3
 *   robodrom has now golden ground, modified some other things
 *
 *   Revision 1.15.4.6  2006/05/29 20:09:36  martius
 *   macrospheres
 *
 *   Revision 1.15.4.5  2006/05/29 18:58:13  martius
 *   terrainground is used again
 *
 *   Revision 1.1.4.4  2006/05/28 22:14:18  martius
 *   new meshground
 *
 *   Revision 1.15.4.3  2006/05/23 21:57:28  martius
 *   new system
 *
 *   Revision 1.15.4.2  2005/11/16 11:27:38  martius
 *   invertmotornstep has configuration
 *
 *   Revision 1.15.4.1  2005/11/15 12:29:56  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.15  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>

// used robot
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>



// used arena
#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivecapsule.h>

// used controller
#include <selforg/sox.h>
#include <selforg/semox.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>

// fetch all the stuff of lpzrobots and std into scope
using namespace lpzrobots;
using namespace std;

Sphererobot3Masses* sphere ;
//const double height = 6.5;
const double height = 2;

enum Env {ThreeBump, SingleBasin, ElipticBasin};
Env env = SingleBasin;
bool track=false;
const char* envnames[3] = {"ThreePot", "SingleBasin","ElipticBasin"};

class ThisSim : public Simulation {
public:


  void addRobot(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, int i){
    Color col;

    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    conf.diameter=1.0;
    conf.pendularrange= 0.30; // 0.15;
    conf.motorpowerfactor  = 150;
    conf.spheremass = 1;
    conf.motorsensor=false;

    conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel));


    //SphererobotArms* sphere = new SphererobotArms ( odeHandle, conf);
    switch(i){
    case 0:
      col.r()=0;
      col.g()=1;
      col.b()=0.1;
      sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere 1", 0.4);
      sphere->place ( osg::Matrix::translate(9.5 , 0 , height+1 ));
      break;
    case 1:
      col.r()=1;
      col.g()=0.2;
      col.b()=0;
      sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere 2", 0.4);
      sphere->place ( osg::Matrix::translate( 2 , -2 , height+1 ));
      break;
    case 3:
      col.r()=0;
      col.g()=0;
      col.b()=1;
      sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere" +
                                        string(envnames[env]), 0.4);
      sphere->place ( osg::Matrix::translate( 0 , 0 , .5 ));
      break;
    default:
    case 2:
      col.r()=0;
      col.g()=1;
      col.b()=0.4;
      sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere 3", 0.4);
      sphere->place ( osg::Matrix::translate( double(rand())/RAND_MAX*10 , 0 , height+1 ));
      break;
    }


    // //AbstractController *controller = new InvertNChannelController(10);
    // AbstractController *controller = new InvertMotorNStep();
    // //    controller->setParam("factorB", 0.1);
    // controller->setParam("steps", 2);
    // //    controller->setParam("nomupdate", 0.005);
    AbstractController *controller = new Sox(1.2,false);
    controller->setParam("epsC", 0.2);
    controller->setParam("epsA", 0.2);
    if(env==ElipticBasin){
      controller->setParam("epsC", 0.3);
      controller->setParam("epsA", 0.3);
    }
    controller->setParam("Logarithmic", 0);
    controller->setParam("sense", 0.5);
    // 1297536669
    // 1297586370.000000

    // SeMoXConf cc = SeMoX::getDefaultConf();
    // cc.modelExt=true;
    // AbstractController* controller = new SeMoX(cc);

    // controller->setParam("epsC", 0.05);
    // controller->setParam("epsA", 0.1);
    // controller->setParam("rootE", 0);
    // controller->setParam("steps", 1);
    // controller->setParam("s4avg", 1);
    // controller->setParam("dampModel", 0.9e-5);
    // controller->setParam("discountS", 0.05);



    //    AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
  AbstractWiring* wiring = new SelectiveOne2OneWiring(new WhiteUniformNoise(),
                                                      new select_from_to(0,2),
                                                      AbstractWiring::Robot);

    OdeAgent* agent;
    if(i==0 || i==3 ){
      agent = new OdeAgent (global);
    }
    else
      agent = new OdeAgent (global, PlotOption(NoPlot));

    agent->init ( controller , sphere , wiring );
    if(track)
      agent->setTrackOptions(TrackRobot(true,false,false,false,"",2));

    global.agents.push_back ( agent );
    global.configs.push_back ( controller );
    global.configs.push_back ( sphere);
  }

  void removeRobot(GlobalData& global){
    if(!global.agents.empty()){
      OdeAgentList::iterator i =  global.agents.end()-1;
      delete (*i)->getRobot();
      delete (*i)->getController();
      delete (*i);
      global.agents.erase(i);
    }
  }


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(16.3129, 18.2069, 12.5683),  Pos(137.914, -28.1771, 0));
    setCameraMode(Static);
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.05;
    //  global.odeConfig.setParam("gravity", 0);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double        height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)
    Playground* playground;
    if(env==ElipticBasin){
      playground = new Playground(odeHandle, osgHandle,
                                  osg::Vec3(20, 0.2, height+1.f), 2);
    }else{
      playground = new Playground(odeHandle, osgHandle,
                                  osg::Vec3(20, 0.2, height+0.3f), 1);
    }
    playground->setColor(Color(.1,0.7,.1));
    playground->setTexture("");
    playground->setPosition(osg::Vec3(0,0,0.01)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    int numpassive=0;
    switch(env){
    case ThreeBump:
      {
        //     TerrainGround* terrainground =
        //       new TerrainGround(odeHandle, osgHandle,
        //                          "terrains/threebumps.ppm", "terrains/threebumps.ppm", 20, 20, height);
        TerrainGround* terrainground =
          new TerrainGround(odeHandle, osgHandle.changeColor(Color(1.0f,194.0/255.0,41.0/255.0)),
                            "terrains/macrospheresLMH_64.ppm",""/*"Images/dusty.rgb" "terrains/macrospheresTex_256.ppm"*/,
                            20, 20, height, OSGHeightField::LowMidHigh);
        terrainground->setPose(osg::Matrix::translate(0, 0, 0.1));
        global.obstacles.push_back(terrainground);
        addRobot(odeHandle, osgHandle, global, 0);
        addRobot(odeHandle, osgHandle, global, 1);
        numpassive=4;
      }
      break;
    case SingleBasin:
      // at Radius 3.92 height difference of 0.5 and at 6.2 height difference of 1
      // ./start -single track -f 2 -r 1297536669

    case ElipticBasin:
      // ./start -eliptic -f 5 -track -r 1297628680
      {
        TerrainGround* terrainground =
          new TerrainGround(odeHandle, osgHandle.changeColor(Color(1.0f,1.0f,1.0f)),
                            //                        "terrains/dip128_flat.ppm","terrains/dip128_flat_texture.ppm",
                            "terrains/dip128.ppm","terrains/dip128_texture.ppm",
                            20, env == SingleBasin ? 20 : 40, height, OSGHeightField::Red);
        terrainground->setPose(osg::Matrix::translate(0, 0, 0.1));
        global.obstacles.push_back(terrainground);
        addRobot(odeHandle, osgHandle, global, 3);
      }
      break;
    }


    // add passive spheres as obstacles
    // - create pointer to sphere (with odehandle, osghandle and
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere
    // - set a texture for the sphere
    // - add sphere to list of obstacles
    for (int i=0; i< numpassive; i+=1){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.5,0.1);
      s1->setPosition(osg::Vec3(-8+2*i,-2,height+0.5));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }


  }


};


int main (int argc, char **argv)
{
  if(Simulation::contains(argv,argc,"-eliptic")>0)
    env=ElipticBasin;
  if(Simulation::contains(argv,argc,"-single")>0)
    env=SingleBasin;
  if(Simulation::contains(argv,argc,"-three")>0)
    env=ThreeBump;
  track = (Simulation::contains(argv,argc,"-track")>0);

  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}





