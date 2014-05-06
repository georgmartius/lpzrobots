/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
 *    mai00bvz@studserv.uni-leipzig.de                                     *
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
 *   Revision 1.7  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.6  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.5  2009/08/11 12:30:39  robot12
 *   update the simstep variable from "this" to globalData! (guettler)
 *
 *   Revision 1.4  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.3  2008/01/15 17:42:09  fhesse
 *   *** empty log message ***
 *
 *   Revision 1.2  2008/01/15 15:30:47  fhesse
 *   *** empty log message ***
 *
 *   Revision 1.1  2008/01/12 15:51:41  fhesse
 *   initial version
 *
 *   Revision 1.20  2007/08/24 08:35:19  martius
 *   restored clean example
 *
 *
 *   Revision 1.18  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.17  2006/08/11 15:46:34  martius
 *   *** empty log message ***
 *
 *   Revision 1.16  2006/08/04 15:07:46  martius
 *   documentation
 *
 *   Revision 1.15  2006/07/14 12:23:53  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.14.4.16  2006/06/25 17:01:56  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.14.4.15  2006/05/18 11:55:56  robot3
 *   made playground smaller (for shadowing issues)
 *
 *   Revision 1.14.4.14  2006/05/15 13:11:30  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.14.4.13  2006/04/25 09:06:16  robot3
 *   *** empty log message ***
 *
 *   Revision 1.14.4.12  2006/03/31 12:14:49  fhesse
 *   orange color for nimm robot
 *
 *   Revision 1.14.4.11  2006/03/31 11:27:53  fhesse
 *   documentation updated
 *   one sphere removed (todo  fix problem with sphere placing)
 *
 *   Revision 1.14.4.10  2006/01/12 15:17:46  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.9  2005/12/29 15:55:33  martius
 *   end is obsolete
 *
 *   Revision 1.14.4.8  2005/12/29 15:47:12  martius
 *   changed to real Sim class
 *
 *   Revision 1.14.4.7  2005/12/14 15:37:25  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.6  2005/12/13 18:12:09  martius
 *   switched to nimm2
 *
 *   Revision 1.14.4.5  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.4  2005/12/09 16:53:17  martius
 *   camera is working now
 *
 *   Revision 1.14.4.3  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *   Revision 1.14.4.2  2005/11/24 16:19:12  fhesse
 *   include corrected
 *
 *   Revision 1.14.4.1  2005/11/15 12:30:07  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>

// include ode library
#include <ode-dbl/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

// used robot
#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>


// used arena
#include <ode_robots/playground.h>
#include <ode_robots/complexplayground.h>

//my substance definitions
#include "mysubstance.h"

// used passive spheres
#include <ode_robots/passivesphere.h>

// used controller
#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
//#include <selforg/sinecontroller.h>
// #include <staticcontroller.h>


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:

  AbstractController *controller;   // outside start() to be able to use it in writeControllerParamsToFile() and loadControllerParamsFromFile(), see far below

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // first: position(x,y,z) second: view(alpha,beta,gamma)
    // gamma=0;
    // alpha == horizontal angle
    // beta == vertical angle
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set noise to 0.1
    global.odeConfig.noise=0.05;
    //  global.odeConfig.setParam("gravity", 0);
   global.odeConfig.setParam("realtimefactor", 0);
   global.odeConfig.setParam("controlinterval", 2);

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double        height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)

    // odeHandle and osgHandle are global references
    // vec3 == length, width, height
//     Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(12, 0.2, 0.5));
//     playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
//     // register playground in obstacles list
//     global.obstacles.push_back(playground);

    OdeHandle PlaygroundHandle(odeHandle);
    //PlaygroundHandle.substance.toRubber(40);
    //PlaygroundHandle.substance.toMetal(2);
    //PlaygroundHandle.substance.toFoam(50);
    //PlaygroundHandle.substance.toPlastic(0.5);
    PlaygroundHandle.substance.roughness=0.3;
    PlaygroundHandle.substance.slip=0.1;
    PlaygroundHandle.substance.hardness=200;
    PlaygroundHandle.substance.elasticity=0.0;


//     Playground* playground = new Playground(PlaygroundHandle, osgHandle, osg::Vec3(16, 0.2, 0.5),
//                                   /*double factorxy = 1*/1,/* bool createGround=true*/ true);
     Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(16, 0.2, 0.8),
                                   /*double factorxy = 1*/1,/* bool createGround=true*/ true);


//     Playground* playground = (Playground*)new ComplexPlayground(PlaygroundHandle, osgHandle ,
//                      "playground2.fig",
//                /*double factor =*/ 1, /*double heightfactor=*/0.02, /*bool createGround=*/false);

playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
//Substance s;
//s.toDefaultSubstance();
//playground->setGroundSubstance(s); -> no ground used

     global.obstacles.push_back(playground);

    // add passive spheres as obstacles
    // - create pointer to sphere (with odehandle, osghandle and
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere
    // - set a texture for the sphere
    // - add sphere to list of obstacles
    for (int i=0; i < 0/*2*/; i++){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
      s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }

    // use Nimm2 vehicle as robot:
    // - get default configuration for nimm2
    // - activate bumpers, cigar mode and infrared front sensors of the nimm2 robot
    // - create pointer to nimm2 (with odeHandle, osg Handle and configuration)
    // - place robot
/*    Nimm2Conf c = Nimm2::getDefaultConf();
    c.force   = 4;
    c.bumper  = true;
    c.cigarMode  = true;
    // c.irFront = true;
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, c, "Nimm2");
    vehicle->place(Pos(0,0,0));*/

    // use Nimm4 vehicle as robot:
    // - create pointer to nimm4 (with odeHandle and osg Handle and possible other settings, see nimm4.h)
    // - place robot
//     OdeRobot* vehicle = new Nimm4(odeHandle, osgHandle, "Nimm4",
//                /*double size=1.0*/1.0, /*double force=3*/1, /*double speed=15*/ 15,
//                /*bool sphereWheels =true*/ true);

    Nimm2Conf conf = Nimm2::getDefaultConf();
//     conf.size=1;
     conf.force=1;//5;
//     conf.speed=12;
//     conf.sphereWheels=true;
//     conf.wheelSize=1;
//     conf.bumper=false;
//     conf.cigarMode=false;
//     conf.irFront=false;
//     conf.irBack=false;
//     conf.irSide=false;
//     conf.irRange=3;
     conf.singleMotor=true;//false;
//           conf.visForce=false;
//     conf.boxMode=false;
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, conf, "Nimm2");


    old_p=Position(0,0,0);  //used further down for summing up path
    vehicle->place(Pos(0,0,0.25));


    // create pointer to controller
    // push controller in global list of configurables

// controller is now initialized in writeControleerParamsToFile() to be able to write Controller params to file there
      //controller = new InvertNChannelController(10);
      //controller->setParam("eps",0.0);
      //controller->setParam("eps",0.1);
    global.configs.push_back(controller);

    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    //agent->setTrackOptions(TrackRobot(true, false, false, false, "track" ,1));
    global.agents.push_back(agent);



    summed_path=0;


    //load controller params
    assert (controller->getSensorNumber()==1);
    assert (controller->getMotorNumber()==1);
    loadControllerParamsFromFile();

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

  // sum path of robot as indicator for activity
  double summed_path;
  Position old_p;

 /** optional additional callback function which is called every simulation step.
  Called between physical simulation step and drawing.
  @param draw indicates that objects are drawn in this timestep
  @param pause indicates that simulation is paused
  @param control indicates that robots have been controlled this timestep
   */
  void addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
  { if ((!pause) && (control)){
      OdeAgentList::iterator i=globalData.agents.begin();
      Position p = (*i)->getRobot()->getPosition();
      // summ up path traveleed by the robot
      double dx=p.x-old_p.x;
      double dy=p.y-old_p.y;
      old_p=p;
      summed_path+= sqrt(dx*dx + dy*dy);
// //Test:
//      FILE* file;
//      char filename[256];
//      sprintf(filename, "ent%g_C.log",0.25);
//      file = fopen(filename,"a");
//      fprintf(file, "%f ", globalData.time);
//      fprintf(file,"%g, %g %g \n", p.x, p.y, p.z);
//      fflush(file);
//      if(file) fclose(file);
    }

   if (globalData.sim_step>=30000) {
// all datapoints in one column
/*     FILE* file;
     char filename[256];
     sprintf(filename, "summed_path.log");
     file = fopen(filename,"a");
     fprintf(file, "%f ", globalData.time);
     fprintf(file,"%g %g %g %g %g %g    %g \n", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, summed_path);
     fflush(file);
     if(file) fclose(file);
     simulation_time_reached=true;*/


     FILE* file;
     char filename[256];
     sprintf(filename, "summed_path.log");
     file = fopen(filename,"a");
     fprintf(file,"%g  ", summed_path);
     fflush(file);
     if(file) fclose(file);
     simulation_time_reached=true;
   }

 }


//   vois setParams(double c, double h){
//
//   }






  void writeInvertNChannelControllerParamsToFile(double _c, double _a, double _h){
    FILE* file;
    char filename[256];
    sprintf(filename, "init_weights.tmp");
    file = fopen(filename,"w+");

    matrix::Matrix C;
    matrix::Matrix A;
    matrix::Matrix H;
    C.set(1,1);
    A.set(1,1);
    H.set(1,1);
    C.val(0,0)=_c;
    A.val(0,0)=_a;
    H.val(0,0)=_h;

    C.store(file);
    H.store(file);
    A.store(file);
    //AbstractController *fakeTempController = new InvertNChannelController(10);
    //fakeTempController->setParam("eps",0.0);
    //fakeTempController->Configurable::print(file,0);
    //if (fakeTempController) delete (fakeTempController);
    controller = new InvertNChannelController(10);
    controller -> init(/*sensornumber*/1, /*motornumber*/1);
    //controller -> setParam("eps",0.0);
    controller -> setParam("eps",0.1);
    controller -> Configurable::print(file,0);
    fflush(file);
    if(file) fclose(file);
  }

  void writeStaticControllerParamsToFile(double _c, double _h){
    FILE* file;
    char filename[256];
    sprintf(filename, "init_weights.tmp");
    file = fopen(filename,"w+");

    matrix::Matrix C;
    matrix::Matrix H;
    C.set(1,1);
    H.set(1,1);
    C.val(0,0)=_c;
    H.val(0,0)=_h;

    C.store(file);
    H.store(file);
    controller = new StaticController(10);
    controller -> init(/*sensornumber*/1, /*motornumber*/1);
    controller -> Configurable::print(file,0);
    fflush(file);
    if(file) fclose(file);

  }




  void loadControllerParamsFromFile(){
    // load controller parameters from file
    FILE* file;
    char filename[256];
    sprintf(filename, "init_weights.tmp");
    file = fopen(filename,"r");
    controller->restore(file);
    if(file) fclose(file);
  }


};

// class ThisTrackRobot : public TrackRobot{  // eine Idee, aber void addCallback (siehe oben sollte besser sein!)
//
//
// }



int main (int argc, char **argv)
{

  FILE* filen;
  char filename[256];
  sprintf(filename, "parameters.log");
  filen = fopen(filename,"a");
  fprintf(filen,"#c  h  a \n");
  fflush(filen);
  if(filen) fclose(filen);
  for (int i=0; i<10; i++){
    std::cout<<i<<". Runde beendet\n";
    for (int c=-15; c<16; c++){
      for (int h=-5; h<6; h++){
        // small random values for a and c when using InvertNChannelController
        //double c=(((double)rand() / RAND_MAX) - 0.5) * 2.0; //Wert zwischen -1 und 1, wird dann ja noch durch 10 geteilt
        //double a=(((double)rand() / RAND_MAX) - 0.5) * 2.0; //Wert zwischen -1 und 1, wird dann ja noch
        // a should have positive sign
        //while ( (a < 0) ) {
        //  a=(((double)rand() / RAND_MAX) - 0.5) * 2.0; //Wert zwischen -1 und 1, wird dann ja noch
        //}
//c=11;h=0;
 //       double a=10;
        ThisSim sim;
//        sim.writeInvertNChannelControllerParamsToFile(((double)c)/10.0, /*a* / 1.0 */((double)a)/10.0, ((double)h)/10.0);
        sim.writeStaticControllerParamsToFile(((double)c)/10.0, ((double)h)/10.0);
        sim.run(argc, argv);
        /*if (i==0)*/{
          FILE* filen;
          char filename[256];
          sprintf(filename, "parameters.log");
          filen = fopen(filename,"a");
          //StaticController:
          fprintf(filen,"%g   %g\n", ((double)c)/10.0, ((double)h)/10.0);
          //InvertNChannelController:
          //fprintf(filen,"%g   %g  %g\n", ((double)c)/10.0, ((double)h)/10.0, ((double)a)/10.0 );
          fflush(filen);
          if(filen) fclose(filen);
        }
      }
    }
    FILE* file;
    char filename[256];
    sprintf(filename, "summed_path.log");
    file = fopen(filename,"a");
    fprintf(file,"\n");
    fflush(file);
    if(file) fclose(file);
  }



/*  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;*/

}

