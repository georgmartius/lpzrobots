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
 *   Revision 1.9  2005-11-09 14:40:13  fhesse
 *   contr. adapted for distr.
 *
 *   Revision 1.8  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <vector>

#include "component_to_robot.h"
#include "noisegenerator.h"
#include "simulation.h"
#include "agent.h"
#include "one2onewiring.h"
#include "playground.h"


#include "sinecontroller.h"
//#include "invertnchannelcontroller.h"


using namespace university_of_leipzig::robots;

PlotMode plotMode = NoPlot;

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.noise=0.1;
  /*  
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(7.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  */

  // set up the vertex list for the knot 
  VertexList vl;
  university_of_leipzig::robots::Matrix<double> mat(4, 12);

  for(unsigned i = 0; i < 12; ++i)
    mat(0, i) = i;

  double r = 0.3;
  double r2 = 0.15;
  double d = 0.5;
  double l = 1.0;
  double x = 0.7;

  double tail = 2.5;

  mat(1,  0) = 0.0;
  mat(1,  1) = 0.0;

  mat(1,  2) = d / 2.0;

  mat(1,  3) = d;
  mat(1,  4) = d + r;
  mat(1,  5) = d;
  mat(1,  6) = 0.0;
  mat(1,  7) = -r;
  mat(1,  8) = 0.0;

  mat(1,  9) = d;

  mat(1, 10) = d / 2.0;
  mat(1, 11) = d;

  double u = (l - x) / 4.0;
  double c = (l + x) / 2.0;
  mat(2,  0) =  -tail;
  mat(2,  1) =  0.0;

  mat(2,  2) =  2.0 * u + 2.0 / 3.0 * x;

  mat(2,  3) =  c;
  mat(2,  4) =  c + u;
  mat(2,  5) =  c;
  mat(2,  6) =  2.0 * u;
  mat(2,  7) =  u;
  mat(2,  8) =  2.0 * u;

  mat(2,  9) =  2.0 * u + 1.0 / 3.0 * x;

  mat(2, 10) =  l;
  mat(2, 11) =  l + tail;



  mat(3,  0) =  0.0;
  mat(3,  1) =  0.0;

  mat(3,  2) =  r / 2.0;

  mat(3,  3) =  r;
  mat(3,  4) =  0.0;
  mat(3,  5) =  -r2;
  mat(3,  6) =  -r2;
  mat(3,  7) =  0.0;
  mat(3,  8) =  r;

  mat(3,  9) =  r / 2.0;

  mat(3, 10) =  0.0;
  mat(3, 11) =  0.0;



  double a_p[3] = {0.0, 0.0, 5.0};
  /*  double fx = 0.2;
  double fy = 0.2;
  double fz = 0.2; */
  for(unsigned i = 0; i < 3; ++i)
    for(unsigned j = 0; j < 12; ++j)
      mat(i + 1, j) += a_p[i];
 

  CubicSpline<dReal> cs;
  cs.create(mat);

  double p = 0.0;
  while(p < 11.0) {
    Vector<dReal> v = cs.get_point(p);


    Vector3<dReal> v3(v(0), v(1), v(2));
    vl.insert(vl.end(), v3);
   

    p = cs.get_distant_point_parameter(p, 0.4);
  }
  
  // create a spiral/circle snake (or something)
  
    for(unsigned i = 0; i < 15; ++i) {
      //     vl.insert(vl.end(), Vector3<double>(i / 3 * cos(M_PI / 180 * i * 5), i / 3 * sin(M_PI / 180 * i * 5), i));
    //vl.insert(vl.end(), Vector3<double>(3 * cos(M_PI / 180 * i * 25), 3 * sin(M_PI / 180 * i * 25), 0.5));
    }
  
  
  // create a streched snake  
  /*
  for(unsigned i = 0; i < 5 ; ++i) {
    vl.insert(vl.end(), Vector3<double>(0.0, i * 0.7, 0.1));
    // vl.insert(vl.end(), Vector3<double>(i * 0.7, 0.0, 0.1));
  }
  */

  // set up a robot arm description
  RobotArmDescription desc;
  desc.p_ode_handle   = new OdeHandle(odeHandle);
  desc.segment_radius = 0.05;
  desc.segment_mass   = 1.0;
  desc.p_vertex_list  = &vl;

  //  CCURobotArmComponent rac(desc);
  // IComponent *p_component = new Test();
  IComponent    *p_component = new CCURobotArmComponent(desc);
  AbstractRobot *p_robot     = new ComponentToRobot(p_component, odeHandle);


  // initialization
  //  global.odeConfig.noise=0.1;
  //  
  AbstractController *controller = new SineController(); //(10);
  controller->setParam("epsC", 0.001);
  controller->setParam("epsA", 0.001);

  // AbstractController *controller = new SineController();
  AbstractWiring* wiring     = new One2OneWiring(new ColorUniformNoise());
  Agent* agent               = new Agent();


  agent->init(controller, p_robot, wiring);  
  global.agents.push_back(agent);
  
  global.configs.push_back(controller);

  global.configs.push_back(p_component);

  showParams(global.configs);
}

void end(GlobalData& global){
   for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++){
     delete (*i);
   }
   global.obstacles.clear();
   
   for(AgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++){
     delete (*i)->getRobot();
     delete (*i)->getController(); 
     delete (*i);
   }
   global.agents.clear();
   
}


// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){
  changeParams(global.configs);
}

void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile", progname);
}



int main (int argc, char **argv)
{  
  if(contains(argv, argc, "-g")) plotMode = GuiLogger;
  if(contains(argv, argc, "-l")) plotMode = GuiLogger_File;
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config);

  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
