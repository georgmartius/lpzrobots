#define dDouble

#include <stdio.h>
#include <math.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>


#include "world.h"
//#include "simulation.h"

#include "component.h"
#include "matrices.h"
#include "cubic_spline.h"


#include "snakebot.h"

#include "agent.h"
#include "one2onewiring.h"
#include "invertnchannelcontroller.h"


//dadurch wird mit den Double-Genauigkeitszeichenmethoden gearbeitet

using namespace university_of_leipzig::robot;

#ifdef dDOUBLE
//#define dsSetViewPoint dsSetViewPointD
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif


// these are the only global variables needed
// (since the simulation functions do not support passing
// user values as arguments... grrr)
World *p_world;

AbstractWiring     *p_wiring;
Agent              *p_agent;
AbstractController *p_controller;

/*
ConfigList configs;
PlotMode plotMode = NoPlot;
*/


void simulation_init()
{

}


/**
 * simulation_start
 *
 *
 */
void simulation_start()
{
  // oops, dsSetViewpoint requires a float vector
  // note that the vector classes support implicit conversion to
  // a pointer to their data
  float a[] = {0.0, 0.0, 10.0};
  float b[] = {0.0, 0.0, 0.0};
  dsSetViewpoint(a, b);

  dsSetSphereQuality(2);

  dsPrint("Welcome to the virtual robot arm simulator\n");
  //Print("Press 'h' for help\n");
  dsPrint("Unfinished version - no commands available\n");
  dsPrint("so currently you can just watch and have fun! ;-)\n");

  //showParams(configs);
}

/*
void end(){
  for(ObstacleList::iterator i=obstacles.begin(); i != obstacles.end(); i++){
    delete (*i);
  }
  obstacles.clear();
  for(Agentist::iterator i=agents.begin(); i != agents.end(); i++){
    delete (*i)->getRobot();
    delete (*i)->getController();
    delete (*i);
  }
  agents.clear();
}
*/


/**
 * simulation_loop
 *
 *
 */
void simulation_loop(int pause)
{
  // draw the scene
  p_world->draw();

  if(pause)
    return;

  p_agent->step(0.1);
  p_world->step(0.01);


}


// this function is called if the user pressed Ctrl-C
/*
void config(){
  changeParams(configs);
}

void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile", progname);
  exit(0);
}
*/

int main(int argc, char *argv[])
{
  std::cout << "here\n";

  WorldDescription ewd;
  World world(ewd);


  p_world = &world;

  // set up the vertex list for the knot 
  VertexList vl;
  university_of_leipzig::robot::Matrix<double> mat(4, 9);

  for(unsigned i = 0; i < 9; ++i)
    mat(0, i) = i;

  mat(1, 0) =  2.5;
  mat(1, 1) =  2.5;
  mat(1, 2) =  7.5;
  mat(1, 3) = 10.0;
  mat(1, 4) =  5.5;
  mat(1, 5) =  0.0;
  mat(1, 6) =  2.5;
  mat(1, 7) =  7.5;
  mat(1, 8) =  7.5;

  mat(2, 0) =  -5.0;
  mat(2, 1) =   0.0;
  mat(2, 2) =   2.5;
  mat(2, 3) =   0.0;
  mat(2, 4) =  -2.5;
  mat(2, 5) =   0.0;
  mat(2, 6) =  1.25;
  mat(2, 7) =   2.5;
  mat(2, 8) =  10.0;

  mat(3, 0) =  2.5;
  mat(3, 1) =  2.5;
  mat(3, 2) =  5.0;
  mat(3, 3) =  2.5;
  mat(3, 4) =  0.5;
  mat(3, 5) =  2.5;
  mat(3, 6) =  5.0;
  mat(3, 7) =  2.5;
  mat(3, 8) =  2.5;

  double fx = 0.2;
  double fy = 0.2;
  double fz = 0.2;
  for(unsigned i = 0; i < 9; ++i) {
    mat(1, i) *= fx;
    mat(2, i) *= fy;
    mat(3, i) *= fz;
  }



  CubicSpline<dReal> cs;
  cs.create(mat);

  double p = 0.0;
  while(p < 8.0) {
    Vector<dReal> v = cs.get_point(p);


    Vector3<dReal> v3(v(0), v(1), v(2));
    vl.insert(vl.end(), v3);
   

    p = cs.get_distant_point_parameter(p, 0.9);
  }
  /*
    // create a spiral snake (or something)
    for(unsigned i = 0; i < 5; ++i) {
    vl.insert(vl.end(), Vector3<double>(i / 3 * cos(PI / 180 * i * 5),
                                        i / 3 * sin(PI / 180 * i * 5),
                                        i));
  }
  */
    // create a streched snake
    /*
    for(unsigned i = 0; i < 3; ++i) {
    vl.insert(vl.end(), Vector3<double>(i * 0.7,
					0.0,
					0.1));
					}
    */
 
  // set up a robot arm description
  RobotArmDescription desc;
  desc.p_world = &world;
  desc.segment_radius   = 0.05;
  desc.segment_mass   = 1.0;
  desc.p_vertex_list = &vl;


  // creates a snakebot in the specified world (the p_world attribute of the
  // description)
  // note that a component removes itself from a world when it gets destroyed
  //  CCURobotArmComponent rac(desc);
  SnakeBot snake_bot(desc);

  PlaneComponentDescription pcd;
  pcd.p_world   = &world;
  pcd.v3_normal = Vector3<dReal>(0.0, 0.0, 1.0);
  pcd.d         = 0.0;

  PlaneComponent pc(pcd);
  //world.register_component(pc);


  // initialization
  //  simulationConfig.noise=0.1;
  //  configs.push_back(&simulationConfig);


  dsFunctions fn;
  memset(&fn, 0, sizeof(fn));
  fn.version          = DS_VERSION;
  fn.start            = &simulation_start;
  fn.step             = &simulation_loop;
  fn.command          = NULL;
  fn.stop             = NULL;
  fn.path_to_textures = "../../textures";



  p_controller = new InvertNChannelController(10);
  p_wiring     = new One2OneWiring(new ColorUniformNoise());
  p_agent      = new Agent();

  p_agent->init(p_controller, &snake_bot, p_wiring);

  /*
    agents.push_back(p_agent);
    configs.push_back(p_controller);
  */


  //simulation_init(&start, &end, &config);
  // start the simulation (returns, if the user closes the simulation)
  //  simulation_start(argc, argv);
  //  simulation_close();  // tidy up.


  dsSimulationLoop(argc, argv, 500, 500, &fn);

  /*
  delete p_agent;
  delete p_wiring;
  delete p_controller;
  */
  dCloseODE ();

  return 0;
}

