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

#include "odehandle.h"

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
  global.odeConfig.setParam("noise", 0.1);
  global.odeConfig.setParam("gravity", -9.8);

 
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(7.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
 
  global.obstacles.push_back(playground);



  AngleList al;

  al.insert(al.end(), Angle(  0.0 * M_PI / 180.0, 0.0, 0.0));
  std::cout << "here\n";

  
  al.insert(al.end(), Angle( 90.0 * M_PI / 180.0, 0.0, 0.0));
  al.insert(al.end(), Angle(180.0 * M_PI / 180.0, 0.0, 0.0));
  al.insert(al.end(), Angle(270.0 * M_PI / 180.0, 0.0, 0.0));

  // set up a robot arm description
  SpiderDescription desc;
  desc.p_ode_handle   = new OdeHandle(odeHandle);
  desc.sphere_radius  = 0.5;
  desc.sphere_mass    = 1.0;
  desc.segment_radius = 0.05;
  desc.segment_mass   = 1.0;
  desc.segment_length = 1.0;
  desc.segment_count  = 2;
  desc.p_angle_list   = &al;
  desc.position       = Vertex(0.0, 0.0, 5.0);
  std::cout << "M1\n";

  //  CCURobotArmComponent rac(desc);
  // IComponent *p_component = new Test();
  IComponent    *p_component = new SpiderComponent(desc);
  std::cout << "M2\n";

  AbstractRobot *p_robot     = new ComponentToRobot(p_component, odeHandle);

  std::cout << "M3\n";

  WireContainer vc;

  std::cout << p_component->get_sub_component_count() << "\n";
  std::cout << p_component->expose_wires(vc) << "\n";


  // initialization
  global.odeConfig.noise=0.1;
  //  configs.push_back(&simulationConfig);

  AbstractController *controller = new SineController();

  //controller->setParam("sineRate", 10000.0);

  // AbstractController *controller = new SineController();
  AbstractWiring* wiring     = new One2OneWiring(new ColorUniformNoise());
  Agent* agent               = new Agent();


  agent->init(controller, p_robot, wiring);  
  global.agents.push_back(agent);
  
  //  global.configs.push_back(&simulationConfig);
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
 

