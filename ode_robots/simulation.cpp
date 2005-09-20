/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.32  2005-09-20 10:54:34  robot3
 *   camera module:
 *   -pressing key c now centers on focused robot
 *   -pressing key b now moves 5.0f behind the robot
 *   -tiny bugfixing (nullpointer crashes etc.)
 *
 *   Revision 1.31  2005/09/19 16:01:58  martius
 *   use dsSetSimulationTime
 *
 *   Revision 1.30  2005/09/13 15:36:38  martius
 *   disabled advanced modes
 *   new grabframe interface used
 *
 *   Revision 1.29  2005/09/13 13:26:00  martius
 *   random seed adjustable
 *   usage with -h
 *
 *   Revision 1.28  2005/09/11 15:16:41  martius
 *   fast frame capturing enabled
 *
 *   Revision 1.27  2005/09/02 17:18:15  martius
 *   camera modes changed
 *
 *   Revision 1.26  2005/08/25 07:36:16  fhesse
 *   unistd.h included to be able to use usleep
 *
 *   Revision 1.25  2005/08/23 11:39:31  robot1
 *   advancedFollowing mode integrated (for switching)
 *
 *   Revision 1.24  2005/08/22 20:34:22  martius
 *   Display robot name when looking at it.
 *   Init robot view always
 *
 *   Revision 1.23  2005/08/22 12:41:05  robot1
 *   advancedTV mode integrated (for switching)
 *
 *   Revision 1.22  2005/08/16 10:09:07  robot1
 *   tiny bugfixing
 *
 *   Revision 1.21  2005/08/12 12:43:57  robot1
 *   command for switching between agents implemented
 *
 *   Revision 1.20  2005/08/12 11:55:01  robot1
 *   camera module integrated
 *
 *   Revision 1.19  2005/08/03 20:34:39  martius
 *   basic random number initialisation
 *   contains returns index instead of bool
 *
 *   Revision 1.18  2005/07/29 10:22:47  martius
 *   drawInterval honored
 *   real time syncronisation
 *
 *   Revision 1.17  2005/07/27 13:23:16  martius
 *   new color and position construction
 *
 *   Revision 1.16  2005/07/21 12:18:43  fhesse
 *   window size 640x480
 *
 *   Revision 1.15  2005/07/18 08:35:21  martius
 *   drawcallback is additionalcallback now
 *
 *   Revision 1.14  2005/07/15 11:35:52  fhesse
 *   added parameter gravity
 *
 *   Revision 1.13  2005/07/13 08:39:21  robot8
 *   added the possibility to use an additional command function, which handels special Inputs if the ODE simulation window has the focus
 *
 *   Revision 1.12  2005/07/11 11:19:38  robot8
 *   adding the line, where the pointer to the additional draw function is set to the value of the parameter drawCallback
 *
 *   Revision 1.11  2005/07/08 10:14:05  martius
 *   added contains (helper for stringlist search)
 *
 *   Revision 1.10  2005/07/07 10:23:36  martius
 *   added user draw callback
 *
 *   Revision 1.9  2005/06/30 13:23:38  robot8
 *   completing the call of the dynamic collisionCallback-function for  standard collisions
 *
 *   Revision 1.8  2005/06/29 09:27:03  martius
 *   *** empty log message ***
 *
 *   Revision 1.7  2005/06/29 09:25:17  martius
 *   customized callback for collision
 *
 *   Revision 1.6  2005/06/22 15:39:49  fhesse
 *   path to textures
 *
 *   Revision 1.5  2005/06/20 10:03:26  fhesse
 *   collision treatment by agents included
 *
 *   Revision 1.4  2005/06/17 09:33:53  martius
 *   aligned values on showParams
 *
 *   Revision 1.3  2005/06/15 14:01:31  martius
 *   moved all general code from main to simulation
 *                                                                 *
 ***************************************************************************/
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#include <drawstuff/drawstuff.h>
#include "camera.h"
#include "grabframe.h"
using namespace std;
#include "simulation.h"


// ODE globals
dWorldID world;
dSpaceID space;
dJointGroupID contactgroup;
dGeomID ground;

int windowWidth=640;
int windowHeight=480;


double simulationTime = 0;
struct timeval realTime;
int nextLeakAnnounce=20;
int leakAnnCounter=1;

VideoStream videostream;

OdeConfig simulationConfig;
int sim_step = 0;
dsFunctions fn;
enum SimulationState { none, initialised, running, closed };
SimulationState state = none;

void (*configfunction)() = 0; // pointer to the config function of the user
void (*collisionCallback)(void* data, dGeomID o1, dGeomID o2) = 0;  // pointer to the user defined nearcallback function
void (*additionalCallback)(bool draw, bool pause) = 0;  // pointer to the user defined additional function
void (*commandFunction)(int key) =0;  // command function, set by user

// Object lists
ObstacleList obstacles;
AgentList agents;

// commandline functions see below
void cmd_handler_init();
bool control_c_pressed();
void cmd_begin_input();
void cmd_end_input();
void usercommand_handler(int key);  // handles the &command (int key) from simulation_init

void  processCmdLine(int argc, char** argv);

// simulation stuff
void simLoop ( int pause );
void nearCallback(void *data, dGeomID o1, dGeomID o2);

// drawing stuff for camera handling
CameraType camType = Static; // default is a non-moving and non-rotating camera
AbstractRobot* viewedRobot; // the robot who is viewed from the camera

void simulation_init(void (*start)(), void (*end)(), 
		     void (*config)(), void (*command)(int n)/* = 0 */,
		     void (*collCallback)(void* data,dGeomID o1, dGeomID o2)/* = 0 */,
		     void (*addCallback)(bool draw, bool pause)/* = 0 */){
  configfunction=config; // store config function for simLoop
  collisionCallback=collCallback; // store config function for simLoop
  additionalCallback = addCallback;
  /**************************Grafikabschnitt**********************/
  fn.version = DS_VERSION;
  fn.start = start;
  fn.step = &simLoop;
  commandFunction = command; //  userfunction is stored to be executed if needed, see usercommand_handler()
  fn.command = usercommand_handler; // controlled by usercommand_handler now
  fn.stop = end;
  fn.path_to_textures = "../../textures";

  /***************************ODE-Abschnitt***********************/
  //****************Weltdefinitionsabschnitt**************
  //Anlegen der Welt an sich
  world = dWorldCreate ();

  //Anlegen eines Raumes der Welt in der Sichtbare Koerper 
  // eine raeumliche Ausdehnung annehmen koennen
  // ist fuer die Kollissionserkennung wichtig
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate ( 1000000 );
 
  //Gravitation zu Erdgravitation
  dWorldSetGravity ( world , 0 , 0 , simulationConfig.gravity );
  dWorldSetERP ( world , 1 );
  ground = dCreatePlane ( space , 0 , 0 , 1 , 0 );
  cmd_handler_init();
  state=initialised;
}

void camera_init(CameraType type, AbstractRobot* robot) {
  // setting only the parameters
  camType=type;
  viewedRobot=robot;
}


void simulation_start(int argc, char** argv){
  if(state!=initialised) return;  
  processCmdLine(argc , argv);
  
  // information on terminal, can be removed if the printout is undesired
  dsPrint("\nControl commands for the camera module:\n");
  dsPrint("---------------------------------------\n");
  dsPrint("   v     : switches between the camera modes (Static, TV, Following)\n");
  dsPrint("   Space : switches between the agents for view\n\n");

  //********************Simmulationsstart*****************

  state=running;
  gettimeofday(&realTime, 0);
  dsSimulationLoop ( argc , argv , windowWidth , windowHeight , &fn );
}

void simulation_close(){
  if(state!=running) return;
  //******Speicherfreigabe, Welt- und Raumzerstoerung*****
  dJointGroupDestroy ( contactgroup );
  dWorldDestroy ( world );
  dSpaceDestroy ( space );
  dCloseODE ();

  state=closed;
}

//Schleife der Simulation
void simLoop ( int pause ){
  // we run the physical simulation as often the drawinterval,
  //  because "drawstuff" will draw the world before this function is called
  //  the drawing of all object should occur if t==0
  for(int t = 0; t < simulationConfig.drawInterval; t++){
    // Parametereingabe  
    if (control_c_pressed()){
      cmd_begin_input();
      if(configfunction) configfunction();
      cmd_end_input();
      gettimeofday(&realTime, 0);
    }

    // the simulation is just run if pause is not enabled
    if (!pause) {
      //**************************Steuerungsabschnitt ************************
      simulationTime += simulationConfig.simStepSize;
    
      sim_step++;
      if ( (sim_step % simulationConfig.controlInterval ) == 0 ){
	for(AgentList::iterator i=agents.begin(); i != agents.end(); i++){
	  (*i)->step(simulationConfig.noise);
	}
      }
  
      /**********************Simulationsschritt an sich**********************/
      dSpaceCollide ( space , 0 , &nearCallback );
      dWorldStep ( world , simulationConfig.simStepSize ); //ODE-Engine geht einen Schritt weiter
      dJointGroupEmpty (contactgroup);    
    }  
    if(additionalCallback) additionalCallback(t==0, pause);

    if(t==0 || pause){
      /**************************Draw the scene ***********************/
      // first repositionize the camera if needed
      if (viewedRobot)
      	moveCamera(camType, *viewedRobot);
      dsSetSimulationTime(simulationTime);
      for(ObstacleList::iterator i=obstacles.begin(); i != obstacles.end(); i++){
	(*i)->draw();
      }
      for(AgentList::iterator i=agents.begin(); i != agents.end(); i++){
	(*i)->getRobot()->draw();
      }
      // grab frame if in captureing mode
      if(videostream.opened && !pause){
	grabAndWriteFrame(videostream);
      }
    }

    // Time syncronisation of real time and simulations time (not if on capture mode)
    if(simulationConfig.realTimeFactor!=0.0 && !videostream.opened){
      struct timeval currentTime;
      gettimeofday(&currentTime, 0);
      // difference in milliseconds
      long diff = (currentTime.tv_sec-realTime.tv_sec)*1000 + (currentTime.tv_usec-realTime.tv_usec)/1000;
      diff -= long(simulationConfig.simStepSize*1000.0/simulationConfig.realTimeFactor); 
      if(diff < -3){ // if less the 3 milliseconds we don't call usleep since it needs time
	usleep(min(100l,-diff-2)*1000);
	nextLeakAnnounce=max(20,nextLeakAnnounce/2);
      }else if (diff > 0){
	if(leakAnnCounter%nextLeakAnnounce==0){ // we do not bother the user all the time
	  printf("Time leak of %li ms (Please increase realTimeFactor)\n", diff);
	  nextLeakAnnounce*=2;
	  leakAnnCounter=0;
	}
	leakAnnCounter++;
      }
      gettimeofday(&realTime, 0);
    }
  }
}

// Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam
// 
void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  bool collision_treated=false;
  // call robots collision treatments
  for(AgentList::iterator i=agents.begin(); i != agents.end() && !collision_treated; i++){
    collision_treated=(*i)->getRobot()->collisionCallback(data, o1, o2);
  }
  
  if (collision_treated) return; // exit if collision was treated by a robot
  
  if(collisionCallback) { // calling user defined collision callback if it exists
    collisionCallback(data,o1,o2);
  }else{                  // using standard collision treatment

    int i,n;  
    const int N = 40;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    if (n > 0) {
      for (i=0; i<n; i++)
	{
	  contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	    dContactSoftERP | dContactSoftCFM | dContactApprox1;
	  contact[i].surface.mu = 0.8; //normale Reibung von Reifen auf Asphalt
	  contact[i].surface.slip1 = 0.005;
	  contact[i].surface.slip2 = 0.005;
	  contact[i].surface.soft_erp = 1;
	  contact[i].surface.soft_cfm = 0.00001;
	  dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	
	}
    }
  }
}

// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}


// Commandline interface stuff
void showParams(const ConfigList& configs)
{
  paramkey* keys;
  paramval* vals;
  const unsigned short spacelength=20;
  char spacer[spacelength];
  memset(spacer, ' ', spacelength);  spacer[spacelength-1]=0;

  for(ConfigList::const_iterator i=configs.begin(); i != configs.end(); i++){
    int pnum = (*i)->getParamList(keys,vals);
    printf("Parameters of %s\n", (*i)->getName());
    for(int j=0; j < pnum; j++) {
      printf(" %s=%s%f\n", keys[j], 
	     spacer+(strlen(keys[j]) > spacelength  ? spacelength : strlen(keys[j])),vals[j]);
    }
    free(keys);
    free(vals);
  }
}

void changeParams(ConfigList& configs){
  char buffer[1024];
  std::cout << "Type: Parameter=Value\n";
  fgets( buffer, 1024, stdin);
  if ( strchr(buffer,'?')!=0){
    showParams(configs);
    return;
  }

  char *p = strchr(buffer,'=');
  if (p){
    *p=0; // terminate key string 
    double v=strtod(p+1,0);
    for(ConfigList::iterator i=configs.begin(); i != configs.end(); i++){
      if ((*i)->setParam(buffer,v))
	printf(" %s=\t%f \n", buffer, (*i)->getParam(buffer));
    }
  }
}

void usage(const char* progname){
  printf("Parameter: %s [-r SEED] [-x WxH] [-pause] [-notex] [-noshadow]\n", progname);
  printf("\t-r SEED\t\tuse SEED as random number seed\n");
  printf("\t-x WxH\t\twindow size of width(W) x height(H) is used (640x480 default)\n");
  printf("\t-pause \t\tstart in pause mode\n");
  printf("\t-notex \t\tdo not display textures\n");
  printf("\t-noshadow\tdo not display shadows\n");
  exit(0);
}


void  processCmdLine(int argc, char** argv){
  if(contains(argv, argc, "-h")) usage(argv[0]);

  int seedIndex = contains(argv, argc, "-r");
  long seed=0;
  // initialize random number generator
  if(seedIndex && argc > seedIndex) {
    seed=atoi(argv[seedIndex]);
  }else{
    seed=time(0);
  }
  printf("Use random number seed: %li\n", seed);
  srand(seed);

  int resolindex = contains(argv, argc, "-x");
  int w,h;
  // initialize random number generator
  if(resolindex && argc > resolindex) {
    if(sscanf(argv[resolindex],"%ix%i", &w,&h) ==2){
      if(w>64 && h>64){
	windowWidth = w;
	windowHeight = h;	
      }
    }
  }
}

/// internals
int Control_C=0;

void control_c(int i){
  Control_C++ ;
  // if (Control_C > 100)exit(0);
}

void cmd_handler_exit(void){
  signal(SIGINT,SIG_DFL);
  Control_C=0;
}

void cmd_handler_init(){
  signal(SIGINT,control_c);
  atexit(cmd_handler_exit);
}

bool control_c_pressed(){
  return Control_C!=0;
}

void cmd_begin_input(){
  cmd_handler_exit();
}

void cmd_end_input(){
  cmd_handler_init();  
}

void initViewedRobot() {
  // setting the robot for view
  if (!viewedRobot) {
    AgentList::iterator i=agents.begin();
    viewedRobot=(*i)->getRobot();
  }
}

void usercommand_handler(int key) {
  // the stuff for handling internal commands
  switch (key) {
  case ' ': // key 32 (space) is for switching between the robots
    initViewedRobot();
    for(AgentList::iterator i=agents.begin(); i != agents.end(); i++){
      if (viewedRobot==(*i)->getRobot()) { // our current agent is found
	if (i!=agents.end()-1) {
	  viewedRobot=(*(i+1))->getRobot(); // take the next robot
	}
	else {
	  AgentList::iterator j=agents.begin();
	  viewedRobot=(*j)->getRobot();
	}
	break;
      }
    }
    printf("View at robot: %s\n", viewedRobot->getName());
    break;
  case 'v': // is for switching between the camera modes
    initViewedRobot();    
    switch (camType) {
    case Static: // now has to be TV
      // initializes the robot to view
      camType = TV;
      break;
    case TV: // now has to be advancedTV
      camType = Following;
      break;
    case Following: // now has to be Following
//       camType = advancedTV;
//       break;
//     case advancedTV:
//       camType = advancedFollowing;
//       break;
//     case advancedFollowing: // now has to be Static
      camType = Static;
      break;
    }
    break;
  case 'g': // toggle video capture mode
    if (videostream.opened){
      printf("Stop capturing mode\n");
      closeVideoStream(videostream);
    }else{
      time_t t = time(NULL);
      struct tm* tm = gmtime(&t);
      char name[129];
      sprintf(name,"frames/frame_%02i_%02i_%02i", tm->tm_hour,tm->tm_min,tm->tm_sec);
      printf("Start capturing to %s\n",name);
      system("mkdir -p frames");
      videostream = openVideoStream(name);
    }
    break;
  case 'c': // move camera to robot position
    if (viewedRobot) {
      moveOnRobot(*viewedRobot);
    }
    break;
  case 'b': // move camera behind robot movement
    if (viewedRobot) {
      moveBehindRobot(*viewedRobot);
    }
    break;
  default: // now call the user command
    if (commandFunction) commandFunction(key);
    break;
  }
}

