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
 *   Revision 1.40.4.5  2005-12-09 16:53:16  martius
 *   camera is working now
 *
 *   Revision 1.40.4.4  2005/12/06 17:38:13  martius
 *   *** empty log message ***
 *
 *   Revision 1.40.4.3  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.40.4.2  2005/11/15 12:29:14  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.40.4.1  2005/11/14 17:37:00  martius
 *   changed makefile structure to have and include directory
 *   mode to selforg
 *
 *   Revision 1.40  2005/11/10 09:20:48  martius
 *   10 minute notification
 *
 *   Revision 1.39  2005/10/21 11:59:58  martius
 *   realtimefactor written small
 *
 *   Revision 1.38  2005/10/18 15:31:32  fhesse
 *   one comment improved
 *
 *   Revision 1.37  2005/10/06 17:11:26  martius
 *   switched to stl lists
 *
 *   Revision 1.36  2005/09/27 13:59:03  martius
 *   doInternals after control
 *
 *   Revision 1.35  2005/09/23 09:55:16  martius
 *   odeConfig gets OdeHandle explicit
 *
 *   Revision 1.34  2005/09/22 13:17:11  martius
 *   OdeHandle and GlobalData finished
 *   doInternalStuff included
 *
 *   Revision 1.33  2005/09/22 11:21:57  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.32  2005/09/20 10:54:34  robot3
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
#include <selforg/configurable.h>

#include "simulation.h"

#include <osgProducer/Viewer>
#include <osg/ArgumentParser>

#include "odeagent.h"

#include "camera.h"
#include "grabframe.h"

#include "abstractobstacle.h"
#include "cameramanipulator.h"

namespace lpzrobots {

  using namespace std;
  using namespace osg;
  using namespace osgProducer;
  

  int Simulation::ctrl_C = 0;

  Simulation::Simulation(){
    nextLeakAnnounce = 20;
    leakAnnCounter = 1;
    sim_step = 0;
    state = none;
    camType = Static;
  }

  Simulation::~Simulation(){  
    if(state!=running) return;
    dJointGroupDestroy ( odeHandle.jointGroup );
    dWorldDestroy ( odeHandle.world );
    dSpaceDestroy ( odeHandle.space );
    dCloseODE ();
  
    state=closed;
  }

  bool Simulation::init(int argc, char** argv){

    /**************** ODE-Section   ***********************/
    odeHandle.world = dWorldCreate ();

    // Create the primary world-space, which is used for collision detection
    odeHandle.space = dHashSpaceCreate (0);
    // the jointGroup is used for collision handling, 
    //  where a lot of joints are created every step
    odeHandle.jointGroup = dJointGroupCreate ( 1000000 );
  
    globalData.odeConfig.setOdeHandle(odeHandle);
 
    //set Gravity to Earth level
    dWorldSetGravity ( odeHandle.world , 0 , 0 , globalData.odeConfig.gravity );
    dWorldSetERP ( odeHandle.world , 1 );

    cmd_handler_init();

    // add ode config to config list
    globalData.configs.push_back(&(globalData.odeConfig));

    /**************** OpenSceneGraph-Section   ***********************/
    processCmdLine(argc, argv);
    // use an ArgumentParser object to manage the program arguments.
    arguments = new ArgumentParser(&argc, argv);

    // set up the usage document, in case we need to print out how to use this program.
    arguments->getApplicationUsage()->setDescription(
                  arguments->getApplicationName() + " Lpzrobots Simulator");
    arguments->getApplicationUsage()->setCommandLineUsage(arguments->getApplicationName());
    arguments->getApplicationUsage()->addCommandLineOption(
		 "-h or --help", "Display this information");

    // construct the viewer.
    viewer = new Viewer(*arguments);

    // set up the value with sensible default event handlers.
    viewer->setUpViewer(osgProducer::Viewer::STANDARD_SETTINGS);

    // if user request help write it out to cout.
    if (arguments->read("-h") || arguments->read("--help")) {
      arguments->getApplicationUsage()->write(std::cout);
      return false;
    }
    // any option left unread are converted into errors to write out later.
    arguments->reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occured when parsing the program aguments.
    if (arguments->errors()) {
      arguments->writeErrorMessages(std::cout);
      return false;
    }

    osgHandle.tesselhints = new TessellationHints();
    if (!osgHandle.tesselhints) return false;
    osgHandle.tesselhints->setDetailRatio(2.0f);
    osgHandle.color = Color(1,1,1,1);

    osgHandle.scene=makeScene();
    if (!osgHandle.scene) return false;

    CameraManipulator* cameramanipulator = new CameraManipulator(osgHandle.scene);
    unsigned int pos = viewer->addCameraManipulator(cameramanipulator);
    viewer->selectCameraManipulator(pos);
    
    // get details on keyboard and mouse bindings used by the viewer.
    viewer->getUsage(*(arguments->getApplicationUsage()));


    state=initialised;
    return true;
  }


  bool Simulation::run(int argc, char** argv){
    if(!init(argc, argv)) return false;
  
    // information on terminal, can be removed if the printout is undesired
    printf("\nControl commands for the camera module:\n");
    printf("---------------------------------------\n");
    printf("   v     : switches between the camera modes (Static, TV, Following)\n");
    printf("   Space : switches between the agents for view\n\n");

    //********************Simmulationsstart*****************
    state=running;
    gettimeofday(&realTime, 0);
    
    start(odeHandle, osgHandle, globalData);  

    // add model to viewer.
    viewer->setSceneData(root); 

    // create the windows and run the threads.
    viewer->realize();

    while (!viewer->done())
      {
	// wait for all cull and draw threads to complete.
	viewer->sync();
           
	loop(false);

	// update the scene by traversing it with the the update visitor which will
	// call all node update callbacks and animations.
	viewer->update();      

      
	// fire off the cull and draw traversals of the scene.
	viewer->frame();
      }
    
    // wait for all cull and draw threads to complete before exit.
    viewer->sync();
    end(globalData);
    return true;

  }


  void Simulation::loop(bool pause){
    // we run the physical simulation as often as "drawinterval",
    //  the drawing of all object should occur if t==0  
    for(int t = 0; t < globalData.odeConfig.drawInterval; t++){
      // Parametereingabe  
      if (control_c_pressed()){
	cmd_begin_input();
	config(globalData);
	cmd_end_input();
	gettimeofday(&realTime, 0);
      }

      // the simulation is just run if pause is not enabled
      if (!pause) {
	//**************************Steuerungsabschnitt ************************
	globalData.time += globalData.odeConfig.simStepSize;      
	sim_step++;
	// print simulation time every 10 min.
	if(sim_step% ( int(1/globalData.odeConfig.simStepSize) * 600) ==0) {
	  printf("Simulation time: %li min\n", sim_step/ ( long(1/globalData.odeConfig.simStepSize)*60));
	}
	// for all agents: robots internal stuff and control step if at controlInterval
	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++){
	  if ( (sim_step % globalData.odeConfig.controlInterval ) == 0 ){
	    (*i)->step(globalData.odeConfig.noise); 
	  }
	  (*i)->getRobot()->doInternalStuff(globalData);
	}
  
	/**********************Simulationsschritt an sich**********************/
	dSpaceCollide ( odeHandle.space , this , &nearCallback );
	dWorldStep ( odeHandle.world , globalData.odeConfig.simStepSize ); 
	//ODE-Engine geht einen Schritt weiter
	dJointGroupEmpty (odeHandle.jointGroup);    
      }  
    
      addCallback(globalData, t==0, pause);

      if(t==0){
	/**************************Draw the scene ***********************/
	// first repositionize the camera if needed
	if (viewedRobot)
	  // moveCamera(camType, *viewedRobot);
	  for(ObstacleList::iterator i=globalData.obstacles.begin(); i != globalData.obstacles.end(); i++){
	    (*i)->update();
	  }
	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++){
	  (*i)->getRobot()->update();
	}
	// grab frame if in captureing mode
	if(videostream.opened && !pause){
	  grabAndWriteFrame(videostream);
	}
      }

      // Time syncronisation of real time and simulations time (not if on capture mode)
      if(globalData.odeConfig.realTimeFactor!=0.0 && !videostream.opened){
	struct timeval currentTime;
	gettimeofday(&currentTime, 0);
	// difference in milliseconds
	long diff = (currentTime.tv_sec-realTime.tv_sec)*1000 + (currentTime.tv_usec-realTime.tv_usec)/1000;
	diff -= long(globalData.odeConfig.simStepSize*1000.0/globalData.odeConfig.realTimeFactor); 
	if(diff < -3){ // if less the 3 milliseconds we don't call usleep since it needs time
	  usleep(min(100l,-diff-2)*1000);
	  nextLeakAnnounce=max(200,nextLeakAnnounce/2);
	}else if (diff > 0){
	  if(leakAnnCounter%nextLeakAnnounce==0 && diff > 10){ // we do not bother the user all the time
	    printf("Time leak of %li ms (Please increase realtimefactor)\n", diff);
	    nextLeakAnnounce*=2;
	    leakAnnCounter=0;
	  }
	  leakAnnCounter++;
	}
	gettimeofday(&realTime, 0);
      }
    }
  }


  void Simulation::processCmdLine(int argc, char** argv){
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

    //   int resolindex = contains(argv, argc, "-x");
    //   int w,h;
    //   // initialize random number generator
    //   if(resolindex && argc > resolindex) {
    //     if(sscanf(argv[resolindex],"%ix%i", &w,&h) ==2){
    //       if(w>64 && h>64){
    // 	windowWidth = w;
    // 	windowHeight = h;	
    //       }
    //     }
    //   }
  }

  // Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam
  // 
  void Simulation::nearCallback(void *data, dGeomID o1, dGeomID o2){    
    Simulation* me = (Simulation*) data;
    if (!me) return;

    bool collision_treated=false;
    // call robots collision treatments
    for(OdeAgentList::iterator i= me->globalData.agents.begin(); 
	i != me->globalData.agents.end() && !collision_treated; i++){
      collision_treated=(*i)->getRobot()->collisionCallback(data, o1, o2);
    }
  
    if (collision_treated) return; // exit if collision was treated by a robot
  
    if(!(me->collCallback(me->odeHandle, data,o1,o2))){
      // using standard collision treatment

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
	    dJointID c = dJointCreateContact (me->odeHandle.world, 
					      me->odeHandle.jointGroup,&contact[i]);
	    dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	
	  }
      }
    }
  }

  /// internals

  void Simulation::control_c(int i){
    ctrl_C++ ;
    // if (Control_C > 100)exit(0);
  }

  void Simulation::cmd_handler_exit(void){
    signal(SIGINT,SIG_DFL);
    ctrl_C=0;
  }

  void Simulation::cmd_handler_init(){
    signal(SIGINT,control_c);
    atexit(cmd_handler_exit);
  }

  bool Simulation::control_c_pressed(){
    return ctrl_C!=0;
  }

  void Simulation::cmd_begin_input(){
    cmd_handler_exit();
  }

  void Simulation::cmd_end_input(){
    cmd_handler_init();  
  }


  // Helper
  int contains(char **list, int len,  const char *str){
    for(int i=0; i<len; i++){
      if(strcmp(list[i],str) == 0) return i+1;
    }
    return 0;
  }

  void Simulation::usage(const char* progname){
    printf("Parameter: %s [-r SEED] [-x WxH] [-pause] [-notex] [-noshadow]\n", progname);
    printf("\t-r SEED\t\tuse SEED as random number seed\n");
    printf("\t-x WxH\t\twindow size of width(W) x height(H) is used (640x480 default)\n");
    printf("\t-pause \t\tstart in pause mode\n");
    printf("\t-notex \t\tdo not display textures\n");
    printf("\t-noshadow\tdo not display shadows\n");
    exit(0);
  }

  // Commandline interface stuff
  void showParams(const ConfigList& configs)
  {
    for(vector<Configurable*>::const_iterator i=configs.begin(); i != configs.end(); i++){
      (*i)->print(stdout, 0);
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

}


// /// user defined start function (called at the beginning of the simulation)
// void (*startFunction)(const OdeHandle&, GlobalData& globalData) = 0;
// /// user defined end function (called after the simulation)
// void (*endFunction)(GlobalData& globalData) =0; 
// /// pointer to the config function of the user
// void (*configFunction)(GlobalData& globalData) = 0 ;
// // command function, set by user
// void (*commandFunction)(const OdeHandle&, GlobalData& globalData, int key) = 0 ; 
// /// pointer to the user defined additional function
// void (*collisionCallback)(const OdeHandle&, void* data, dGeomID o1, dGeomID o2) = 0;
// /// pointer to the user defined additional function which is executed in each simulationstep
// void (*additionalCallback)(GlobalData& globalData, bool draw, bool pause) = 0;

// void _start();
// void _end();

// // commandline functions see below
// void cmd_handler_init();
// bool control_c_pressed();
// void cmd_begin_input();
// void cmd_end_input();
// void usercommand_handler(int key);  // handles the &command (int key) from simulation_init

// void  processCmdLine(int argc, char** argv);

// // simulation stuff
// void simLoop ( int pause );
// void nearCallback(void *data, dGeomID o1, dGeomID o2);

// // drawing stuff for camera handling
// CameraType camType = Static; // default is a non-moving and non-rotating camera
// OdeRobot* viewedRobot; // the robot who is viewed from the camera

// void simulation_init(void (*start)(const OdeHandle&, GlobalData& globalData), 
// 		     void (*end)(GlobalData& globalData), 
// 		     void (*config)(GlobalData& globalData), 
// 		     void (*command)(const OdeHandle&, GlobalData& globalData, int key) /* = 0*/, 
// 		     void (*collCallback)(const OdeHandle&, void* data, dGeomID o1, dGeomID o2) /*= 0*/,
// 		     void (*addCallback)(GlobalData& globalData, bool draw, bool pause) /*= 0*/) {

//   startFunction=start;    // store start function for simLoop
//   endFunction=end;       // store end function for simLoop
//   configFunction=config; // store config function for simLoop
//   collisionCallback=collCallback; // store config function for simLoop
//   additionalCallback = addCallback;
//   /**************************Grafikabschnitt**********************/
//   fn.version = DS_VERSION;
//   fn.start = _start;
//   fn.step = &simLoop;
//   commandFunction = command; //  userfunction is stored to be executed if needed, see usercommand_handler()
//   fn.command = usercommand_handler; // controlled by usercommand_handler now
//   fn.stop = _end;
//   fn.path_to_textures = "../../textures";

//   /***************************ODE-Abschnitt***********************/
//   //****************Weltdefinitionsabschnitt**************
//   //Anlegen der Welt an sich
//   odeHandle.world = dWorldCreate ();

//   //Anlegen eines Raumes der Welt in der Sichtbare Koerper 
//   // eine raeumliche Ausdehnung annehmen koennen
//   // ist fuer die Kollissionserkennung wichtig
//   odeHandle.space = dHashSpaceCreate (0);
//   odeHandle.jointGroup = dJointGroupCreate ( 1000000 );
//   globalData.odeConfig.setOdeHandle(odeHandle);
 
//   //Gravitation zu Erdgravitation
//   dWorldSetGravity ( odeHandle.world , 0 , 0 , globalData.odeConfig.gravity );
//   dWorldSetERP ( odeHandle.world , 1 );
//   ground = dCreatePlane ( odeHandle.space , 0 , 0 , 1 , 0 );
//   cmd_handler_init();

//   // add ode config to config list
//   globalData.configs.push_back(&(globalData.odeConfig));

//   state=initialised;
// }

// void camera_init(CameraType type, OdeRobot* robot) {
//   // setting only the parameters
//   camType=type;
//   viewedRobot=robot;
// }


// void simulation_start(int argc, char** argv){
//   if(state!=initialised) return;  
//   processCmdLine(argc , argv);
  
//   // information on terminal, can be removed if the printout is undesired
//   dsPrint("\nControl commands for the camera module:\n");
//   dsPrint("---------------------------------------\n");
//   dsPrint("   v     : switches between the camera modes (Static, TV, Following)\n");
//   dsPrint("   Space : switches between the agents for view\n\n");

//   //********************Simmulationsstart*****************

//   state=running;
//   gettimeofday(&realTime, 0);
//   dsSimulationLoop ( argc , argv , windowWidth , windowHeight , &fn );
// }

// void simulation_close(){
//   if(state!=running) return;
//   //******Speicherfreigabe, Welt- und Raumzerstoerung*****
//   dJointGroupDestroy ( odeHandle.jointGroup );
//   dWorldDestroy ( odeHandle.world );
//   dSpaceDestroy ( odeHandle.space );
//   dCloseODE ();

// state=closed;
// }

// void _start(){
//   if(startFunction) startFunction(odeHandle, globalData);
// }
// void _end(){
//   if(endFunction) endFunction(globalData);
// }

// //Schleife der Simulation
// void simLoop ( int pause ){
//   // we run the physical simulation as often the drawinterval,
//   //  because "drawstuff" will draw the world before this function is called
//   //  the drawing of all object should occur if t==0  
//   for(int t = 0; t < globalData.odeConfig.drawInterval; t++){
//     // Parametereingabe  
//     if (control_c_pressed()){
//       cmd_begin_input();
//       if(configFunction) configFunction(globalData);
//       cmd_end_input();
//       gettimeofday(&realTime, 0);
//     }

//     // the simulation is just run if pause is not enabled
//     if (!pause) {
//       //**************************Steuerungsabschnitt ************************
//       globalData.time += globalData.odeConfig.simStepSize;      
//       sim_step++;
//       // print simulation time every 10 min.
//       if(sim_step% ( int(1/globalData.odeConfig.simStepSize) * 600) ==0) {
// 	printf("Simulation time: %li min\n", sim_step/ ( long(1/globalData.odeConfig.simStepSize)*60));
//       }
//       // for all agents: robots internal stuff and control step if at controlInterval
//       for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++){
// 	if ( (sim_step % globalData.odeConfig.controlInterval ) == 0 ){
// 	  (*i)->step(globalData.odeConfig.noise); 
// 	}
// 	(*i)->getRobot()->doInternalStuff(globalData);
//       }
  
//       /**********************Simulationsschritt an sich**********************/
//       dSpaceCollide ( odeHandle.space , 0 , &nearCallback );
//       dWorldStep ( odeHandle.world , globalData.odeConfig.simStepSize ); //ODE-Engine geht einen Schritt weiter
//       dJointGroupEmpty (odeHandle.jointGroup);    
//     }  
//     if(additionalCallback) additionalCallback(globalData, t==0, pause);

//     if(t==0){
//       /**************************Draw the scene ***********************/
//       // first repositionize the camera if needed
//       if (viewedRobot)
//       	moveCamera(camType, *viewedRobot);
//       dsSetSimulationTime(globalData.time);
//       for(ObstacleList::iterator i=globalData.obstacles.begin(); i != globalData.obstacles.end(); i++){
// 	(*i)->draw();
//       }
//       for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++){
// 	(*i)->getRobot()->draw();
//       }
//       // grab frame if in captureing mode
//       if(videostream.opened && !pause){
// 	grabAndWriteFrame(videostream);
//       }
//     }

//     // Time syncronisation of real time and simulations time (not if on capture mode)
//     if(globalData.odeConfig.realTimeFactor!=0.0 && !videostream.opened){
//       struct timeval currentTime;
//       gettimeofday(&currentTime, 0);
//       // difference in milliseconds
//       long diff = (currentTime.tv_sec-realTime.tv_sec)*1000 + (currentTime.tv_usec-realTime.tv_usec)/1000;
//       diff -= long(globalData.odeConfig.simStepSize*1000.0/globalData.odeConfig.realTimeFactor); 
//       if(diff < -3){ // if less the 3 milliseconds we don't call usleep since it needs time
// 	usleep(min(100l,-diff-2)*1000);
// 	nextLeakAnnounce=max(200,nextLeakAnnounce/2);
//       }else if (diff > 0){
// 	if(leakAnnCounter%nextLeakAnnounce==0 && diff > 10){ // we do not bother the user all the time
// 	  printf("Time leak of %li ms (Please increase realtimefactor)\n", diff);
// 	  nextLeakAnnounce*=2;
// 	  leakAnnCounter=0;
// 	}
// 	leakAnnCounter++;
//       }
//       gettimeofday(&realTime, 0);
//     }
//   }
// }

// // Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam
// // 
// void nearCallback(void *data, dGeomID o1, dGeomID o2)
// {
//   bool collision_treated=false;
//   // call robots collision treatments
//   for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end() && !collision_treated; i++){
//     collision_treated=(*i)->getRobot()->collisionCallback(data, o1, o2);
//   }
  
//   if (collision_treated) return; // exit if collision was treated by a robot
  
//   if(collisionCallback) { // calling user defined collision callback if it exists
//     collisionCallback(odeHandle, data,o1,o2);
//   }else{                  // using standard collision treatment

//     int i,n;  
//     const int N = 40;
//     dContact contact[N];
//     n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
//     if (n > 0) {
//       for (i=0; i<n; i++)
// 	{
// 	  contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
// 	    dContactSoftERP | dContactSoftCFM | dContactApprox1;
// 	  contact[i].surface.mu = 0.8; //normale Reibung von Reifen auf Asphalt
// 	  contact[i].surface.slip1 = 0.005;
// 	  contact[i].surface.slip2 = 0.005;
// 	  contact[i].surface.soft_erp = 1;
// 	  contact[i].surface.soft_cfm = 0.00001;
// 	  dJointID c = dJointCreateContact (odeHandle.world,odeHandle.jointGroup,&contact[i]);
// 	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	
// 	}
//     }
//   }
// }

// // Helper
// int contains(char **list, int len,  const char *str){
//   for(int i=0; i<len; i++){
//     if(strcmp(list[i],str) == 0) return i+1;
//   }
//   return 0;
// }


// // Commandline interface stuff
// void showParams(const ConfigList& configs)
// {
//   for(vector<Configurable*>::const_iterator i=configs.begin(); i != configs.end(); i++){
//     (*i)->print(stdout, 0);
//   }
// }

// void changeParams(ConfigList& configs){
//   char buffer[1024];
//   std::cout << "Type: Parameter=Value\n";
//   fgets( buffer, 1024, stdin);
//   if ( strchr(buffer,'?')!=0){
//     showParams(configs);
//     return;
//   }

//   char *p = strchr(buffer,'=');
//   if (p){
//     *p=0; // terminate key string 
//     double v=strtod(p+1,0);
//     for(ConfigList::iterator i=configs.begin(); i != configs.end(); i++){
//       if ((*i)->setParam(buffer,v))
// 	printf(" %s=\t%f \n", buffer, (*i)->getParam(buffer));
//     }
//   }
// }

// void usage(const char* progname){
//   printf("Parameter: %s [-r SEED] [-x WxH] [-pause] [-notex] [-noshadow]\n", progname);
//   printf("\t-r SEED\t\tuse SEED as random number seed\n");
//   printf("\t-x WxH\t\twindow size of width(W) x height(H) is used (640x480 default)\n");
//   printf("\t-pause \t\tstart in pause mode\n");
//   printf("\t-notex \t\tdo not display textures\n");
//   printf("\t-noshadow\tdo not display shadows\n");
//   exit(0);
// }



// /// internals
// int Control_C=0;

// void control_c(int i){
//   Control_C++ ;
//   // if (Control_C > 100)exit(0);
// }

// void cmd_handler_exit(void){
//   signal(SIGINT,SIG_DFL);
//   Control_C=0;
// }

// void cmd_handler_init(){
//   signal(SIGINT,control_c);
//   atexit(cmd_handler_exit);
// }

// bool control_c_pressed(){
//   return Control_C!=0;
// }

// void cmd_begin_input(){
//   cmd_handler_exit();
// }

// void cmd_end_input(){
//   cmd_handler_init();  
// }

// void initViewedRobot() {
//   // setting the robot for view
//   if (!viewedRobot) {
//     OdeAgentList::iterator i=globalData.agents.begin();
//     viewedRobot=(*i)->getRobot();
//   }
// }

// void usercommand_handler(int key) {
//   // the stuff for handling internal commands
//   switch (key) {
//   case ' ': // key 32 (space) is for switching between the robots
//     initViewedRobot();
//     for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++){
//       if (viewedRobot==(*i)->getRobot()) { // our current agent is found
// 	if (i!=globalData.agents.end()-1) {
// 	  viewedRobot=(*(i+1))->getRobot(); // take the next robot
// 	}
// 	else {
// 	  OdeAgentList::iterator j=globalData.agents.begin();
// 	  viewedRobot=(*j)->getRobot();
// 	}
// 	break;
//       }
//     }
//     cout << "View at robot: " << viewedRobot->getName() << endl;
//     break;
//   case 'v': // is for switching between the camera modes
//     initViewedRobot();    
//     switch (camType) {
//     case Static: // now has to be TV
//       // initializes the robot to view
//       camType = TV;
//       break;
//     case TV: // now has to be advancedTV
//       camType = Following;
//       break;
//     case Following: // now has to be Following
//       camType = advancedTV;
//       break;
//     case advancedTV:
//       camType = advancedFollowing;
//       break;
//     case advancedFollowing: // now has to be Static
//       camType = Static;
//       break;
//     }
//     break;
//   case 'g': // toggle video capture mode
//     if (videostream.opened){
//       printf("Stop capturing mode\n");
//       closeVideoStream(videostream);
//     }else{
//       time_t t = time(NULL);
//       struct tm* tm = gmtime(&t);
//       char name[129];
//       sprintf(name,"frames/frame_%02i_%02i_%02i", tm->tm_hour,tm->tm_min,tm->tm_sec);
//       printf("Start capturing to %s\n",name);
//       system("mkdir -p frames");
//       videostream = openVideoStream(name);
//     }
//     break;
//   case 'c': // move camera to robot position
//     if (viewedRobot) {
//       moveOnRobot(*viewedRobot);
//     }
//     break;
//   case 'b': // move camera behind robot movement
//     if (viewedRobot) {
//       moveBehindRobot(*viewedRobot);
//     }
//     break;
//   default: // now call the user command
//     if (commandFunction) commandFunction(odeHandle, globalData, key);
//     break;
//   }
// }

