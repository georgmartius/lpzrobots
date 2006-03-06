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
 *   Revision 1.40.4.23  2006-03-06 16:53:49  robot3
 *   now ExtendedViewer is used because of the new getCurrentCameraManipulator(),
 *   code optimizations
 *
 *   Revision 1.40.4.22  2006/03/04 15:04:33  robot3
 *   cameramanipulator is now updated with every draw intervall
 *
 *   Revision 1.40.4.21  2006/03/03 12:11:32  robot3
 *   neccessary changes made for new cameramanipulators
 *
 *   Revision 1.40.4.20  2006/02/22 15:26:23  martius
 *   frame grabbing with osg works again
 *
 *   Revision 1.40.4.19  2006/02/20 10:50:20  martius
 *   pause, random, windowsize, Ctrl-keys
 *
 *   Revision 1.40.4.18  2006/02/14 17:36:03  martius
 *   prevent overflow in time sync
 *
 *   Revision 1.40.4.17  2006/02/14 17:31:12  martius
 *   much better time syncronisation
 *
 *   Revision 1.40.4.16  2006/02/01 14:00:32  martius
 *   remerging of non-fullscreen start
 *
 *   Revision 1.40.4.15  2006/02/01 10:24:34  robot3
 *   new camera manipulator added
 *
 *   Revision 1.40.4.13  2006/01/12 22:32:51  martius
 *   key eventhandler integrated
 *
 *   Revision 1.40.4.12  2006/01/12 15:16:53  martius
 *   transparency
 *
 *   Revision 1.40.4.11  2005/12/29 16:49:48  martius
 *   end is obsolete
 *   tidyUp is used for deletion
 *
 *   Revision 1.40.4.10  2005/12/29 12:54:19  martius
 *   multiple tesselhints
 *
 *   Revision 1.40.4.9  2005/12/15 17:02:04  martius
 *   light is in sky and standart cams removed
 *   config has a default implentation now
 *
 *   Revision 1.40.4.8  2005/12/14 15:36:25  martius
 *   do not check for unused vars
 *
 *   Revision 1.40.4.7  2005/12/13 18:10:33  martius
 *   changelog
 *
 *   Revision 1.40.4.6  2005/12/11 23:35:07  martius
 *   *** empty log message ***
 *
 *   Revision 1.40.4.5  2005/12/09 16:53:16  martius
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
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <selforg/configurable.h>
#include <selforg/abstractcontroller.h>

#include "simulation.h"

//#include <osgProducer/Viewer>
//#include "extendedViewer.h"
#include <osg/ShapeDrawable>
#include <osg/ArgumentParser>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>

#include "odeagent.h"

#include "grabframe.h"

#include "abstractobstacle.h"
#include "cameramanipulator.h"
#include "cameramanipulatorTV.h"
#include "cameramanipulatorFollow.h"

namespace lpzrobots {

  using namespace std;
  using namespace osg;
  using namespace osgProducer;
  

  int Simulation::ctrl_C = 0;

  Simulation::Simulation(){
    nextLeakAnnounce = 20;
    leakAnnCounter = 1;
    sim_step = 0;
    state    = none;
    pause    = false;
    viewer   = 0;
    cam      = 0;
    arguments= 0;
  }

  Simulation::~Simulation(){  
    if(state!=running) return;
    dJointGroupDestroy ( odeHandle.jointGroup );
    dWorldDestroy ( odeHandle.world );
    dSpaceDestroy ( odeHandle.space );
    dCloseODE ();
  
    state=closed;
     // tweak this, because Simulation is inherited from Referenced
    osgGA::GUIEventHandler::unref_nodelete();
    // tweak this, because Simulation is inherited from Referenced
    Producer::Camera::Callback::unref_nodelete(); 
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

    osgDB::FilePathList l = osgDB::getDataFilePathList();
    l.push_back("../../osg/data");
    osgDB::setDataFilePathList(l);

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
    viewer = new ExtendedViewer(*arguments);

    // set up the value with sensible default event handlers.
    unsigned int options =  Viewer::SKY_LIGHT_SOURCE | 
      Viewer::STATE_MANIPULATOR | 
      Viewer::STATS_MANIPULATOR | 
      Viewer::VIEWER_MANIPULATOR | 
      Viewer::ESCAPE_SETS_DONE;
    viewer->setUpViewer(options);

    viewer->getEventHandlerList().push_front(this);

    // if user request help write it out to cout.
    if (arguments->read("-h") || arguments->read("--help")) {
      arguments->getApplicationUsage()->write(std::cout);
      return false;
    }
    // any option left unread are converted into errors to write out later.
    //    arguments->reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occured when parsing the program aguments.
    if (arguments->errors()) {
      arguments->writeErrorMessages(std::cout);
      return false;
    }

    osgHandle.tesselhints[0] = new TessellationHints();
    osgHandle.tesselhints[1] = new TessellationHints();
    osgHandle.tesselhints[2] = new TessellationHints();
    osgHandle.tesselhints[0]->setDetailRatio(0.1f); // Low
    osgHandle.tesselhints[1]->setDetailRatio(1.0f); // Middle
    osgHandle.tesselhints[2]->setDetailRatio(3.0f); // High

    osgHandle.color = Color(1,1,1,1);

    osgHandle.scene=makeScene();
    if (!osgHandle.scene) return false;

    osgHandle.normalState = new StateSet();
    
    // set up blending for transparent stateset      
    osg::StateSet* stateset = new StateSet();
    osg::BlendFunc* transBlend = new osg::BlendFunc;
    transBlend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
    stateset->setAttributeAndModes(transBlend, osg::StateAttribute::ON);    
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //stateset->setRenderBinDetails(5,"RenderBin");
    stateset->setMode(GL_CULL_FACE,osg::StateAttribute::ON); // disable backface because of problems
    osgHandle.transparentState = stateset;

    // setup the camera manipulators
    CameraManipulator* defaultCameramanipulator =
      new CameraManipulator(osgHandle.scene, globalData);
    CameraManipulator* cameramanipulatorTV = 
      new CameraManipulatorTV(osgHandle.scene, globalData);
    CameraManipulator* cameramanipulatorFollow = 
      new CameraManipulatorFollow(osgHandle.scene, globalData);
    unsigned int pos = viewer->addCameraManipulator(defaultCameramanipulator);
    viewer->addCameraManipulator(cameramanipulatorTV);
    viewer->addCameraManipulator(cameramanipulatorFollow);
    viewer->selectCameraManipulator(pos); // this is the default camera type
    
    // get details on keyboard and mouse bindings used by the viewer.
    viewer->getUsage(*(arguments->getApplicationUsage()));

    state=initialised;
    return true;
  }


  bool Simulation::run(int argc, char** argv){
    if(!init(argc, argv)) return false;
  
    // information on terminal, can be removed if the printout is undesired
    printf ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
    printf ( "------------------------------------------------------------------------\n" );
    printf ( "Press Ctrl-C for an basic commandline interface (on the console).\n\n" );
    printf ( "Press h      for help.\n\n" );

    //********************Simulation start*****************
    state=running;
    globalData.time=0;
    resetSyncTimer();
    
    start(odeHandle, osgHandle, globalData);  

    // add model to viewer.
    viewer->setSceneData(root); 

    Producer::CameraConfig* cfg = viewer->getCameraConfig();
    cam = cfg->getCamera(0);
    
    Producer::RenderSurface* rs = cam->getRenderSurface();
    rs->setWindowName( "LpzRobots - Selforg" );
    
    // the following starts the system in windowed mode
    int x = rs->getWindowOriginX();
    int y = rs->getWindowOriginY();
    rs->setWindowRectangle(x,y,windowWidth, windowHeight);
    rs->fullScreen(false);

    cam->addPostDrawCallback(this);

    // create the windows and run the threads.
    viewer->realize();

    while (!viewer->done())
      {
	// wait for all cull and draw threads to complete.
	viewer->sync();
           
	loop();

	// update the scene by traversing it with the the update visitor which will
	// call all node update callbacks and animations.
	viewer->update();      

      
	// fire off the cull and draw traversals of the scene.
	viewer->frame();
      }
    
    // wait for all cull and draw threads to complete before exit.
    viewer->sync();    
    tidyUp(globalData);
    end(globalData);
    return true;

  }

  void Simulation::config(GlobalData& globalData){
    changeParams(globalData.configs);
  }

  void Simulation::end(GlobalData& globalData){
  }

  void Simulation::loop(){
    // we run the physical simulation as often as "drawinterval",
    //  the drawing of all object should occur if t==0  
    for(int t = 0; t < globalData.odeConfig.drawInterval; t++){
      // Parametereingabe  
      if (control_c_pressed()){
	cmd_begin_input();
	config(globalData);
	cmd_end_input();
	resetSyncTimer();
      }

      // the simulation is just run if pause is not enabled
      if (!pause) {
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
 	/************************** Update the scene ***********************/
	for(ObstacleList::iterator i=globalData.obstacles.begin(); i != globalData.obstacles.end(); i++){
	  (*i)->update();
	}
	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++){
	  (*i)->getRobot()->update();
	}
 	// update the camera
	osgGA::MatrixManipulator* mm =viewer->getCurrentCameraManipulator();
	if(mm){
	  CameraManipulator* cameramanipulator = dynamic_cast<CameraManipulator*>(mm);      
	  if(cameramanipulator)
	    cameramanipulator->update();
	}
      
// 	// grab frame if in captureing mode
// 	if(videostream.opened && !pause){
// 	  grabAndWriteFrame(videostream);
// 	}
      }

    }  
    /************************** Time Syncronisation ***********************/
    // Time syncronisation of real time and simulations time (not if on capture mode, or pause)
    if(globalData.odeConfig.realTimeFactor!=0.0 && !pause){
      long elaped = timeOfDayinMS() - realtimeoffset;
      // difference between actual time and current time in milliseconds
      long diff = long((globalData.time*1000.0 - simtimeoffset)
		       / globalData.odeConfig.realTimeFactor  ) - elaped;
      if(diff > 10000 || diff < -10000)  // check for overflow or other weird things
	resetSyncTimer();
      else{
	if(diff > 4){ // if less the 3 milliseconds we don't call usleep since it needs time
	  usleep((diff-2)*1000); 
	  nextLeakAnnounce=4;
	}else if (diff < 0){	
	  // we do not bother the user all the time
	  if(leakAnnCounter%nextLeakAnnounce==0 && diff < -100 && !videostream.isOpen()){
	    printf("Time leak of %li ms (Suggestion: realtimefactor=%g , next in annoucement in %i )\n",
		   -diff, globalData.odeConfig.realTimeFactor*0.5, nextLeakAnnounce);
	    nextLeakAnnounce=min(nextLeakAnnounce*2,512);
	    leakAnnCounter=0;
	    resetSyncTimer();
	  }
	  leakAnnCounter++;
	} 
      }
    }else if (pause) {
      usleep(1000); 	
    }    
  }

  bool Simulation::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&) {  
    bool handled = false;
    switch(ea.getEventType()) {
    case(osgGA::GUIEventAdapter::KEYDOWN):
      {	 
	handled = command(odeHandle, osgHandle, globalData, ea.getKey(), true); 
	if(handled) break;
	//	printf("Key: %i\n", ea.getKey());	
	switch(ea.getKey()){
	case 18:  // Ctrl - r
	  if(videostream.isOpen()){
	    printf("Stop video recording!\n");
	    videostream.close();
	  }else{
	    char dir[128];
	    char filename[140];
	    createNewDir("video", dir);
	    printf("Start video recording in %s!\n", dir);	    
	    sprintf(filename, "%s/frame", dir);
	    videostream.open(filename);
	  }	  
	  break;	
	case 16: // Ctrl - p
	  pause = !pause;
	  printf( pause ? "Pause\n" : "Continue\n" );
	  resetSyncTimer();
	  handled = true;
	  break;	 
	}
      }      	  
    case(osgGA::GUIEventAdapter::KEYUP):
      {
	  handled = command(odeHandle, osgHandle, globalData, ea.getKey(), false);	
      }
    default: 
      break;      
    }
    return handled;
  }
  
  void Simulation::getUsage (osg::ApplicationUsage& au) const {
    au.addKeyboardMouseBinding("Simulation: Ctrl-r","Start/Stop video recording");
    au.addKeyboardMouseBinding("Simulation: Ctrl-p","Pause on/off");
    bindingDescription(au);
  }
  
  void Simulation::accept(osgGA::GUIEventHandlerVisitor& v) {
    v.visit(*this);
  }

  ///////////////// Camera::Callback interface
 void Simulation::operator() (const Producer::Camera &c){
    // grab frame if in captureing mode
    if(videostream.isOpen() && !pause){
      if(!videostream.grabAndWriteFrame(c)){
	fprintf(stderr,"Stop video recording because of failture!\n");
	videostream.close();
      }
    }
  }
 
  /// clears obstacle and agents lists and delete entries
  void Simulation::tidyUp(GlobalData& global){
    // clear obstacles list
    for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++){
      delete (*i);
    }
    global.obstacles.clear();
    
    // clear agents list
    for(OdeAgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++){
      delete (*i)->getRobot();
      delete (*i)->getController();
      delete (*i);
    }
    global.agents.clear();
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

    int resolindex = contains(argv, argc, "-x");
    windowWidth = 640;
    windowHeight = 480;	
    if(resolindex && argc > resolindex) {
      sscanf(argv[resolindex],"%ix%i", &windowWidth,&windowHeight);
      windowWidth = windowWidth < 64 ? 64 : (windowWidth > 1600 ? 1600 : windowWidth);
      windowHeight = windowHeight < 64 ? 64 : (windowHeight > 1200 ? 1200 : windowHeight);
    }

    pause = contains(argv, argc, "-pause")!=0;

  }

  // Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam
  // 
  void Simulation::nearCallback(void *data, dGeomID o1, dGeomID o2){    
    Simulation* me = (Simulation*) data;
    if (!me) return;

    bool collision_treated=false;
    // call robots collision treatments
    for(OdeAgentList::iterator i= me->globalData.agents.begin(); 
	(i != me->globalData.agents.end()) && !collision_treated; i++){
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

  long Simulation::timeOfDayinMS(){    
    struct timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec*1000 + t.tv_usec/1000;	
  }

  void Simulation::resetSyncTimer(){
    realtimeoffset = timeOfDayinMS();
    simtimeoffset = int(globalData.time*1000);
  }



  // Helper
  int contains(char **list, int len,  const char *str){
    for(int i=0; i<len; i++){
      if(strcmp(list[i],str) == 0) return i+1;
    }
    return 0;
  }

  void Simulation::usage(const char* progname){
    printf("Parameter: %s [-r SEED] [-x WxH] [-pause]\n", progname);
    printf("\t-r SEED\t\tuse SEED as random number seed\n");
    printf("\t-x WxH\t\twindow size of width(W) x height(H) is used (640x480 default)\n");
    printf("\t-pause \t\tstart in pause mode\n");
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

  void createNewDir(const char* base, char *newdir){
    struct stat s;
    for(int i=0; i<1000; i++){
      sprintf(newdir,"%s%03i", base, i);
      if(stat(newdir,&s)!=0){ // file/dir does not exist -> take it
	mkdir(newdir, S_IREAD | S_IWRITE | S_IEXEC | S_IRGRP | S_IXGRP );
	return;	
      }      
    }
    assert(1); // should not happen
  }

  void Simulation::setCameraHomePos(const osg::Vec3& eye, const osg::Vec3& view){
    osgGA::MatrixManipulator* mm =viewer->getCurrentCameraManipulator();
    if(mm){
      CameraManipulator* cameramanipulator = dynamic_cast<CameraManipulator*>(mm);      
      if(cameramanipulator)
	cameramanipulator->setHome(eye, view);
    }
  }


}

