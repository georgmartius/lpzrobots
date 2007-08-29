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
 *   Revision 1.68  2007-08-29 13:08:26  martius
 *   added HUD with time and caption
 *
 *   Revision 1.67  2007/08/24 11:52:18  martius
 *   timer reset on key stroke handling
 *   different substance callback handling
 *
 *   Revision 1.66  2007/07/31 08:20:33  martius
 *   list of spaces for internal collision detection added
 *
 *   Revision 1.65  2007/07/19 15:54:55  martius
 *   fixme added
 *
 *   Revision 1.64  2007/07/03 13:09:32  martius
 *   odeHandle knows about time
 *
 *   Revision 1.63  2007/06/21 16:19:59  martius
 *   -nopgraphics option which disables graphics rendering
 *
 *   Revision 1.62  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.61  2007/05/09 14:57:25  robot3
 *   to increase or reduce the simulation speed (realtimefactor), use + and -
 *   to toggle to the maximum simulation speed, use * in the simulation
 *
 *   Revision 1.60  2007/04/20 12:18:56  martius
 *   help output
 *
 *   Revision 1.59  2007/03/26 13:06:19  martius
 *   use new commandline interface
 *
 *   Revision 1.58  2007/03/16 10:55:03  martius
 *   new nearcallback structure
 *   Toplevel nearcallback uses robots collision control
 *   normal nearcallback uses new substance implementation
 *
 *   Revision 1.57  2007/03/05 17:54:24  martius
 *   soundMan with parameters
 *
 *   Revision 1.56  2007/02/27 11:56:40  robot5
 *   Minor changes for SoundMan functionalities.
 *
 *   Revision 1.55  2007/02/12 13:29:40  martius
 *   addCallback has flag about controllstep
 *
 *   Revision 1.54  2006/12/13 09:13:03  martius
 *   agents get comments about changed parameter for logfile
 *
 *   Revision 1.53  2006/12/11 18:31:34  martius
 *   list of configurables for agents
 *   reference counting and memleaks fixed
 *   onlycontrol used in steps where  controller is not used
 *
 *   Revision 1.52  2006/11/30 10:06:41  robot5
 *   Added support for Sndchanger (experimental). Startup with argument -s.
 *
 *   Revision 1.51  2006/10/20 14:24:55  martius
 *   max velocity for joint-correction limited
 *
 *   Revision 1.50  2006/09/22 10:57:36  martius
 *   again hard collisions
 *
 *   Revision 1.49  2006/09/21 22:09:01  martius
 *   timeleak is seldom annouced
 *
 *   Revision 1.48  2006/09/21 16:17:27  der
 *   different friction because of terrain
 *
 *   Revision 1.47  2006/09/21 16:01:48  martius
 *   relaxed collisions
 *
 *   Revision 1.46  2006/09/20 15:30:40  martius
 *   shadowsize
 *
 *   Revision 1.45  2006/09/20 12:55:09  martius
 *   ERP = 0.999 and CFM=0.0001
 *
 *   Revision 1.44  2006/08/30 09:00:19  martius
 *   code for full collision control inserted (as a comment)
 *
 *   Revision 1.43  2006/08/04 15:07:46  martius
 *   documentation
 *
 *   Revision 1.42  2006/07/14 15:17:33  fhesse
 *   start option for intended simulation time added
 *   -simtime [min]
 *
 *   Revision 1.40.4.34  2006/07/10 12:03:47  martius
 *   Cosmetics
 *
 *   Revision 1.40.4.33  2006/06/29 16:41:14  robot3
 *   you can now see the bounding shapes of meshes
 *   if you type ./start -drawboundings on command line
 *
 *   Revision 1.40.4.32  2006/06/29 16:31:47  robot3
 *   includes cleared up
 *
 *   Revision 1.40.4.31  2006/06/25 16:52:23  martius
 *   filelogging is done with a plotoption
 *
 *   Revision 1.40.4.30  2006/05/28 22:11:44  martius
 *   - noshadow cmdline flag
 *
 *   Revision 1.40.4.29  2006/05/15 13:07:35  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.40.4.28  2006/05/08 12:12:20  robot3
 *   changes from Revision 1.40.4.27 reverted
 *
 *   Revision 1.40.4.27  2006/04/27 16:31:35  robot3
 *   -motionblur inlucded
 *   -if the simulation is not in videoRedordingMode,
 *    50fps are now as standard used.
 *
 *   Revision 1.40.4.26  2006/03/29 15:07:29  martius
 *   Dummy Primitive for Environment
 *
 *   Revision 1.40.4.25  2006/03/19 13:38:08  robot3
 *   cameramanipulator for race mode included
 *
 *   Revision 1.40.4.24  2006/03/08 11:21:20  robot3
 *   Following Cameramanipulator is now on key 2
 *
 *   Revision 1.40.4.23  2006/03/06 16:53:49  robot3
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
#include <selforg/abstractcontroller.h>
#include <selforg/abstractwiring.h>

#include "simulation.h"
#include "odeagent.h"
#include "console.h"

#include <osg/ShapeDrawable>
#include <osg/ArgumentParser>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>

#include "primitive.h"
#include "abstractobstacle.h"

#include "extendedViewer.h"
#include "cameramanipulatorTV.h"
#include "cameramanipulatorFollow.h"
#include "cameramanipulatorRace.h"

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
    useShadow= true;
    noGraphics=false;
    simulation_time=-1;
    simulation_time_reached=false;
    viewer   = 0;
    cam      = 0;
    arguments= 0;
    // we have to count references by our selfes
    osg::Referenced::ref();
    osgGA::GUIEventHandler::ref();
    Producer::Camera::Callback::ref();
  }

  Simulation::~Simulation(){
    if(state!=running) return;
    dJointGroupDestroy ( odeHandle.jointGroup );
    dWorldDestroy ( odeHandle.world );
    dSpaceDestroy ( odeHandle.space );
    dCloseODE ();

    state=closed;
    if(arguments) delete arguments;
    // we have to count references by our selfes
    osgGA::GUIEventHandler::unref();
    Producer::Camera::Callback::unref_nodelete();
    Producer::Camera::Callback::unref_nodelete();
    osg::Referenced::unref_nodelete();    
  }

  bool Simulation::init(int argc, char** argv){

    /**************** ODE-Section   ***********************/
    odeHandle.init(&globalData.time);

    globalData.odeConfig.setOdeHandle(odeHandle);

    //set Gravity to Earth level
    dWorldSetGravity ( odeHandle.world , 0 , 0 , globalData.odeConfig.gravity );
    dWorldSetERP ( odeHandle.world , 0.3 );
    dWorldSetCFM ( odeHandle.world,1e-4);

    dWorldSetContactMaxCorrectingVel (odeHandle.world, 100); // default is infinity
    dWorldSetContactSurfaceLayer (odeHandle.world, 0.001); // default is 0

    cmd_handler_init();

    globalData.environment = new DummyPrimitive();

    // add ode config to config list
    globalData.configs.push_back(&(globalData.odeConfig));
    globalconfigurables.push_back(&(globalData.odeConfig));

    /**************** OpenSceneGraph-Section   ***********************/

    osgDB::FilePathList l = osgDB::getDataFilePathList();
    l.push_back("../../osg/data");
    osgDB::setDataFilePathList(l);

    processCmdLine(argc, argv);

    if(!noGraphics){
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
    }

    for(int i=0; i<3; i++){
      osgHandle.tesselhints[i] = new TessellationHints();
      osgHandle.tesselhints[i]->ref();
    }
    osgHandle.tesselhints[0]->setDetailRatio(0.1f); // Low
    osgHandle.tesselhints[1]->setDetailRatio(1.0f); // Middle
    osgHandle.tesselhints[2]->setDetailRatio(3.0f); // High

    osgHandle.color = Color(1,1,1,1);

    osgHandle.scene=makeScene();
    if (!osgHandle.scene) return false;

    osgHandle.normalState = new StateSet();
    osgHandle.normalState->ref();

    // set up blending for transparent stateset
    osg::StateSet* stateset = new StateSet();
    osg::BlendFunc* transBlend = new osg::BlendFunc;
    transBlend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
    stateset->setAttributeAndModes(transBlend, osg::StateAttribute::ON);
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //stateset->setRenderBinDetails(5,"RenderBin");
    stateset->setMode(GL_CULL_FACE,osg::StateAttribute::ON); // disable backface because of problems
    osgHandle.transparentState = stateset;
    osgHandle.transparentState->ref();

    if(!noGraphics){
      // setup the camera manipulators
      CameraManipulator* defaultCameramanipulator =
	new CameraManipulator(osgHandle.scene, globalData);
      CameraManipulator* cameramanipulatorFollow =
	new CameraManipulatorFollow(osgHandle.scene, globalData);
      CameraManipulator* cameramanipulatorTV =
	new CameraManipulatorTV(osgHandle.scene, globalData);
      CameraManipulator* cameramanipulatorRace =
	new CameraManipulatorRace(osgHandle.scene, globalData);
      unsigned int pos = viewer->addCameraManipulator(defaultCameramanipulator);
      viewer->addCameraManipulator(cameramanipulatorFollow);
      viewer->addCameraManipulator(cameramanipulatorTV);
      viewer->addCameraManipulator(cameramanipulatorRace);
      viewer->selectCameraManipulator(pos); // this is the default camera type
      
      // get details on keyboard and mouse bindings used by the viewer.
      viewer->getUsage(*(arguments->getApplicationUsage()));
    }

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
    initializeConsole();

    //********************Simulation start*****************
    state=running;
    globalData.time=0;
    resetSyncTimer();

    start(odeHandle, osgHandle, globalData);

    if(!noGraphics){
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

    // set our motion blur callback as the draw callback for each scene handler
    //      osgProducer::OsgCameraGroup::SceneHandlerList &shl = viewer->getSceneHandlerList();
    //      for (osgProducer::OsgCameraGroup::SceneHandlerList::iterator i=shl.begin(); i!=shl.end(); ++i)
    //      {
    //          (*i)->setDrawCallback(new MotionBlurDrawCallback(globalData));
    //      }
    }

    if(!noGraphics){
      while ( (!viewer->done()) && (!simulation_time_reached) )
	{
	  // wait for all cull and draw threads to complete.
	  viewer->sync();
	  
	  if(!loop()) break;
	  
	  // wait for all cull and draw threads to complete.
	  viewer->sync();
	  // update the scene by traversing it with the the update visitor which will
	  // call all node update callbacks and animations.
	  viewer->update();
	  	  
	  // fire off the cull and draw traversals of the scene.
	  viewer->frame();
	}
      
      // wait for all cull and draw threads to complete before exit.
      viewer->sync();
    }else{
      while ( !simulation_time_reached)
      {
	if(!loop()) break;
      }
    }

    closeConsole();
    end(globalData);
    tidyUp(globalData);
    return true;

  }

  bool Simulation::config(GlobalData& globalData){
    return handleConsole(globalData);
  }

  void Simulation::end(GlobalData& globalData){
  }

  bool Simulation::loop(){
    // we run the physical simulation as often as "drawinterval",
    //  the drawing of all object should occur if t==0
    bool run=true;
    for(int t = 0; t < globalData.odeConfig.drawInterval; t++){
      // Parametereingabe
      if (control_c_pressed()){
	cmd_begin_input();
	run=config(globalData);
	cmd_end_input();
	resetSyncTimer();
      }

      // the simulation just runs if pause is not enabled
      if (!pause) {
	globalData.time += globalData.odeConfig.simStepSize;
	sim_step++;
	// print simulation time every 10 min.
	if(sim_step% ( int(1/globalData.odeConfig.simStepSize) * 600) ==0) {
	  printf("Simulation time: %li min\n", sim_step/ ( long(1/globalData.odeConfig.simStepSize)*60));
	}
	// finish simulation, if intended simulation time is reached
	if(simulation_time!=-1){ // check time only if activated 
	  if( (sim_step/ ( long(1/globalData.odeConfig.simStepSize)*60))  == simulation_time) {
	    if (!simulation_time_reached){ // print out once only
	      printf("%li min simulation time reached -> simulation stopped \n", simulation_time);
	    }
	    simulation_time_reached=true;
	  }
	}

	// for all agents: robots internal stuff and control step if at controlInterval
	for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++){
	  if ( (sim_step % globalData.odeConfig.controlInterval ) == 0 ){
	    (*i)->step(globalData.odeConfig.noise, globalData.time);
	    (*i)->getRobot()->doInternalStuff(globalData);
	  }else{
	    (*i)->onlyControlRobot();
	  }
	}

	/**********************Simulationsschritt an sich**********************/
	dSpaceCollide ( odeHandle.space , this , &nearCallback_TopLevel );
	FOREACHC(list<dSpaceID>, odeHandle.getSpaces(), i){	  
	  dSpaceCollide ( *i , this , &nearCallback );
	}


	dWorldStep ( odeHandle.world , globalData.odeConfig.simStepSize );
	//ODE-Engine geht einen Schritt weiter
	dJointGroupEmpty (odeHandle.jointGroup);
      }

      addCallback(globalData, t==0, pause, (sim_step % globalData.odeConfig.controlInterval ) == 0);

      if(t==0 && !noGraphics){
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
	// update timestats
	setTimeStats(globalData.time,globalData.odeConfig.realTimeFactor);

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
	  nextLeakAnnounce=100;
	}else if (diff < 0){
	  // we do not bother the user all the time
	  if(leakAnnCounter%nextLeakAnnounce==0 && diff < -100 && !videostream.isOpen()){
	    nextLeakAnnounce=min(nextLeakAnnounce*10,10000);
	    printf("Time leak of %li ms (Suggestion: realtimefactor=%g , next in annoucement in %i )\n",
		   -diff, globalData.odeConfig.realTimeFactor*0.5, nextLeakAnnounce);

	    leakAnnCounter=0;
	    resetSyncTimer();
	  }
	  leakAnnCounter++;
	}
      }
    }else if (pause) {
      usleep(1000);
    }
    return run;
  }

  bool Simulation::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&) {
    bool handled = false;
    switch(ea.getEventType()) {
    case(osgGA::GUIEventAdapter::KEYDOWN):
      {
	handled = command(odeHandle, osgHandle, globalData, ea.getKey(), true);	
	if(handled) {
	  resetSyncTimer();
	  break;
	}
	//	printf("Key: %i\n", ea.getKey());
	switch(ea.getKey()){
	case 6 : // Ctrl - f
	  for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++){
	    if(!(*i)->removePlotOption(File)){
	      (*i)->addPlotOption(PlotOption(File, Controller, filelogginginterval, globalconfigurables));
	    }
	  }
	  return true;
	  break;
	case 7 : // Ctrl - g
	  for(OdeAgentList::iterator i=globalData.agents.begin(); i != globalData.agents.end(); i++){
	    if(!(*i)->removePlotOption(GuiLogger)){
	      (*i)->addPlotOption(PlotOption(GuiLogger, Controller, guiloggerinterval, globalconfigurables));
	    }
	  }
	  return true;
	  break;
	case 65450: // keypad *  // normal * is allready used by LOD 
	  globalData.odeConfig.setParam("realtimefactor", 0);
	  std::cout << "realtimefactor = " << globalData.odeConfig.getParam("realtimefactor") << std::endl;
	  handled=true;
	  break;
	case 65451: // keypad +
	case 43: // +
	  { 
	    double rf = globalData.odeConfig.realTimeFactor;
	    if (rf >= 2)
	      globalData.odeConfig.setParam("realtimefactor", rf+1);
	    else if (rf>=1.0)
	      globalData.odeConfig.setParam("realtimefactor", rf+0.25);
	    else if (rf>=0.1)
	      globalData.odeConfig.setParam("realtimefactor", rf+0.1);
	    else
	      globalData.odeConfig.setParam("realtimefactor", 0.1);
	    std::cout << "realtimefactor = " <<  globalData.odeConfig.getParam("realtimefactor")<< std::endl;
	    handled=true;
	  }
	  break;
	case 65453: // keypad -
	case 45: // -
	  { 
	    double rf = globalData.odeConfig.realTimeFactor;
	    if (rf>2)
	      globalData.odeConfig.setParam("realtimefactor", rf-1);
	    else if (rf>=1.0)
	      globalData.odeConfig.setParam("realtimefactor", rf-0.25);
	    else if (rf>=0.1)
	      globalData.odeConfig.setParam("realtimefactor", rf-0.1);
	    else
	      globalData.odeConfig.setParam("realtimefactor", 0.1);
	    std::cout << "realtimefactor = " <<  globalData.odeConfig.getParam("realtimefactor")<< std::endl;
	    handled=true;
	  }
	  break;
	case 18:  // Ctrl - r
	  if(videostream.isOpen()){
	    printf("Stop video recording!\n");
	    videostream.close();
	    //	    printf("Switching back to 50fps!\n");
	    globalData.odeConfig.videoRecordingMode=false;
	  }else{
	    //	    printf("For video recording the simulation now switches to 25fps!\n");
	    globalData.odeConfig.videoRecordingMode=true;
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
	default:
	  // std::cout << ea.getKey() << std::endl;
	  return false;
	  break;	
	}
      }
      break;
    case(osgGA::GUIEventAdapter::KEYUP):
      handled = command(odeHandle, osgHandle, globalData, ea.getKey(), false);	
      if(handled) resetSyncTimer();
    default:
      break;
    }
    return handled;
  }
    
  void Simulation::getUsage (osg::ApplicationUsage& au) const {
    au.addKeyboardMouseBinding("Simulation: Ctrl-f","File-Logging on/off");
    au.addKeyboardMouseBinding("Simulation: Ctrl-g","Restart the Gui-Logger");
    au.addKeyboardMouseBinding("Simulation: Ctrl-r","Start/Stop video recording");
    au.addKeyboardMouseBinding("Simulation: Ctrl-p","Pause on/off");
    au.addKeyboardMouseBinding("Simulation: +","increase simulation speed (realtimefactor)");
    au.addKeyboardMouseBinding("Simulation: -","decrease simulation speed (realtimefactor)");
    au.addKeyboardMouseBinding("Simulation: *","set maximum simulation speed (realtimefactor=0)");
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
      delete (*i)->getWiring();
      delete (*i);
    }
    if(global.environment){ delete global.environment; global.environment=0;}

    if(osgHandle.normalState) osgHandle.normalState->unref();
    if(osgHandle.transparentState) osgHandle.transparentState->unref();
    for(int i=0; i<3; i++){
      if(osgHandle.tesselhints[i]) osgHandle.tesselhints[i]->unref();
    }
    global.agents.clear();
    
    if(!noGraphics)    // delete viewer;
      viewer->getEventHandlerList().clear();
  }


  void Simulation::processCmdLine(int argc, char** argv){
    if(contains(argv, argc, "-h")) usage(argv[0]);
    if(contains(argv, argc, "--help")) usage(argv[0]);

    // guilogger-loading stuff here
    // start with online windows
    int index = contains(argv, argc, "-g");
    guiloggerinterval=1;
    if(index) {
      if(argc > index)
	guiloggerinterval=atoi(argv[index]);
      plotoptions.push_back(PlotOption(GuiLogger, Controller, guiloggerinterval, globalconfigurables));
    }

    // logging to file
    filelogginginterval=5;
    index = contains(argv, argc, "-f");
    if(index) {
      if(argc > index)
	filelogginginterval=atoi(argv[index]);
      plotoptions.push_back(PlotOption(File, Controller, filelogginginterval, globalconfigurables));
    }

    // starting neuronviz
    neuronvizinterval=10;
    index = contains(argv, argc, "-n");
    if(index) {
      if(argc > index)
	neuronvizinterval=atoi(argv[index]);
      plotoptions.push_back(PlotOption(NeuronViz, Controller, neuronvizinterval, globalconfigurables));
    }

    // using SoundMan for acustic output
    index = contains(argv, argc, "-s");
    if(index) {
      string param="";
      if(argc > index) param=argv[index];
      plotoptions.push_back(PlotOption(SoundMan, Controller, 1, globalconfigurables,param));
    }

    index = contains(argv, argc, "-r");
    long seed=0;
    // initialize random number generator
    if(index && argc > index) {
      seed=atoi(argv[index]);
    }else{
      seed=time(0);
    }
    printf("Use random number seed: %li\n", seed);
    srand(seed);
    globalData.odeConfig.randomSeed=seed;

    int resolindex = contains(argv, argc, "-x");
    windowWidth = 640;
    windowHeight = 480;
    if(resolindex && argc > resolindex) {
      sscanf(argv[resolindex],"%ix%i", &windowWidth,&windowHeight);
      windowWidth = windowWidth < 64 ? 64 : (windowWidth > 1600 ? 1600 : windowWidth);
      windowHeight = windowHeight < 64 ? 64 : (windowHeight > 1200 ? 1200 : windowHeight);
    }

    pause = contains(argv, argc, "-pause")!=0;
    useShadow = contains(argv, argc, "-noshadow")==0;
    noGraphics = contains(argv, argc, "-nographics")!=0;
    shadowTexSize = 2048;
    int shadowsizeindex = contains(argv, argc, "-shadowsize");
    if(shadowsizeindex && argc > shadowsizeindex) {
      sscanf(argv[shadowsizeindex],"%i", &shadowTexSize);
      printf("shadowTexSize=%i\n",shadowTexSize);
    }

    osgHandle.drawBoundings= contains(argv, argc, "-drawboundings")!=0;

    // read intended simulation time
    index = contains(argv, argc, "-simtime");
    if (index){
      if(argc > index)
	simulation_time=atoi(argv[index]);
      printf("simtime=%li\n",simulation_time);
    }

  }

//   // This function is called, if there was a possible Collision detected (in a space used at call of dSpaceCollide)
//   void nearCallback_Old(void *data, dGeomID o1, dGeomID o2){
//     Simulation* me = (Simulation*) data;
//     if (!me) return;

//     bool collision_treated=false;
//     // call robots collision treatments
//     for(OdeAgentList::iterator i= me->globalData.agents.begin();
// 	(i != me->globalData.agents.end()) && !collision_treated; i++){
//       collision_treated=(*i)->getRobot()->collisionCallback(data, o1, o2);
//     }

//     if (collision_treated) return; // exit if collision was treated by a robot

//     if(!(me->collCallback(me->odeHandle, data,o1,o2))){

//       // Todo: here is a code, that generated all possible contact points,
//       //  for the new collision code, make sure we incorperate that.
// //       if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {
// //       // colliding a space with something
// //       dSpaceCollide2 (o1,o2,data,&nearCallback);
// //       // collide all geoms internal to the space(s)
// //       if (dGeomIsSpace (o1)) dSpaceCollide (o1,data,&nearCallback);
// //       if (dGeomIsSpace (o2)) dSpaceCollide (o2,data,&nearCallback);
// //     }
// //     else {
// //       // colliding two non-space geoms, so generate contact
// //       // points between o1 and o2
// //       int num_contact = dCollide (o1,o2,max_contacts,contact_array,skip);
// //       // add these contact points to the simulation
// //       ...
// //     }

//       // using standard collision treatment

//       int i,n;
//       const int N = 80;
//       dContact contact[N];
//       n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
//       if (n > 0) {
// 	for (i=0; i<n; i++)
// 	  {

//  	    // contact[i].surface.mode = dContactBounce | dContactSoftCFM;
// 	    //  	    contact[i].surface.mu = 1;
// 	    //  	    contact[i].surface.mu2 = 0;
// 	    //  	    contact[i].surface.bounce = 0.1;
// 	    //  	    contact[i].surface.bounce_vel = 0.1;
// 	    //  	    contact[i].surface.soft_cfm = 0.1;

// 	    contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
// 	      dContactSoftERP | dContactSoftCFM | dContactApprox1;
// 	    contact[i].surface.mu = 0.8; //normale Reibung von Reifen auf Asphalt
// 	    contact[i].surface.slip1 = 0.005;
// 	    contact[i].surface.slip2 = 0.005;
// 	    contact[i].surface.soft_erp = 0.999;
// 	    contact[i].surface.soft_cfm = 0.001;
// 	    dJointID c = dJointCreateContact (me->odeHandle.world,
// 					      me->odeHandle.jointGroup,&contact[i]);
// 	    dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
// 	  }
//       }
//     }
//   }


// This function is called, if there was a possible Collision detected (in a space used at call of dSpaceCollide (0))
  void Simulation::nearCallback_TopLevel(void *data, dGeomID o1, dGeomID o2){
    Simulation* me = (Simulation*) data;
    if (!me) return;

    bool collision_treated=false;
    // call robots collision treatments (old stuff, should be removed at some point)
    for(OdeAgentList::iterator i= me->globalData.agents.begin();
	(i != me->globalData.agents.end()) && !collision_treated; i++){
      collision_treated=(*i)->getRobot()->collisionCallback(data, o1, o2);
    }

    if (collision_treated) return; // exit if collision was treated by a robot
        
    nearCallback(data, o1, o2);    
  }


  void Simulation::nearCallback (void *data, dGeomID o1, dGeomID o2)
  {
    Simulation* me = (Simulation*) data;
    if (!me) return;
    if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {
      // colliding a space with something
      dSpaceCollide2 (o1,o2,data,&nearCallback);
      // collide all geoms internal to the space(s)
      if (dGeomIsSpace (o1)){
		if(! me->odeHandle.isIgnoredSpace((dxSpace*)o1) )
	   		dSpaceCollide ((dxSpace*)o1,data,&nearCallback);
      }
      if (dGeomIsSpace (o2)){
		if(! me->odeHandle.isIgnoredSpace((dxSpace*)o2) )
	   		dSpaceCollide ((dxSpace*)o2,data,&nearCallback);
      }
    } else {
      // colliding two non-space geoms, so generate contact
      // points between o1 and o2
      /// use the new method with substances
      dSurfaceParameters surfParams;
      // check whether ignored pair (e.g. connected by joint)
      if(me->odeHandle.isIgnoredPair(o1, o2 )) {
		//cerr << "ign:  " << o1  << " " << o2  << "\t " << me->odeHandle.ignoredPairs->size()<< endl;
		return;
      }
      //cerr << "col:  " << o1  << " " << o2  << "\t " << me->odeHandle.ignoredPairs->size()<< endl;
      //      Primitive* p1 = (Primitive*)dGeomGetData (o1);
      //      Primitive* p2 = (Primitive*)dGeomGetData (o2);
      Primitive* p1 = dynamic_cast<Primitive*>((Primitive*)dGeomGetData (o1));
      Primitive* p2 = dynamic_cast<Primitive*>((Primitive*)dGeomGetData (o2));
      if(!p1 || !p2) {
		cerr << "collision detected without primitive\n";
		return;
      }

      int i,n;
      const int N = 80;
      dContact contact[N];
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      if(n>0){
	const Substance& s1 = p1->substance;
	const Substance& s2 = p2->substance;
	int callbackrv = 1;
	if(s1.callback) {
	  callbackrv = s1.callback(surfParams, me->globalData, s1.userdata, contact, n,
				   o1, o2, s1, s2);
	}
	if(s2.callback && callbackrv==1) {
	  callbackrv = s2.callback(surfParams, me->globalData, s2.userdata, contact, n, 
				    o2, o1, s2, s1 );
	}
	if(callbackrv==1){
	  Substance::getSurfaceParams(surfParams, s1,s2, me->globalData.odeConfig.simStepSize);
	  //Substance::printSurfaceParams(surfParams);
	}
	if(callbackrv==0) return;
	for (i=0; i < n; i++) {
	  contact[i].surface = surfParams;
	  dJointID c = dJointCreateContact (me->odeHandle.world,
					    me->odeHandle.jointGroup,&contact[i]);
	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
	}
      } // if contact points
    } // if geoms 
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


  void Simulation::usage(const char* progname){
    printf("Usage: %s [-g [interval]] [-f [interval]] [-r seed]"
	   " [-x WxH] [-pause] [-noshadow] [-drawboundings] [-simtime [min]]\n", progname);
    printf("\t-g interval\tuse guilogger (default interval 1)\n");
    printf("\t-f interval\twrite logging file (default interval 5)\n");
    printf("\t-n interval\tuse neuronviz (default interval 10)\n");
    printf("\t-s \"-disc|ampl|freq val\"\tuse soundMan \n");
    printf("\t-r seed\t\trandom number seed\n");
    printf("\t-x WxH\t\twindow size of width(W) x height(H) is used (default 640x480)\n");
    printf("\t-pause \t\tstart in pause mode\n");
    printf("\t-nographics \t\tstart without any graphics\n");
    printf("\t-noshadow \tdisables shadows and shaders\n");
    printf("\t-shadowsize size \tsets the size of the shadow texture (default 2048)\n");
    printf("\t-drawboundings\tenables the drawing of the bounding shapes of the meshes\n");
    printf("\t-simtime min\tintended simulation time in minutes\n");

  }

  void Simulation::setCameraHomePos(const osg::Vec3& eye, const osg::Vec3& view){
    if(!noGraphics){
      osgGA::MatrixManipulator* mm =viewer->getCurrentCameraManipulator();
      if(mm){
	CameraManipulator* cameramanipulator = dynamic_cast<CameraManipulator*>(mm);
	if(cameramanipulator)
	  cameramanipulator->setHome(eye, view);
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

}

