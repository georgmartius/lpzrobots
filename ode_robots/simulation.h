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
 ***************************************************************************
 *                                                                         *
 *  simulation.h and simulation.cpp provide a generic ode-robot simulation *
 *  framework. It implements the initialisation, the simulation loop,      *
 *  and a basic command line interface.                                    *
 *  Usage: call simulation_init(), simulation_start(), simulation_close()  *
 *         see template_onerobot/main.cpp for an example                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.47  2010-06-28 14:42:42  martius
 *   usage info can also be provided by derived simulations
 *
 *   Revision 1.46  2010/06/03 13:40:59  guettler
 *   - added method setCameraMode(modenumber): 1 - static, 2 - follow, 3 - TV, 4 - race
 *   - added method setWatchingAgent(agent)
 *
 *   Revision 1.45  2010/03/23 13:37:48  martius
 *   added graphics update function since we need to call it also for offscreen rendering
 *
 *   Revision 1.44  2010/03/17 09:33:16  martius
 *   removed memory leaks and some small bugs
 *   valgrind suppression file is updated
 *
 *   Revision 1.43  2010/03/16 17:10:25  martius
 *   new lpzviewer class
 *   osgHandle changed (osgconfig/osgscene)
 *   robotcameramanager added
 *
 *   Revision 1.42  2010/03/05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *   Revision 1.41  2009/08/21 09:49:08  robot12
 *   (guettler) support for tasked simulations.
 *   - use the simulation template_taskedSimulations.
 *   - merged (not completely) from lpzrobots_tasked.
 *   - graphics is supported, but only for one simulation of a pool
 *
 *   Revision 1.110.2.1  2009/08/11 16:00:44  guettler
 *   - support for tasked simulations, does not yet work with graphics
 *   - in development state
 *
 *   Revision 1.40  2009/08/11 12:16:08  robot12
 *   BUGFIX: sim_step is no longer relevant, disable it to avoid wrong use (guettler)
 *
 *   Revision 1.39  2009/08/07 09:26:32  martius
 *   plotoptions and globalconfigurables are now in globaldata
 *
 *   Revision 1.38  2009/07/30 12:27:34  jhoffmann
 *   support for new CameraHandle
 *
 *   Revision 1.37  2009/07/29 14:19:49  jhoffmann
 *   Various bugfixing, remove memory leaks (with valgrind->memcheck / alleyoop)
 *
 *   Revision 1.36  2009/04/23 14:32:35  guettler
 *   cosmetic
 *
 *   Revision 1.35  2009/04/23 14:17:34  guettler
 *   new: simulation cycles, first simple implementation,
 *   use the additional method bool restart() for starting new cycles,
 *   template simulation can be found in template_cycledSimulation
 *   (originally taken from template_onerobot)
 *
 *   Revision 1.34  2009/03/12 08:43:40  martius
 *   fixed video recording
 *
 *   Revision 1.33  2008/09/16 14:46:41  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.32  2008/05/05 06:07:26  guettler
 *   wrong prototype storeODERobotsCFG()
 *
 *   Revision 1.31  2008/04/23 07:17:16  martius
 *   makefiles cleaned
 *   new also true realtime factor displayed,
 *    warning if out of sync
 *   drawinterval in full speed is 10 frames, independent of the speed
 *
 *   Revision 1.30  2008/04/17 15:59:00  martius
 *   OSG2 port finished
 *
 *   Revision 1.29.2.7  2008/04/15 16:21:52  martius
 *   Profiling
 *   Multithreading also for OSG and ODE but disables because of instabilities
 *
 *   Revision 1.29.2.6  2008/04/14 11:25:30  guettler
 *   The OSG step (Viewer) runs now in a seperate thread! A Sideeffect is that
 *   the simulation runs one step out of sync with the ode, don't worry about
 *   that. This increases the simulation speed up to 30% on a test pc.
 *   Together with the parallelisation of the ODE we have an total speed up of
 *   94% with the shadowtype 5 on an dual Pentium3 1Ghz and NVidia5250!
 *
 *   Revision 1.29.2.5  2008/04/14 10:49:23  guettler
 *   The ODE simstep runs now in a parallel thread! A Sideeffect is that
 *   the simulation runs one step out of sync with the ode, don't worry about
 *   that. This increases the simulation speed up to 50% on a test pc.
 *
 *   Revision 1.29.2.4  2008/04/11 10:41:35  martius
 *   config file added
 *
 *   Revision 1.29.2.3  2008/04/09 13:57:59  guettler
 *   New ShadowTechnique added.
 *
 *   Revision 1.29.2.2  2008/04/09 10:18:41  martius
 *   fullscreen and window options done
 *   fonts on hud changed
 *
 *   Revision 1.29.2.1  2008/04/08 14:09:23  martius
 *   compiles and runs with OSG2.2. Juhu
 *
 *   Revision 1.29  2007/08/24 11:52:42  martius
 *   resetsynctimer is protected
 *
 *   Revision 1.28  2007/06/21 16:19:48  martius
 *   -nopgraphics option which disables graphics rendering
 *
 *   Revision 1.27  2007/03/26 13:06:30  martius
 *   use new commandline interface
 *
 *   Revision 1.26  2007/03/16 10:53:18  martius
 *   new nearcallback structure
 *
 *   Revision 1.25  2007/02/12 13:29:48  martius
 *   addCallback has flag about controllstep
 *
 *   Revision 1.24  2006/12/13 09:13:03  martius
 *   agents get comments about changed parameter for logfile
 *
 *   Revision 1.23  2006/12/11 18:31:34  martius
 *   list of configurables for agents
 *   reference counting and memleaks fixed
 *   onlycontrol used in steps where  controller is not used
 *
 *   Revision 1.22  2006/09/20 15:30:40  martius
 *   shadowsize
 *
 *   Revision 1.21  2006/08/04 15:07:46  martius
 *   documentation
 *
 *   Revision 1.20  2006/07/14 15:17:33  fhesse
 *   start option for intended simulation time added
 *   -simtime [min]
 *
 *   Revision 1.18.4.19  2006/06/29 16:31:47  robot3
 *   includes cleared up
 *
 *   Revision 1.18.4.18  2006/06/25 16:52:23  martius
 *   filelogging is done with a plotoption
 *
 *   Revision 1.18.4.17  2006/05/28 22:12:03  martius
 *   - noshadow cmdline flag
 *
 *   Revision 1.18.4.16  2006/05/15 13:07:48  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.18.4.15  2006/03/30 12:34:47  martius
 *   documentation updated
 *
 *   Revision 1.18.4.14  2006/03/06 16:54:05  robot3
 *   now ExtendedViewer is used because of the new getCurrentCameraManipulator(),
 *   code optimizations
 *
 *   Revision 1.18.4.13  2006/02/22 15:26:32  martius
 *   frame grabbing with osg works again
 *
 *   Revision 1.18.4.12  2006/02/20 10:50:20  martius
 *   pause, random, windowsize, Ctrl-keys
 *
 *   Revision 1.18.4.11  2006/02/14 17:36:14  martius
 *   much better time syncronisation
 *
 *   Revision 1.18.4.10  2006/01/17 17:01:53  martius
 *   *** empty log message ***
 *
 *   Revision 1.18.4.9  2006/01/12 22:33:23  martius
 *   key eventhandler integrated
 *
 *   Revision 1.18.4.8  2005/12/29 16:49:48  martius
 *   end is obsolete
 *   tidyUp is used for deletion
 *
 *   Revision 1.18.4.7  2005/12/29 12:54:09  martius
 *   multiple Tesselhints
 *
 *   Revision 1.18.4.6  2005/12/15 17:02:04  martius
 *   light is in sky and standart cams removed
 *   config has a default implentation now
 *
 *   Revision 1.18.4.5  2005/12/11 23:35:07  martius
 *   *** empty log message ***
 *
 *   Revision 1.18.4.4  2005/12/09 16:53:17  martius
 *   camera is working now
 *
 *   Revision 1.18.4.3  2005/12/06 17:38:13  martius
 *   *** empty log message ***
 *
 *   Revision 1.18.4.2  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.18.4.1  2005/11/14 17:37:01  martius
 *   changed makefile structure to have and include directory
 *   mode to selforg
 *
 *   Revision 1.18  2005/10/06 17:11:26  martius
 *   switched to stl lists
 *
 *   Revision 1.17  2005/09/27 13:58:48  martius
 *   added drawLine
 *
 *   Revision 1.16  2005/09/22 13:17:11  martius
 *   OdeHandle and GlobalData finished
 *   doInternalStuff included
 *
 *   Revision 1.15  2005/09/22 11:21:57  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.14  2005/08/12 11:55:01  robot1
 *   camera module integrated
 *
 *   Revision 1.13  2005/08/03 20:33:30  martius
 *   changed signature of contains (but it stays compatible)
 *
 *   Revision 1.12  2005/07/27 13:23:16  martius
 *   new color and position construction
 *
 *   Revision 1.11  2005/07/18 08:35:27  martius
 *   drawcallback is additionalcallback now
 *
 *   Revision 1.10  2005/07/13 08:39:21  robot8
 *   added the possibility to use an additional command function, which handels special Inputs if the ODE simulation window has the focus
 *
 *   Revision 1.9  2005/07/08 10:14:05  martius
 *   added contains (helper for stringlist search)
 *
 *   Revision 1.8  2005/07/07 10:23:44  martius
 *   added user draw callback
 *
 *   Revision 1.7  2005/06/30 13:23:38  robot8
 *   completing the call of the dynamic collisionCallback-function for  standard collisions
 *
 *   Revision 1.6  2005/06/29 09:27:11  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2005/06/29 09:25:06  martius
 *   customized callback for collision
 *
 *   Revision 1.4  2005/06/17 08:42:01  martius
 *   documentation
 *
 *   Revision 1.3  2005/06/15 14:01:31  martius
 *   moved all general code from main to simulation
 *                                                                 *
 ***************************************************************************/
#ifndef __SIMULATION_H
#define __SIMULATION_H

// include base classes of class Simulation
#include "base.h"

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osg/Camera>


#include <cmath>
#define PI M_PI // (3.14159265358979323846)
#include <vector>
#include <iterator>
#include <string>

#include "globaldata.h"
#include "grabframe.h"
#include "pos.h"
#include "camerahandle.h"

/***  some forward declarations  ***/
class PlotOption; // selforg

namespace lpzrobots {
  class LPZViewer;
}
/*** end of forward declarations ***/


namespace lpzrobots {

  class Simulation : public Base, public osgGA::GUIEventHandler, public Configurable
  {
  public:

    /* typedef */ enum SimulationState { none, initialised, running, closed };

    Simulation();
    virtual ~Simulation();

    /** starts the Simulation. Do not overload it.
	This function returns of the simulation is terminated.
	@return: true if closed regulary, false on error
    */
    bool run(int argc, char** argv);

    // the following function have to be overloaded.

    /// start() is called at the first start of the cycles and should create all the object (obstacles, agents...).
    virtual void start(const OdeHandle&, const OsgHandle&, GlobalData& globalData) = 0;

    // the following functions have dummy default implementations

    /**
     * restart() is called at the second and all following starts of the cylce
     * The end of a cycle is determined by (simulation_time_reached==true)
     * @param the odeHandle
     * @param the osgHandle
     * @param globalData
     * @return if the simulation should be restarted; this is false by default
     */
    virtual bool restart(const OdeHandle&, const OsgHandle&, GlobalData& globalData);

    /// end() is called at the end and should tidy up
    virtual void end(GlobalData& globalData);
    /** config() is called when the user presses Ctrl-C
	@return false to exit program, true otherwise
    */
    virtual bool config(GlobalData& globalData);
    /** is called if a key was pressed.
	For keycodes see: osgGA::GUIEventAdapter
	@return true if the key was handled
    */
    virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData,
			 int key, bool down) { return false; };

    /** this can be used to describe the key bindings used by command()
     */
    virtual void bindingDescription(osg::ApplicationUsage & au) const {};

    /** this can be used to print additional usage information (cmd-line options)
     */
    virtual void usage() const {};

    /** collCallback() can be used to overload the standart collision handling.
	However it is called after the robots collision handling.
	@return true if collision is treated, false otherwise
    */
    virtual bool collCallback(const OdeHandle&, void* data, dGeomID o1, dGeomID o2) { return false;};

    /** optional additional callback function which is called every simulation step.
	Called between physical simulation step and drawing.
	@param draw indicates that objects are drawn in this timestep
	@param pause always false (only called of simulation is running)
	@param control indicates that robots have been controlled this timestep
     */
    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {};

    virtual void odeStep();

    virtual void osgStep();

  protected:
    // GUIEventHandler
    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
    virtual void getUsage (osg::ApplicationUsage & au) const;
    virtual void accept(osgGA::GUIEventHandlerVisitor& v);
    virtual bool init(int argc, char** argv);

    virtual void updateGraphics(); ///< update the graphics objects

    /** define the home position and view orientation of the camera.
	view.x is the heading angle in degree. view.y is the tilt angle in degree (nick),
	view.z is ignored
    */
    void setCameraHomePos(const osg::Vec3& eye, const osg::Vec3& view);

    /**
     * Sets the mode of the camera, the numbers are the same like the keys
     * @param mode camera mode (1 - static, 2 - follow, 3 - TV, 4 - race)
     */
    void setCameraMode(const unsigned int mode);

    /**
     * Sets the agent to be watched with the camera.
     * @param agent to set
     */
    void setWatchingAgent(OdeAgent* agent);

    static void nearCallback_TopLevel(void *data, dGeomID o1, dGeomID o2);
    static void nearCallback(void *data, dGeomID o1, dGeomID o2);
    bool control_c_pressed();

    // plotoptions is a list of possible online output,
    // if the list is empty no online gnuplot windows and no logging to file occurs.
    // The list is modified with commandline options, see run() in simulation.cpp
    std::list<PlotOption>& plotoptions; // points now to globaldata.plotoptions
    /// this list contains by default only the odeconfig. This list should be added to new agents
    std::list<Configurable*>& globalconfigurables; // points now to globaldata.globalconfigurables


  private:
    void insertCmdLineOption(int& argc,char**& argv);
    bool loop();
    /// clears obstacle and agents lists and delete entries
    void tidyUp(GlobalData& globalData);

  protected:
    /// returns false if the program is to exit
    virtual bool processCmdLine(int argc, char** argv); 
    void resetSyncTimer();
    long timeOfDayinMS();

  private:
    static void control_c(int i);
    static void cmd_handler_exit();
    static void cmd_handler_init();
    static void cmd_begin_input();
    static void cmd_end_input();

    // Commandline interface stuff
    static void main_usage(const char* progname);

    bool storeOdeRobotsCFG();


  protected:
    GlobalData globalData;
    osg::ref_ptr<VideoStream> videostream;

    int nextLeakAnnounce;
    int leakAnnCounter;
    long realtimeoffset;
    long simtimeoffset;
    double truerealtimefactor; // calculated true speed
    bool justresettimes;      // true if we just reset sync times

    paramint windowWidth;
    paramint windowHeight;

    bool pause;
    bool simulation_time_reached;
    long int simulation_time;
    bool noGraphics;

    // use globalData.sim_step instead
     //long sim_step;

    int guiloggerinterval;
    int filelogginginterval;
    int neuronvizinterval;

    char odeRobotsCfg[256]; /// < filename of config file

    //  CameraType camType; // default is a non-moving and non-rotating camera
    //  OdeRobot* viewedRobot; // the robot who is viewed from the camera

    /// the current cycle; the simulation restarts if restart() returns true
    int currentCycle;

    CameraHandle cameraHandle;

    parambool useOdeThread;
	parambool useOsgThread;
	parambool useQMPThreads; // decides if quick mp is used in this simulation
	parambool inTaskedMode;

	std::string windowName;

  protected:
    SimulationState state;
    osg::ArgumentParser* arguments;
    LPZViewer* viewer; // inherited from osgViewer::Viewer
    osgGA::KeySwitchMatrixManipulator* keyswitchManipulator;

    static int ctrl_C;
    char** orig_argv;

    // multiprocessoring stuff
    pthread_t odeThread;
    pthread_t osgThread;
    bool odeThreadCreated;
    bool osgThreadCreated;

  };

  // /// initializes or resets the camera per user, if wanted
  // void camera_init(CameraType type, OdeRobot* robot);

  // /// starts the simulation.
  // void simulation_start(int argc, char** argv);
  // /// call this after the @simulation_start()@ has returned to tidy up.
  // void simulation_close();


  // Commandline interface stuff
  /// shows all parameters of all given configurable objects
  void showParams(const ConfigList& configs);

  /// creates a new directory with the stem base, which is not yet there (using subsequent numbers)
  void createNewDir(const char* base, char *newdir);
}

#endif
