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
 *   Revision 1.8  2006-12-11 18:10:27  martius
 *   noisefactor and default constructor
 *
 *   Revision 1.7  2006/11/30 10:02:11  robot5
 *   Added support for Sndchanger (experimental). Startup with argument -s.
 *
 *   Revision 1.6  2006/11/23 13:06:10  martius
 *   onlyControlRobot is a new function. Useful for ODE simulations where physics runs faster than
 *   control cycle
 *
 *   Revision 1.5  2006/11/17 13:46:53  martius
 *   list of configureables to appear in configuration file
 *
 *   Revision 1.4  2006/08/04 15:16:13  martius
 *   documentation
 *
 *   Revision 1.3  2006/08/02 09:35:09  martius
 *   LastPlot as a dummy option added
 *
 *   Revision 1.2  2006/07/14 12:23:57  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.7  2006/06/25 16:51:35  martius
 *   configureable has name and revision
 *   a robot is configureable by default
 *
 *   Revision 1.1.2.6  2006/05/23 21:13:08  martius
 *   - add virtual destructor
 *
 *   Revision 1.1.2.5  2006/05/15 13:08:34  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.1.2.4  2006/03/30 12:33:53  fhesse
 *   trackrobot now protected to give OdeAgent access
 *
 *   Revision 1.1.2.3  2006/01/31 16:45:18  martius
 *   neuronviz plotoption
 *
 *   Revision 1.1.2.2  2005/11/15 12:30:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.1.2.1  2005/11/14 17:37:56  martius
 *   moved to selforg
 *
 *   Revision 1.9  2005/11/07 17:03:30  martius
 *   class PlotOption added
 *   agent can be constructed with a list of PlotOptions
 *   tracking file gets controller parameters as well
 *
 *   Revision 1.8  2005/10/24 13:32:07  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.7  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.6  2005/08/03 20:34:58  martius
 *   use if Inspectable interface
 *
 *   Revision 1.5  2005/07/26 17:01:47  martius
 *   flushing every 10
 *   guilogger is opened with nice -2
 *
 *   Revision 1.4  2005/07/18 10:13:46  martius
 *   noise moved to wiring
 *
 *   Revision 1.3  2005/07/14 15:57:53  fhesse
 *   now agent contains controller, robot and wiring, plotting ability included, therefore plotagent can be removed; ono2onewiring replaces one2oneagent
 *
 *   Revision 1.2  2005/06/15 14:02:47  martius
 *   revised and basicly tested
 *                                                                 *
 ***************************************************************************/
#ifndef __AGENT_H
#define __AGENT_H

#include <stdio.h>
#include <list>
#include <utility>
#include <string>


class AbstractRobot;
class AbstractController;
class AbstractWiring;
class Configurable;

#include "types.h"
#include "trackrobots.h"

class Agent;

/** Output mode for agent.
 */
enum PlotMode {
  /// dummy (does nothing) is there for compatibility, might be removed later
  NoPlot,
  /// write into file
  File,
  /// plotting with guilogger (gnuplot)
  GuiLogger,
  /// plotting with guiscreen (gnuplot) in file logging mode
  GuiLogger_File,
  /// net visualiser
  NeuronViz,

  /// Acustic output of robotic values via external Sndchanger
  SndChanger,

  /// dummy used for upper bound of plotmode type
  LastPlot
};

/** Output either sensors from robot or from controller 
    (there can be a difference depending on the used wiring)
 */
enum PlotSensors {Robot, Controller};

/** This class contains options for the use of an external plot utility like guilogger or neuronviz
    or just simply file output
 */
class PlotOption {
public:
  friend class Agent;

  PlotOption(){ mode=NoPlot; whichSensors=Controller; interval=1; pipe=0; }
  PlotOption( PlotMode mode, PlotSensors whichSensors = Controller, 
	      int interval = 1, std::list<const Configurable*> confs = std::list<const Configurable*>())
    : mode(mode), whichSensors(whichSensors), interval(interval), configureables(confs) { pipe=0; }

  virtual ~PlotOption(){}

  /// nice predicate function for finding by mode
  struct matchMode : public std::unary_function<const PlotOption&, bool> {
    matchMode(PlotMode mode) : mode(mode) {}
    int mode;
    bool operator()(const PlotOption& m) { return m.mode == mode; }
  };

  void addConfigurable(const Configurable*);

private:

  bool open(); //< opens the connections to the plot tool 
  void close();//< closes the connections to the plot tool

  FILE* pipe;
  long t;
  std::string name;

  PlotMode mode;
  PlotSensors whichSensors;
  int interval;
  std::list< const Configurable* > configureables;
};


/** The Agent contains a controller, a robot and a wiring, which connects robot and controller.
    Additionally there are some ways to keep track of internal information.
    You have the possibility to keep track of sensor values, 
     motor values and internal parameters of the controller with PlotOptions. 
    The name PlotOptions is a bit missleaded, it should be "OutputOptions", 
     however you can write the data into a file or send it to visialisation tools like
     guilogger or neuronviz.

    If want to log the position, speed and orienation of your robot you can use setTrackOptions().
 */
class Agent {
public:
  /** constructor. PlotOption as output setting.
      noisefactor is used to set the relative noise strength of this agent     
   */
  Agent(const PlotOption& plotOption = PlotOption(NoPlot), double noisefactor = 1);
  /** constructor. A list of PlotOption can given.
      noisefactor is used to set the relative noise strength of this agent     
   */
  Agent(const std::list<PlotOption>& plotOptions, double noisefactor = 1);

  /** destructor
   */
  virtual ~Agent();  

  /** initializes the object with the given controller, robot and wiring
      and initializes the output options
  */
  virtual bool init(AbstractController* controller, AbstractRobot* robot, AbstractWiring* wiring);

  /** Performs an step of the agent, including sensor reading, pushing sensor values through wiring, 
      controller step, pushing controller outputs (= motorcommands) back through wiring and sent 
      resulting motorcommands to robot.
      @param noise Noise strength.
  */
  virtual void step(double noise);

  /** Sends only last motor commands again to robot.  */
  virtual void onlyControlRobot();

  /** Returns a pointer to the controller.
   */
  virtual AbstractController* getController() { return controller;}

  /** Returns a pointer to the robot.
   */
  virtual AbstractRobot* getRobot() { return robot;}

  /** Returns a pointer to the wiring.
   */
  virtual AbstractWiring* getWiring() { return wiring;}

  /// sets the trackoptions which enable tracking of a robot
  virtual void setTrackOptions(const TrackRobot& trackrobot);


  /** adds the PlotOptions to the list of plotoptions
      If a plotoption with the same Mode exists, then the old one is deleted first
   */
  virtual void addPlotOption(const PlotOption& plotoption);

  /** removes the PlotOptions with the given type 
      @return true if sucessful, false otherwise
   */
  virtual bool removePlotOption(PlotMode mode);
  
protected:

  /**
   * Plots controller sensor- and motorvalues and internal controller parameters.
   * @param rx actual sensorvalues from robot (used for generation of motorcommand in actual timestep)
   * @param rsensornumber length of rx   
   * @param cx actual sensorvalues which are passed to controller (used for generation of motorcommand in actual timestep)
   * @param csensornumber length of cx   
   * @param y actual motorcommand (generated in the actual timestep)
   * @param motornumber length of y 
   */
  virtual void plot(const sensor* rx, int rsensornumber, const sensor* cx, int csensornumber, 
		    const motor* y, int motornumber);
  

  AbstractController* controller;
  AbstractRobot* robot;
  AbstractWiring* wiring;

  /// number of sensors of robot
  int rsensornumber;
  /// number of motors of robot
  int rmotornumber;
  /// number of sensors of comntroller
  int csensornumber;
  /// number of motors of comntroller
  int cmotornumber;

  /// factor that is  muliplied with noise stength
  double noisefactor;

  sensor *rsensors;
  motor  *rmotors;
  sensor *csensors;
  motor  *cmotors;

  void internInit();

 protected:
  TrackRobot trackrobot;

 private:
  std::list<PlotOption> plotOptions;

  int t;

};

#endif
