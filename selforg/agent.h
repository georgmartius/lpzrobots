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
 *   Revision 1.1.2.3  2006-01-31 16:45:18  martius
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
using namespace std;

class AbstractRobot;
class AbstractController;
class AbstractWiring;

#include "types.h"
#include "trackrobots.h"

class Agent;

/** Plot mode for plot agent.
 */
enum PlotMode {
  /// no plotting to screen or logging to file
  NoPlot, 
  /// only plotting to screen, no logging to file
  GuiLogger, 
  /// plotting to screen and logging to file
  GuiLogger_File,
  /// net visualiser
  NeuronViz
  };

/** Plot either sensors from robot or from controller 
    (there can be a difference depending on the used wiring)
 */
enum PlotSensors {Robot, Controller};

/** This class contains option and internal data for the use of an external plot util
    like guilogger or neuronviz
 */
class PlotOption {
public:
  friend class Agent;

  PlotOption(){ mode=NoPlot; whichSensors=Controller; interval=1; pipe=0; }
  PlotOption( PlotMode mode, PlotSensors whichSensors = Controller, int interval = 1)
    :mode(mode), whichSensors(whichSensors), interval(interval) {  pipe=0; }

private:

  bool open(); /// opens the connections to the plot tool 
  void close();/// closes the connections to the plot tool

  FILE* pipe;
  long t;

  PlotMode mode;
  PlotSensors whichSensors;
  int interval;  
};


/** Object containing controller, robot and wiring between them.
    (Corresponding to use of the word in the robotic/simulation domain.)
 */
class Agent {
public:
  /** constructor
   */
  Agent(const PlotOption& plotOption);
  Agent(const list<PlotOption>& plotOptions);

  /** destructor
   */
  virtual ~Agent();  

  /** initializes the object with the given controller, robot and wiring
      and initializes pipe to guilogger
  */
  virtual bool init(AbstractController* controller, AbstractRobot* robot, AbstractWiring* wiring);


  /** Performs an step of the agent, including sensor reading, pushing sensor values through wiring, 
      controller step, pushing controller outputs (= motorcommands) back through wiring and sent 
      resulting motorcommands to robot.
      @param noise Noise strength.
  */
  virtual void step(double noise);

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

protected:

  /**
   * Plots controller sensor- and motorvalues and internal controller parameters.
   * @param rx actual sensorvalues from robot (used for generation of motorcommand in actual timestep)
   * @param cx actual sensorvalues which are passed to controller (used for generation of motorcommand in actual timestep)
   * @param y actual motorcommand (generated in the actual timestep)
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

  sensor *rsensors;
  motor  *rmotors;
  sensor *csensors;
  motor  *cmotors;


private:
  void internInit();

  list<PlotOption> plotOptions;
  TrackRobot trackrobot;

  int t;
};

#endif
