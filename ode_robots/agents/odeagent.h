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
 *   Revision 1.12  2010-12-17 17:00:26  martius
 *   odeagent has new constructor (old is marked as deprecated) -> log files have again
 *    important information about simulation
 *   addsensorstorobotadapater copies configurables
 *   torquesensors still in debug mode
 *   primitives support explicit decelleration (useful for rolling friction)
 *   hurling snake has rolling friction
 *
 *   Revision 1.11  2010/10/20 13:16:51  martius
 *   motor babbling mode added: try to fix robot (has to be improved!)
 *
 *   Revision 1.10  2009/08/07 09:27:58  martius
 *   additional constructor with globaldata
 *
 *   Revision 1.9  2009/03/27 13:55:06  martius
 *   traceing in a extra function
 *
 *   Revision 1.8  2008/04/18 09:50:24  guettler
 *   Implemented step functions for multiple threads of the class Simulation
 *
 *   Revision 1.7  2008/04/17 15:59:00  martius
 *   OSG2 port finished
 *
 *   Revision 1.6.2.1  2008/04/17 15:05:47  martius
 *   use RandGen
 *
 *   Revision 1.6  2007/08/30 09:46:41  martius
 *   simulation time
 *
 *   Revision 1.5  2007/03/28 07:16:58  martius
 *   trace is drawn thicker
 *
 *   Revision 1.4  2006/12/11 18:11:01  martius
 *   noisefactor and default constructor
 *
 *   Revision 1.3  2006/07/20 17:19:43  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:31  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.6  2006/05/15 13:14:10  robot3
 *   STRG-R now makes screenshots in jpg-format
 *   STRG-F now toggles the file logging (controller stuff) on/off
 *   STRG-G now restarts the GuiLogger
 *
 *   Revision 1.1.2.5  2006/03/31 16:16:58  fhesse
 *   changed trace() to init_tracing()
 *   and check for init at beginning of step
 *
 *   Revision 1.1.2.4  2006/03/29 15:08:06  martius
 *   Agent::interninit not necessary
 *
 *   Revision 1.1.2.3  2006/03/28 14:14:44  fhesse
 *   tracing of a given primitive (in the osg window) added
 *
 *   Revision 1.1.2.2  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.1.2.1  2005/11/15 12:29:18  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __ODEAGENT_H
#define __ODEAGENT_H

#include <selforg/agent.h>
#include "oderobot.h"
#include "osgprimitive.h"
#include "primitive.h"

namespace lpzrobots {
  class Joint;
  
  typedef std::list<PlotOption> PlotOptionList;
  
  /** Specialised agent for ode robots
   */
  class OdeAgent : public Agent {
  public:
    /// obsolete provide globaldata, see below
    OdeAgent(const PlotOption& plotOption = PlotOption(NoPlot), double noisefactor = 1) __attribute__ ((deprecated));
    /// obsolete provide globaldata, see below
    OdeAgent(const std::list<PlotOption>& plotOptions, double noisefactor = 1) __attribute__ ((deprecated));
    /// Constructor: The plotoptions are taken from globaldata
    OdeAgent(const GlobalData& globalData, double noisefactor = 1);    
    /** Constructor: A single plotoption is used as given by plotOption */
    OdeAgent(const GlobalData& globalData, const PlotOption& plotOption, double noisefactor = 1);
    /** Constructor: The plotoptions are taken from the given plotOptions 
        (if you wish to overwrite them)
    */
    OdeAgent(const GlobalData& globalData, const PlotOptionList& plotOptions, double noisefactor = 1);
    /** destructor
     */
    virtual ~OdeAgent() {}

    /** initializes the object with the given controller, robot and wiring
	and initializes pipe to guilogger
    */
    virtual bool init(AbstractController* controller, OdeRobot* robot, AbstractWiring* wiring,
		      long int seed = 0){
      return Agent::init(controller, robot, wiring, seed);
    }


    virtual void step(double noise, double time);

    /**
     * Special function for the class Simulation to seperate the step
     * of the WiredController (plus TrackRobot) and the setting and getting
     * of the motor- and sensorvalues.
     * @param noise @see step()
     * @param time @see step()
     */
    virtual void stepOnlyWiredController(double noise, double time);

    /**
     * Special function for the class Simulation to seperate the step
     * of the WiredController (plus TrackRobot) and the setting and getting
     * of the motor- and sensorvalues.
     */
    virtual void setMotorsGetSensors();

    /** Enables the motor babbling mode. 
        The robot move to the air and is fixed by a fixed joint.
        See WiredController::startMotorBabblingMode().
    */
    virtual void startMotorBabblingMode (int steps, AbstractController* babblecontroller = 0);
    
    /** stops the motor babbling mode. */
    virtual void stopMotorBabblingMode ();
    
    /**
     * Returns a pointer to the robot.
     */
    virtual OdeRobot* getRobot() { return (OdeRobot*)robot;}

    /// gives the number of past robot positions shown as trace in osg
    virtual int getTraceLength(){return trace_length;}


  protected:
    void internInit(){
      trace_length=0; // number of past robot positions shown in osg
    }
    
    /**
     * initialize tracing in ode
     * @param tracelength number of past positions shown as trace in osg
     * @param tracethickness  thickness of the trace
     */
    virtual void init_tracing(int tracelength=1000, double tracethickness=0.05);

    
    /**
     * continues the trace by one segment
     */
    virtual void trace();

    /** tries to fixate the robot at fixatingPos */
    virtual void tryFixateRobot();

  private:
    void constructor_helper(const GlobalData* globalData);

    int trace_length;
    double trace_thickness;
    int counter;
    bool tracing_initialized;

    OSGPrimitive** segments; // stores segments(cylinders) of the trace
    osg::Vec3 lastpos;

    Pos fixatingPos; 
    bool fixateRobot;
    Joint* fixedJoint;
  };

}

#endif
