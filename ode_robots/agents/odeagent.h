/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
#ifndef __ODEAGENT_H
#define __ODEAGENT_H

#include <selforg/agent.h>
#include <selforg/storeable.h>
#include "oderobot.h"
#include "osgprimitive.h"
#include "primitive.h"

namespace lpzrobots {
  class Joint;
  
  typedef std::list<PlotOption> PlotOptionList;
  
  /** Specialised agent for ode robots
   */
  class OdeAgent : public Agent, public Storeable {
  public:

    /** @deprecated obsolete provide globaldata, see the other constructors
     */
    OdeAgent(const PlotOption& plotOption = PlotOption(NoPlot), double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "$ID$") __attribute__ ((deprecated));
    /** @deprecated obsolete provide globaldata, see the other constructors
     */
    OdeAgent(const std::list<PlotOption>& plotOptions, double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "$ID$") __attribute__ ((deprecated));
    /** The plotoptions are taken from globaldata
        @param noisefactor factor for sensor noise for this agent
     */
    OdeAgent(const GlobalData& globalData, double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "");
    /** Provided for convinience. A single plotoption is used as given by plotOption */
    OdeAgent(const GlobalData& globalData, const PlotOption& plotOption, double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "");
    /** Provided for convinience. The plotoptions are taken from the given plotOptions 
        (and not from globaldata, if you wish to overwrite them)
    */
    OdeAgent(const GlobalData& globalData, const PlotOptionList& plotOptions, double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "");
    virtual ~OdeAgent() {}

    /** initializes the object with the given controller, robot and wiring
	and initializes plotoptionengine
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
        The robot is move into the air and is fixed by a fixed joint if fixRobot==true
        See WiredController::startMotorBabblingMode().
    */
    virtual void startMotorBabblingMode (int steps,
					 AbstractController* babblecontroller = 0,
					 bool fixRobot=true);
    
    /** stops the motor babbling mode. */
    virtual void stopMotorBabblingMode ();
    
    /**
     * Returns a pointer to the robot.
     */
    virtual OdeRobot* getRobot() { return (OdeRobot*)robot;}
    /**
     * Returns a const pointer to the robot.
     */
    virtual const OdeRobot* getRobot() const { return (OdeRobot*)robot;}

    /// gives the number of past robot positions shown as trace in the graphical rendering
    virtual int getTraceLength(){return trace_length;}

    /** sets the number of past robot positions shown as trace in the graphical rendering.
        Has to be called before the tracing is initialized (otherwise returns false).
     */
    virtual bool setTraceLength(int tracelength);

    /// sets the thickness of the tube representing the trace in the graphical rendering
    virtual void setTraceThickness(int tracethickness){
      this->trace_thickness = tracethickness;
    }



    /****** STOREABLE **********/
    virtual bool store(FILE* f) const;
    virtual bool restore(FILE* f);  
    

  protected:
    void internInit(){
      trace_length=0; // number of past robot positions shown in osg
    }
    
    /**
     * initialize tracing in ode
     */
    virtual void init_tracing();

    
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
