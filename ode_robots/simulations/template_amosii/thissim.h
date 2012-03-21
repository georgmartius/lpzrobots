/*
 * thissim.h
 *
 *  Created on: Feb 21, 2012
 *      Author: timo
 */

#ifndef THISSIM_H_
#define THISSIM_H_


#include <ode_robots/simulation.h>

// forward declarations START
namespace lpzrobots {
    class Joint;
    class GlobalData;
    class OdeRobot;
    class Playground;
    class OdeHandle;
    class OsgHandle;
}
class AbstractController;


/**
 * The simulation class
 */
class ThisSim : public lpzrobots::Simulation {

public:
    ThisSim();

    /**
    * starting function (executed once at the beginning of the simulation loop)
    */
    virtual void start(const lpzrobots::OdeHandle& odeHandle,
          const lpzrobots::OsgHandle& osgHandle,
          lpzrobots::GlobalData& global);

    // Called between physical simulation step and drawing in every timestep
    virtual void addCallback(lpzrobots::GlobalData& globalData, bool draw, bool pause, bool control);


    /**
     * restart() is called at the second and all following starts of the cylce
     * The end of a cycle is determined by (simulation_time_reached==true)
     * @param the odeHandle
     * @param the osgHandle
     * @param globalData
     * @return if the simulation should be restarted; this is false by default
     */
    virtual bool restart(const lpzrobots::OdeHandle&,
            const lpzrobots::OsgHandle&,
            lpzrobots::GlobalData& globalData);

    /**
    * add own key handling stuff here, just insert some case values
    */
    virtual bool command(const lpzrobots::OdeHandle&,
          const lpzrobots::OsgHandle&,
          lpzrobots::GlobalData& globalData,
          int key,
          bool down);

protected:
    // own list of inspectables
    typedef std::list<Inspectable*> InspectableList;
    InspectableList myInspectables;
    lpzrobots::Joint* robotfixator, *boxfixator;
    lpzrobots::GlobalData* global_data;

    AbstractController* controller;
    lpzrobots::OdeRobot* robot;
    lpzrobots::Playground* playground;

};


#endif /* THISSIM_H_ */
