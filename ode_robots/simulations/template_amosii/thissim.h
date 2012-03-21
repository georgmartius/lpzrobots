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
    class AmosII;
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

    /**
    * add own key handling stuff here, just insert some case values
    */
    virtual bool command(const lpzrobots::OdeHandle&,
          const lpzrobots::OsgHandle&,
          lpzrobots::GlobalData& globalData,
          int key,
          bool down);

protected:
    lpzrobots::Joint*   robotfixator;
    AbstractController* controller;
    lpzrobots::AmosII*  amos;
};


#endif /* THISSIM_H_ */
