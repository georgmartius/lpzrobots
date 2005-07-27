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
 ***************************************************************************
 *                                                                         *
 *  simulation.h and simulation.cpp provide a generic ode-robot simulation *
 *  framework. It implements the initialisation, the simulation loop,      * 
 *  and a basic command line interface.                                    *
 *  Usage: call simulation_init(), simulation_start(), simulation_close()  *
 *         see template_onerobot/main.cpp for an example                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.12  2005-07-27 13:23:16  martius
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

#include <math.h>
#define PI M_PI // (3.14159265358979323846)
#include <ode/ode.h>
#include <ode/common.h>
#include <vector>
#include <iterator>
using namespace std;

// ODE globals
extern dWorldID world;             ///
extern dSpaceID space;             ///
extern dJointGroupID contactgroup; ///
extern dGeomID ground; 

// Simulation control variables
#include "odeconfig.h"
extern double simulationTime;      ///
extern OdeConfig simulationConfig; ///

// Object lists
#include "configurable.h"
#include "abstractobstacle.h"
#include "agent.h"

typedef vector<AbstractObstacle*> ObstacleList; ///
extern ObstacleList obstacles;                  ///

typedef vector<Configurable*> ConfigList;       ///

typedef vector<Agent*> AgentList;               ///
extern AgentList agents;                        ///

// simulation stuff
/** Initialises the simulation and registers the given callback functions.
    @param start() is called at the start and should create all the object (obstacles, agents...).
    @param end() is called at the end and should tidy up
    @param config() is called when the user presses Ctrl-C. Calls usually @changeParams()@
    @param collCallback() if defined it is called instead of the default collision handling.
      However it is called after the robots collision handling.       
    @param drawCallback optional additional draw function
 */
void simulation_init(void (*start)(), void (*end)(), 
		     void (*config)(), void (*command)(int n) = 0, 
		     void (*collCallback)(void* data, dGeomID o1, dGeomID o2) = 0,
		     void (*addCallback)(bool draw, bool pause) = 0);
/// starts the simulation.
void simulation_start(int argc, char** argv);
/// call this after the @simulation_start()@ has returned to tidy up.
void simulation_close();

// Helper
/// returns true if the list contains the given string
bool contains(char **list, int len,  const char *str);

// Commandline interface stuff
/// shows all parameters of all given configurable objects
void showParams(const ConfigList& configs);
/// offers the possibility to change parameter of all configurable objects
void changeParams(ConfigList& configs);

//dadurch wird mit den Double-Genauigkeitszeichenmethoden gearbeitet
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

#endif
