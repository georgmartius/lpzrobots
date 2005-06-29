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
 *   Revision 1.5  2005-06-29 09:25:06  martius
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
extern dGeomID ground; /// Untergrundfläche

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
 */
void simulation_init(void (*start)(), void (*end)(), void (*config)(), void (*collCallback) = 0 );
/// starts the simulation.
void simulation_start(int argc, char** argv);
/// call this after the @simulation_start()@ has returned to tidy up.
void simulation_close();

// Helper
Position mkPosition(double x, double y, double z); ///
Color mkColor(double r, double g, double b);       ///

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
