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
 *   Revision 1.3  2005-06-15 14:01:31  martius
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
extern dWorldID world;
extern dSpaceID space;
extern dJointGroupID contactgroup;
extern dGeomID ground; //Untergundfläche

// Simulation control variables
#include "odeconfig.h"
extern double simulationTime;
extern OdeConfig simulationConfig;

// Object lists
#include "configurable.h"
#include "abstractobstacle.h"
#include "agent.h"

typedef vector<AbstractObstacle*> ObstacleList;
extern ObstacleList obstacles;

typedef vector<Configurable*> ConfigList;

typedef vector<Agent*> AgentList;
extern AgentList agents;

// simulation stuff
void simulation_init(void (*start)(), void (*end)(), void (*config)() );
void simulation_start(int argc, char** argv);
void simulation_close();

// Helper
Position mkPosition(double x, double y, double z);
Color mkColor(double r, double g, double b);

// Commandline interface stuff
void showParams(const ConfigList& configs);
void changeParams(ConfigList& configs);

//dadurch wird mit den Double-Genauigkeitszeichenmethoden gearbeitet
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif


#endif
