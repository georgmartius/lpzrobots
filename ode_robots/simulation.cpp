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
 *   Revision 1.13  2005-07-13 08:39:21  robot8
 *   added the possibility to use an additional command function, which handels special Inputs if the ODE simulation window has the focus
 *
 *   Revision 1.12  2005/07/11 11:19:38  robot8
 *   adding the line, where the pointer to the additional draw function is set to the value of the parameter drawCallback
 *
 *   Revision 1.11  2005/07/08 10:14:05  martius
 *   added contains (helper for stringlist search)
 *
 *   Revision 1.10  2005/07/07 10:23:36  martius
 *   added user draw callback
 *
 *   Revision 1.9  2005/06/30 13:23:38  robot8
 *   completing the call of the dynamic collisionCallback-function for  standard collisions
 *
 *   Revision 1.8  2005/06/29 09:27:03  martius
 *   *** empty log message ***
 *
 *   Revision 1.7  2005/06/29 09:25:17  martius
 *   customized callback for collision
 *
 *   Revision 1.6  2005/06/22 15:39:49  fhesse
 *   path to textures
 *
 *   Revision 1.5  2005/06/20 10:03:26  fhesse
 *   collision treatment by agents included
 *
 *   Revision 1.4  2005/06/17 09:33:53  martius
 *   aligned values on showParams
 *
 *   Revision 1.3  2005/06/15 14:01:31  martius
 *   moved all general code from main to simulation
 *                                                                 *
 ***************************************************************************/
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <drawstuff/drawstuff.h>
using namespace std;
#include "simulation.h"

// ODE globals
dWorldID world;
dSpaceID space;
dJointGroupID contactgroup;
dGeomID ground;

double simulationTime = 0;
OdeConfig simulationConfig;
int sim_step = 0;
dsFunctions fn;
enum SimulationState { none, initialised, running, closed };
SimulationState state = none;

void (*configfunction)() = 0; // pointer to the config function of the user
void (*collisionCallback)(void* data, dGeomID o1, dGeomID o2) = 0;  // pointer to the user defined nearcallback function
void (*additionalDrawCallback)() = 0;  // pointer to the user defined additional draw function

// Object lists
ObstacleList obstacles;
AgentList agents;

// commandline functions see below
void cmd_handler_init();
bool control_c_pressed();
void cmd_begin_input();
void cmd_end_input();

// simulation stuff
void simLoop ( int pause );
void nearCallback(void *data, dGeomID o1, dGeomID o2);

void simulation_init(void (*start)(), void (*end)(), 
		     void (*config)(), void (*command)(int n)/* = 0 */ , void (*collCallback)(void* data,dGeomID o1, dGeomID o2)/* = 0 */,
		     void (*drawCallback)()/* = 0 */){
  configfunction=config; // store config function for simLoop
  collisionCallback=collCallback; // store config function for simLoop
  /**************************Grafikabschnitt**********************/
  fn.version = DS_VERSION;
  fn.start = start;
  fn.step = &simLoop;
  fn.command = command;
  fn.stop = end;
  fn.path_to_textures = "../../textures";

  /***************************ODE-Abschnitt***********************/
  //****************Weltdefinitionsabschnitt**************
  //Anlegen der Welt an sich
  world = dWorldCreate ();

  //Anlegen eines Raumes der Welt in der Sichtbare Koerper 
  // eine raeumliche Ausdehnung annehmen koennen
  // ist fuer die Kollissionserkennung wichtig
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate ( 1000000 );
 
  //Gravitation zu Erdgravitation
  dWorldSetGravity ( world , 0 , 0 , -9.81 );
  dWorldSetERP ( world , 1 );
  ground = dCreatePlane ( space , 0 , 0 , 1 , 0 );
  cmd_handler_init();
  state=initialised;
  additionalDrawCallback = drawCallback;
}

void simulation_start(int argc, char** argv){
  if(state!=initialised) return;
  //********************Simmulationsstart*****************
  state=running;
  dsSimulationLoop ( argc , argv , 500 , 500 , &fn );  
}

void simulation_close(){
  if(state!=running) return;
  //******Speicherfreigabe, Welt- und Raumzerstoerung*****
  dJointGroupDestroy ( contactgroup );
  dWorldDestroy ( world );
  dSpaceDestroy ( space );
  dCloseODE ();

  state=closed;
}

//Schleife der Simulation
void simLoop ( int pause )
{
  // Parametereingabe  
  if (control_c_pressed()){
    cmd_begin_input();
    if(configfunction) configfunction();
    cmd_end_input();
  }

  //die Simulation wird nur weitergefhrt wenn keine Pause aktiviert wurde
  if (!pause) {
    //**************************Steuerungsabschnitt ************************
    simulationTime += simulationConfig.simStepSize;
    
    sim_step = sim_step + 1;
    if ( (sim_step % simulationConfig.controlInterval ) == 0 ){
      for(AgentList::iterator i=agents.begin(); i != agents.end(); i++){
	(*i)->step(simulationConfig.noise);
      }
    }
  
    /**********************Simulationsschritt an sich**********************/
    dSpaceCollide ( space , 0 , &nearCallback );
    dWorldStep ( world , simulationConfig.simStepSize ); //ODE-Engine geht einen Schritt weiter
    dJointGroupEmpty (contactgroup);
    
  }  
  if(sim_step % simulationConfig.drawInterval == 0 || pause){
    if(additionalDrawCallback) additionalDrawCallback();
    /**************************Zeichenabschnitt***********************/
    for(ObstacleList::iterator i=obstacles.begin(); i != obstacles.end(); i++){
      (*i)->draw();
    }
    for(AgentList::iterator i=agents.begin(); i != agents.end(); i++){
      (*i)->getRobot()->draw();
    }
  }
}

//Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam
//Hier wird die Kollission n�er untersucht
// TODO call robots collisionCallback
void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  bool collision_treated=false;
  for(AgentList::iterator i=agents.begin(); i != agents.end() && !collision_treated; i++){
    collision_treated=(*i)->getRobot()->collisionCallback(data, o1, o2);
  }
  
  if (collision_treated) return;
  
  if(collisionCallback) {
    collisionCallback(data,o1,o2);
  }else{

    int i,n;  
    const int N = 10;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    if (n > 0) {
      for (i=0; i<n; i++)
	{
	  contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	    dContactSoftERP | dContactSoftCFM | dContactApprox1;
	  contact[i].surface.mu = 0.8; //normale Reibung von Reifen auf Asphalt
	  contact[i].surface.slip1 = 0.005;
	  contact[i].surface.slip2 = 0.005;
	  contact[i].surface.soft_erp = 1;
	  contact[i].surface.soft_cfm = 0.00001;
	  dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	
	}
    }
  }
}

// Helper
bool contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return true;
  }
  return false;
}


Position mkPosition(double x, double y, double z){
  Position pos={x, y, z};
  return pos;
};

Color mkColor(double r, double g, double b){
  Color color={r, g, b};
  return color;
};

// Commandline interface stuff
void showParams(const ConfigList& configs)
{
  paramkey* keys;
  paramval* vals;
  const unsigned short spacelength=20;
  char spacer[spacelength];
  memset(spacer, ' ', spacelength);  spacer[spacelength-1]=0;

  for(ConfigList::const_iterator i=configs.begin(); i != configs.end(); i++){
    int pnum = (*i)->getParamList(keys,vals);
    printf("Parameters of %s\n", (*i)->getName());
    for(int j=0; j < pnum; j++) {
      printf(" %s=%s%f\n", keys[j], 
	     spacer+(strlen(keys[j]) > spacelength  ? spacelength : strlen(keys[j])),vals[j]);
    }
    free(keys);
    free(vals);
  }
}

void changeParams(ConfigList& configs){
  char buffer[1024];
  std::cout << "Type: Parameter=Value\n";
  fgets( buffer, 1024, stdin);
  if ( strchr(buffer,'?')!=0){
    showParams(configs);
    return;
  }

  char *p = strchr(buffer,'=');
  if (p){
    *p=0; // terminate key string 
    double v=strtod(p+1,0);
    for(ConfigList::iterator i=configs.begin(); i != configs.end(); i++){
      if ((*i)->setParam(buffer,v))
	printf(" %s=\t%f \n", buffer, (*i)->getParam(buffer));
    }
  }
}

/// internals
int Control_C=0;

void control_c(int i){
  Control_C++ ;
  // if (Control_C > 100)exit(0);
}

void cmd_handler_exit(void){
  signal(SIGINT,SIG_DFL);
  Control_C=0;
}

void cmd_handler_init(){
  signal(SIGINT,control_c);
  atexit(cmd_handler_exit);
}

bool control_c_pressed(){
  return Control_C!=0;
}

void cmd_begin_input(){
  cmd_handler_exit();
}

void cmd_end_input(){
  cmd_handler_init();  
}




