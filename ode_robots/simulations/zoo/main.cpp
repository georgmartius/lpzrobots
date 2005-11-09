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
 *   Revision 1.10  2005-11-09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include "agent.h"
#include "noisegenerator.h"

#include "invertnchannelcontroller.h"
#include "invertmotorspace.h"
#include "invertmotornstep.h"
#include "one2onewiring.h"
#include "derivativewiring.h"

#include "playground.h"
#include "sphere.h"

#include "hurlingsnake.h"
#include "schlangeforce.h"
#include "nimm2.h"
#include "nimm4.h"

// for video capturing use:
// realtimefactor=0.5
// drawinterval=5

PlotMode plotMode = NoPlot;

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  global.odeConfig.setParam("gravity",-9.81); // do not use 'global.odeConfig.gravity=0.0;', 
                                            // because world is already initialized and 
                                            // dWorldSetGravity will not be called when 
                                            // you only set the value  
  int chessTexture = dsRegisterTexture("chess.ppm");
  printf("Chess: %i", chessTexture);
  int dust = dsRegisterTexture("dusty.ppm");
  printf("Chess: %i", chessTexture);


  //Anfangskameraposition und Punkt auf den die Kamera blickt
  //float KameraXYZ[3]= {0.276f,7.12f,1.78f};
  //float KameraViewXYZ[3] = {-88.0f,-5.5f,0.0000f};
  float KameraXYZ[3]= {-0.0f, 10.4f, 6.8f};
  float KameraViewXYZ[3] = {-90.0f,-40.0f,0.0000f};

  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  global.odeConfig.setParam("noise",0.1);
  global.odeConfig.setParam("drawinterval",1);
  global.odeConfig.setParam("controlinterval",1);
  // initialization
  
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(25.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);

  Sphere* sphere;
  for (int i=-3; i<3; i++){
    sphere = new Sphere(odeHandle);
    sphere->setColor(184 / 255.0, 233 / 255.0, 237 / 255.0);
    sphere->setTexture(dust);
    sphere->setPosition(i*0.5-2,i*0.5,1.0); //positionieren und generieren

    global.obstacles.push_back(sphere);
  }

  

  Agent* agent;
  AbstractWiring* wiring;
  AbstractRobot* robot;
  AbstractController *controller;

  SchlangeForce* snake;
  SchlangenConf snakeConf = SchlangeForce::getDefaultConf();
  //******* S C H L A N G E  (Short) *********/
  snakeConf.armAnzahl=4;
  snakeConf.maxWinkel=M_PI/3;
  snakeConf.frictionGround=0.1;
  snakeConf.factorForce=0.6; //3;
  snakeConf.factorSensors=5;
  snake = new SchlangeForce ( 1 , odeHandle, snakeConf );
  {
    Color col(0,0.5,0.8);
    snake->place(Position(-5,-5,0),&col); 
  }
  controller = new InvertMotorNStep(10);  
  wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  agent = new Agent( plotMode );
  agent->init(controller, snake, wiring);
  global.agents.push_back(agent);
  global.configs.push_back(controller);
  global.configs.push_back(snake);   
  snake->setParam("gamma",/*0.0000*/ 0.0);
  

  //******* S C H L A N G E  (Long)  *********/
  snakeConf = SchlangeForce::getDefaultConf();
  snakeConf.armAnzahl   = 8;
  snakeConf.maxWinkel   = M_PI/3;
  snakeConf.frictionGround=0.1;
  snakeConf.factorForce=1 ;//3.5;
  snakeConf.factorSensors=5;
  snake = new SchlangeForce ( 2 , odeHandle, snakeConf );
  {
    Color col(0,0.5,0.8);
    snake->place(Position(0,0,0),&col); 
  }
  controller = new InvertMotorNStep(10);  
  wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  agent = new Agent( NoPlot );
  agent->init(controller, snake, wiring);
  global.agents.push_back(agent);
  global.configs.push_back(controller);
  global.configs.push_back(snake);   
  snake->setParam("gamma",/*0.0000*/ 0.0);
  
  Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
  nimm2conf.size = 1.6;

  //******* N I M M  2 *********/
  for(int r=0; r < 3; r++) {    
    robot = new Nimm2(odeHandle, nimm2conf);
    robot->place(Position ((r-1)*5,5,0));
    ((Nimm2*)robot)->setTextures(DS_WOOD, chessTexture);
    controller = new InvertMotorNStep(10);   
    //    controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent = new Agent( NoPlot );
    agent->init(controller, robot, wiring);
    global.configs.push_back(controller);
    global.agents.push_back(agent);        
  }

  //******* N I M M  4 *********/
  for(int r=0; r < 1; r++) {
    robot = new Nimm4(odeHandle);
    Position p((r-1)*5,-3,0);
    robot->place(p);
    ((Nimm4*)robot)->setTextures(DS_WOOD, chessTexture);
    controller = new InvertMotorSpace(20);
    controller->setParam("s4avg",10); 
    controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent = new Agent( NoPlot );
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);        
  }

  //****** H U R L I N G **********/
  for(int r=0; r < 2; r++) {
    HurlingSnake* snake;
    snake = new HurlingSnake(odeHandle);
    Color c;    
    if (r==0) c=Color(0.8, 0.8, 0);
    if (r==1) c=Color(0,   0.8, 0);
    snake->place(Position(r*5,-6,0.3), &c);
    snake->setParam("factorForce",8);

    controller = new InvertMotorNStep(10);   
    // deriveconf = DerivativeWiring::getDefaultConf();
//     deriveconf.blindMotorSets=0;
//     deriveconf.useId = true;
//     deriveconf.useFirstD = true;
//     deriveconf.derivativeScale = 50;
//     wiring = new DerivativeWiring(deriveconf, new ColorUniformNoise(0.1));
    wiring = new One2OneWiring(new ColorUniformNoise(0.05));
    agent = new Agent( NoPlot );
    agent->init(controller, snake, wiring);
    global.configs.push_back(controller);
    global.agents.push_back(agent);     
  }
  showParams(global.configs);
}

void end(GlobalData& global){
  for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++){
    delete (*i);
  }
  global.obstacles.clear();
  for(AgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++){
    delete (*i)->getRobot();
    delete (*i)->getController();
    delete (*i);
  }
  global.agents.clear();
}


// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){
  changeParams(global.configs);
}

void printUsage(const char* progname){
  printf("Usage: %s [-g] [-l] [-r seed]\n\t-g\tuse guilogger\n\t-l\tuse guilogger with logfile\n\t-r seed\trandom number seed ", progname);
}

int main (int argc, char **argv)
{  
  if(contains(argv, argc, "-g")) plotMode = GuiLogger;
  if(contains(argv, argc, "-l")) plotMode = GuiLogger_File;
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config);
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
 
