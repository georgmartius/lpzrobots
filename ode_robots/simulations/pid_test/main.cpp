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
 *   Revision 1.7.4.2  2006-06-25 17:01:55  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.7.4.1  2005/11/15 12:29:51  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.7  2005/11/09 13:43:17  fhesse
 *   GPL added
 *                                                                 *
 *                                                                         * 
 ***************************************************************************/
#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"
#include <selforg/noisegenerator.h>
#include "odeagent.h"
#include <selforg/one2onewiring.h>
#include "playground.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>

#include "sphererobot.h"
#include "sphererobotTest.h"

ConfigList configs;
list<PlotOption> plotoptions;
AbstractController *controller;

SphererobotTest* sphere1;


//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global) 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (3); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.setParam("noise",0);
  global.odeConfig.setParam("gravity", -9.81 );
  global.odeConfig.setParam("controlinterval", 1 );
  

    
  Playground* playground = new Playground(odeHandle);
  playground->setGeometry(7, 0.2, 1);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  global.obstacles.push_back(playground);
    
  
  //****************
  SphererobotConf conf = Sphererobot::getStandartConf();  
  conf.maxforce=0;
  conf.slidermass=0.0001;
  conf.sliderrange=0.1;
  conf.spheremass=1; 
  //sphere1 = new Sphererobot ( 1 , ODEHandle(world , space , contactgroup), conf);
  sphere1 = new SphererobotTest ( 1 , odeHandle, conf);
  Color col(0,0.5,0.8);
  sphere1->place ( Position ( 0 , 0 , 0 ) , &col );
  //AbstractController *controller = new InvertNChannelController(10);  
  controller = new SineController();  
  controller->setParam("sineRate", 20);  
  controller->setParam("phaseShift", 0.8);

  One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
  OdeAgent* agent = new OdeAgent ( plotoptions );
  agent->init ( controller , sphere1 , wiring );
  global.agents.push_back ( agent ); 
  configs.push_back ( controller );
      
  showParams(configs);
}

void end(GlobalData& global){
  for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++){
    delete (*i);
  }
  global.obstacles.clear();
  for(OdeAgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++){
    delete (*i)->getRobot();
    delete (*i)->getController();
    delete (*i);
  }
  global.agents.clear();
}


// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){
  changeParams(configs);
}

//Funktion die eingegebene Befehle/kommandos verarbeitet
void command (const OdeHandle&, GlobalData& globalData, int cmd)
{
  //dsPrint ( "Eingabe erfolgt %d (`%c')\n" , cmd , cmd );
  switch ( (char) cmd )
    {
    case 'y' : dBodyAddForce ( sphere1->getObjektAt (Sphererobot::Pendular).body , 0 ,0 , 100 ); break;
    case 'a' : dBodyAddForce ( sphere1->getObjektAt (Sphererobot::Pendular).body , 0 , 0 , 100 ); break;
    case 'x' : dBodyAddTorque ( sphere1->getObjektAt ( Sphererobot::Pendular ).body , 0 , 0 , 2 ); break;
    case 'c' : dBodyAddTorque ( sphere1->getObjektAt ( Sphererobot::Pendular ).body , 0 , 0 , -2 ); break;
    case 'S' : controller->setParam("sineRate", controller->getParam("sineRate")-0.5); 
      printf("sineRate : %g\n", controller->getParam("sineRate"));
      break;
    case 's' : controller->setParam("sineRate", controller->getParam("sineRate")+0.5); 
      printf("sineRate : %g\n", controller->getParam("sineRate"));
      break;
    case 'P' : sphere1->servo->KP+=5; printf("KP : %g\n", sphere1->servo->KP); break;
    case 'p' : sphere1->servo->KP-=5; printf("KP : %g\n", sphere1->servo->KP); break;
    case 'D' : sphere1->servo->KD*=1.01; printf("KD : %g\n", sphere1->servo->KD); break;
    case 'd' : sphere1->servo->KD*=0.99; printf("KD : %g\n", sphere1->servo->KD); break;
    case 'I' : sphere1->servo->KI*=1.01; printf("KI : %g\n", sphere1->servo->KI); break;
    case 'i' : sphere1->servo->KI*=0.99; printf("KI : %g\n", sphere1->servo->KI); break;
    }
}
 

int main (int argc, char **argv)
{  
  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config, &command , 0 , 0 );
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}

 
