#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"
#include "simulation.h"
#include "one2oneagent.h"
#include "playground.h"

#include "invertnchannelcontroller.h"
#include "noisegenerator.h"

#include "schlange.h"

#include "atomsimRobot.h"

//*****************************************************
vector<atomsimAtom*> atomsammlung;
vector<atomsimRobot*> robotersammlung;

int atomIDzaehler = 1;
int roboterIDzaehler = 1;


//*****************************************************

ConfigList configs;



// Funktion die die Steuerung des Roboters uebernimmt
bool StepRobot()
{
  for(AgentList::iterator i=agents.begin(); i != agents.end(); i++){
    (*i)->step(simulationConfig.noise);
  }
  return true;
}

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start() 
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  simulationConfig.noise=0.1;
  
  Playground* playground = new Playground(&world, &space);
  playground->setGeometry(10.0, 0.2, 2.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  obstacles.push_back(playground);
  
  //*******robots and their atoms******
  robotersammlung.push_back ( new atomsimRobot ( &roboterIDzaehler , &world , &space , &contactgroup , &atomsammlung , new atomsimAtom ( roboterIDzaehler , &atomIDzaehler , &world , &space , 0.0 , 0.0 , 1.0 , 0.3 , 0.5 , 1 , 1 , 20 ,  4/*Maxatombindungszahl*/ , 20/*getBindungsblockdauer*/ , 20.0/*Maxmotorkraft*/ , 40.0/*Motorgeschwindigkeitsfaktor*/ , 1.0 , 0.0 , 0.0 ) , 10 , 1.0/2  ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 0.2 , 0 , 4 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0 , 1 , 0 ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  ,  1 , 0 , 8 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0 , 0 , 1 ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 2.4 , 0 , 13 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 1 , 1 , 0.0 ) );
  
  AbstractController *controller = new InvertNChannelController(10);
  One2OneAgent* agent = new One2OneAgent( new ColorUniformNoise () , NoPlot/*GuiLogger*/ );
  agent->init(controller, robotersammlung.back () );
  agents.push_back(agent);
  configs.push_back(controller);
  
  
  robotersammlung.push_back ( new atomsimRobot ( &roboterIDzaehler , &world , &space , &contactgroup , &atomsammlung , new atomsimAtom ( roboterIDzaehler , &atomIDzaehler , &world , &space , 0.0 , 2.0 , 1.0 , 0.3 , 0.5 , 1 , 1 , 20 ,  4/*Maxatombindungszahl*/ , 20/*getBindungsblockdauer*/ , 20.0/*Maxmotorkraft*/ , 40.0/*Motorgeschwindigkeitsfaktor*/ , 1.0 , 0.0 , 0.0 ) , 10 , 1.0/2  ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 0.2 , 2.0 , 3.0 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0 , 1 , 0 ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  ,  1 , 2.0 , 6 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0 , 0 , 1 ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 1.8 , 2.0 , 9 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 1 , 1 , 0.0 ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 2.2 , 2.0 , 12 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0.0 , 1 , 1.0 ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 2.4 , 2.0 , 15 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 1 , 0.0 , 1.0 ) );
  
  AbstractController *controller2 = new InvertNChannelController(10);
  One2OneAgent* agent2 = new One2OneAgent( new ColorUniformNoise () , NoPlot/*GuiLogger*/ );
  agent2->init(controller2, robotersammlung.back () );
  agents.push_back(agent2);
  configs.push_back(controller2);  
  
  
  
  
  //******free atoms*********
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 2 , -2 , 1 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0.2 , 0.2 , 0.2 ) );	
  
  //****************  
  configs.push_back(&simulationConfig);
  //****************  
  
  

  //************
  
  showParams(configs);
}

void end(){
  for(ObstacleList::iterator i=obstacles.begin(); i != obstacles.end(); i++){
    delete (*i);
  }
  obstacles.clear();
  for(AgentList::iterator i=agents.begin(); i != agents.end(); i++){
    delete (*i)->getRobot();
    delete (*i)->getController();
    delete (*i);
  }
  agents.clear();
}


// this function is called if the user pressed Ctrl-C
void config(){
  changeParams(configs);
}

//Funktion die eingegebene Befehle/kommandos verarbeitet
void command (int cmd)
{
	//dsPrint ( "Eingabe erfolgt %d (`%c')\n" , cmd , cmd );
	switch ( (char) cmd )
	{
		case 'h' :	dsPrint ( "\n\n-------------------------------------------Help---------------------------------------\n" );
				dsPrint ( "y= Adding a konstant force to the black atom, so that it will collide\n" );
			break;
		case 'y' : dBodyAddForce ( (*atomsammlung.back()).getBody() , 0 , 100 , 0 ); break;
		case 'v' : Position newpos;
			   newpos.x = -10;
			   newpos.y = 0;
			   newpos.z = robotersammlung.back ()->getUrsprungsatom ()->getZ ();
			   
			   Color newcolor;
			   newcolor.r = robotersammlung.back ()->getUrsprungsatom ()->getColorR ();
			   newcolor.g = robotersammlung.back ()->getUrsprungsatom ()->getColorG ();;
			   newcolor.b = robotersammlung.back ()->getUrsprungsatom ()->getColorB ();;
			
			   robotersammlung.back ()->place ( newpos , &newcolor);
		break;
		
		case 'k' : robotersammlung.push_back ( robotersammlung.back ()->rekursivKopieren ( robotersammlung.back ()->getUrsprungsatom () , true ) );
		
			   Position newpos2;
			   newpos2.x = -10;
			   newpos2.y = 0;
			   newpos2.z = robotersammlung.back ()->getUrsprungsatom ()->getZ ();
			   
			   Color newcolor2;
			   newcolor2.r = robotersammlung.back ()->getUrsprungsatom ()->getColorR ();
			   newcolor2.g = robotersammlung.back ()->getUrsprungsatom ()->getColorG ();;
			   newcolor2.b = robotersammlung.back ()->getUrsprungsatom ()->getColorB ();;
			
			   robotersammlung.back ()->place ( newpos2 , &newcolor2);
			   
			   
			   
			   AbstractController* controller;
			   controller = new InvertNChannelController(10);
			   One2OneAgent* agent;
			   agent = new One2OneAgent( new ColorUniformNoise () , NoPlot/*GuiLogger*/);
			   agent->init(controller, robotersammlung.back () );
  
			   agents.push_back(agent);
			   configs.push_back(controller);
		break;
		
		case 't' : robotersammlung.back()->roboterAuftrennen (); 
		
		break;
		
		case 'r' : 
			   atomsimRobot* neuerRob1;
			   atomsimRobot* neuerRob2;
			   Position posA;
			   posA.x = 0;
			   posA.y = 20;
			   posA.z = 10;
			   Position posB;
			   posB.x = 0;
			   posB.y = -20;
			   posB.z = 10;
			   robotersammlung[0]->roboterRekombination ( 0 , 1.0/2 , robotersammlung [1] , &neuerRob1 , &neuerRob2 , posA , posB );
			   
			   
			   robotersammlung.push_back ( neuerRob1 );
			   
			   AbstractController* controller3;
			   controller3 = new InvertNChannelController(10);
			   One2OneAgent* agent3;
			   agent3 = new One2OneAgent( new ColorUniformNoise () , NoPlot/*GuiLogger*/);
			   agent3->init(controller3, robotersammlung.back () );
  
			   agents.push_back(agent3);
			   configs.push_back(controller3);
			   
			   
			   robotersammlung.push_back ( neuerRob2 );
			   
			   AbstractController* controller4;
			   controller4 = new InvertNChannelController(10);
			   One2OneAgent* agent4;
			   agent4 = new One2OneAgent( new ColorUniformNoise () , NoPlot/*GuiLogger*/);
			   agent4->init(controller4, robotersammlung.back () );
  
			   agents.push_back(agent4);
			   configs.push_back(controller4);
		break;
		
		case 'n' : atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , -12 , 0 , 1 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0.5 , 0.2 , 0.8 ) );
		break;
		
	}
}

//Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam
//Hier wird die Kollission n�er untersucht
void atomCallback (void *data, dGeomID o1, dGeomID o2)
{
	int collision;
	unsigned int n , m;
	bool huellenkollision = false;

	const int N = 10;
	dContact contact[N];
	collision = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));

	for ( n = 0; n < atomsammlung.size (); n++ )
	{
		for ( m = 0; m < atomsammlung.size (); m++ )
		{
			//testet ob zwei Atomhuellen collidiert sind
			if ( (o1 == (*atomsammlung[n] ).getAtomhuelleGeom () ) &&( o2 == (*atomsammlung[m] ).getAtomhuelleGeom () ) )		
				huellenkollision = true;

			if ( huellenkollision == true ) break;
		}
		if ( huellenkollision == true ) break;
	}
	if  ( ( ( huellenkollision == true ) )
	&& ( (*atomsammlung[n]).getRoboterID () == (*atomsammlung[m]).getRoboterID () ) )
	{
		//dann erfolgt gar nichts, als wie wenn die Huelle nicht existent waere, so koenen Atome des selben Roboters nicht miteinander verschmelzen oder abspalten, es kommt nur zur den normalen Atomkollisionen, bei denen aber nur eine normale Kollision erfolgt, da es keine Huellenkollision ist
	}	
	else	
		if (collision > 0)
		{
			if ( huellenkollision == false )
				for ( int i=0; i<collision; i++)
				{
				
					contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
					dContactSoftERP | dContactSoftCFM | dContactApprox1;
					contact[i].surface.mu = 0.8; //normale Reibung von Reifen auf Asphalt = 0.8
					contact[i].surface.slip1 = 0.0051;
					contact[i].surface.slip2 = 0.0051;
					contact[i].surface.soft_erp = 1;
					contact[i].surface.soft_cfm = 0.00001; //Elastizität der Stoesse: klein keine 							Elastizität, groß viel Elsatizität
					dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
					dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
					
				}
		
			if ( huellenkollision == true )
			{
				//wenn bis hierher alle Bedingungen erfüllt sind, dann sind zwei Atome kollisiert, die in der Atomsammlung vorhanden sind
	
				//nur Kollision wenn beide Atome verschiedene RoboterIDs haben und eines nicht 0 und das andere 0 ist
				if (
					( ( (*atomsammlung[n]).getRoboterID () != 0) && ( (*atomsammlung[m]).getRoboterID () == 0 ) )
					||
					( ( (*atomsammlung[n]).getRoboterID () == 0) && ( (*atomsammlung[m]).getRoboterID () != 0 ) )
				   )
				{
					//Kollisionsaufruffe duerfen nur fuer die Roboteratome aufgerufen werden
					//Test von o1-> o1 ist einzelnes Atom
					if ( (*atomsammlung[n]).getRoboterID () == 0 )
						(*atomsammlung[m]).kollision ( atomsammlung[n] );
					// sonst ist o2 das einzelne Atom
					else
						(*atomsammlung[n]).kollision ( atomsammlung[m] );
				}
			}
		
		}
}

void atomDraw ()
{
	for ( unsigned int n = 0; n < atomsammlung.size (); n++ )
		atomsammlung[n]->drawAtom ();
}


int main (int argc, char **argv)
{  
  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config, &command , &atomCallback , &atomDraw );
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
