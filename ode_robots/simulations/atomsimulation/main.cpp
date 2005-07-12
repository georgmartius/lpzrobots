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
  playground->setGeometry(7.0, 0.2, 1.5);
  playground->setPosition(0,0,0); // playground positionieren und generieren
  obstacles.push_back(playground);
    
  robotersammlung.push_back ( new atomsimRobot ( &roboterIDzaehler , &world , &space , &contactgroup , &atomsammlung , new atomsimAtom ( roboterIDzaehler , &atomIDzaehler , &world , &space , 0.0 , 0.0 , 1.0 , 0.3 , 0.5 , 1 , 1 , 20 ,  4/*Maxatombindungszahl*/ , 20/*getBindungsblockdauer*/ , 20.0/*Maxmotorkraft*/ , 40.0/*Motorgeschwindigkeitsfaktor*/ , 1.0 , 0.0 , 0.0 ) , 10 , 1.0/2  ) );
  
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 0.2 , 0 , 4 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0 , 1 , 0 ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  ,  1 , 0 , 8 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0 , 0 , 1 ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 2.4 , 0 , 13 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 1 , 1 , 0.0 ) );
  atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , &world , &space  , 2 , -2 , 1 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0.2 , 0.2 , 0.2 ) );
	
  
  AbstractController *controller = new InvertNChannelController(10);
  
  One2OneAgent* agent = new One2OneAgent( new ColorUniformNoise () , /*NoPlot*/GuiLogger);
  agent->init(controller, robotersammlung[0] );
  
  agents.push_back(agent);
  
  
  
    
  //****************  
  configs.push_back(&simulationConfig);
  //****************  
  
  configs.push_back(controller);

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
  simulation_init(&start, &end, &config, &atomCallback , &atomDraw );
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
