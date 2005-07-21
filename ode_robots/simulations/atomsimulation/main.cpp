#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "noisegenerator.h"
#include "simulation.h"
#include "agent.h"
#include "one2onewiring.h"

#include "playground.h"
#include "closedplayground.h"

#include "invertnchannelcontroller.h"
#include "invertmotorspace.h"
#include "sinecontroller.h"

#include "noisegenerator.h"

#include "schlange.h"

#include "atomsimRobot.h"

//*****************************************************
vector<atomsimAtom*> atomsammlung;
vector<atomsimRobot*> robotersammlung;

int atomIDzaehler = 1;
int roboterIDzaehler = 1;

//*****************************************************
//world parameters
double playgroundx = 20;
double playgroundthickness = 0.2;
double playgroundheight = 10;

//*****************************************************
//evolutionary parameters

int lifecycle = 200; //this is the intervall of one lifecyle of an robot
int startingpopulationsize = 2; //it have to be at least two robots
int maxpopulationsize = 16;
int selektionsanzahl = 2; //this parameter says how many robots are selected form the former generation


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
  
  ClosedPlayground* playground = new ClosedPlayground ( world, space );
  playground->setGeometry ( playgroundx , playgroundthickness , playgroundheight );
  playground->setPosition ( 0 , 0 , 0 ); // playground positionieren und generieren
  obstacles.push_back ( playground );
  
  //*******robots and their atoms******
  
	int anzprozeile = (int) ( sqrt ( (double) maxpopulationsize ) );
	if ( anzprozeile < sqrt ( (double) maxpopulationsize ) )
		anzprozeile += 1;
	Position posA;
	
  for ( int n = 0; n < startingpopulationsize; n++)
  {
  	posA.x = ( 0 - 0.5 * playgroundx ) + ( n % anzprozeile + 0.5 ) * ( playgroundx / (maxpopulationsize / anzprozeile));
	posA.y = ( 0 - 0.5 * playgroundx ) + ( n / anzprozeile + 0.5 ) * ( playgroundx / (maxpopulationsize / anzprozeile));
	posA.z = 0;
  	dsPrint ( "x=%lf y=%lf z=%lf\n" , posA.x , posA.y ,posA.z );
  
	robotersammlung.push_back ( new atomsimRobot ( &roboterIDzaehler , world , space , contactgroup , &atomsammlung , new atomsimAtom ( roboterIDzaehler , &atomIDzaehler , world , space , posA.x + 0.0 , posA.y + 2.0 , posA.z + 1.0 , 0.3 , 0.5 , 1 , 1 , 15 ,  4/*Maxatombindungszahl*/ , 20/*getBindungsblockdauer*/ , 20.0/*Maxmotorkraft*/ , 40.0/*Motorgeschwindigkeitsfaktor*/ , 1.0 , 0.0 , 0.0 ) , 10 , 1.0/2  ) );
	atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  , posA.x + 0.2 , posA.y + 2, posA.z + 4 , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 0 , 1 , 0 ) );
	atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  ,  posA.x + 1 , posA.y + 2 , posA.z + 8 , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 0 , 0 , 1 ) );
	atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  , posA.x + 2.4 , posA.y + 2 , posA.z + 13 , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 1 , 1 , 0.0 ) );
	
	AbstractController *controller = new InvertMotorSpace ( 10 );
	One2OneWiring* wiring = new One2OneWiring( new ColorUniformNoise () );
	Agent* agent = new Agent( NoPlot/*GuiLogger*/ );
	agent->init(controller, robotersammlung.back () , wiring);
	agents.push_back(agent);
	configs.push_back(controller);
  
  } 
  
  
  //******free atoms*********
  for ( int x = 0; x < anzprozeile; x ++ )
  {
  	for ( int y = 0; y < anzprozeile; y ++ )
  	{
		atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  , x-0.1 , y-0.2 , 1 , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 0.2*x , 0.2*y , 0.2 ) );
		atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  , x+0.1 , y + 0.3 , 1 , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 0.3*x , 0.3*y , 0.3 ) );
		atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  , x-0.2 , y + 0.5 , 1 , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 0.4*x , 0.4*y , 0.4 ) );
		atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  , x+0.2 , y + 0.0 , 1 , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 0.4*x , 0.4*y , 0.4 ) );
		//atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  , x-0.2 , y - 0.1 , 1 , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 0.4*x , 0.4*y , 0.4 ) );
		//atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  , x-0.2 , y + 0.4 , 1 , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 0.4*x , 0.4*y , 0.4 ) );
	}
  }
  
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
    delete (*i)->getWiring ();
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
			   controller = new InvertMotorSpace ( 10 );
			   One2OneWiring* wiring;
			   wiring = new One2OneWiring( new ColorUniformNoise () );
			   Agent* agent;
			   agent = new Agent( NoPlot/*GuiLogger*/ );
			   
			   agent->init(controller, robotersammlung.back () , wiring );
  
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
			   controller3 = new InvertMotorSpace ( 10 );
			   One2OneWiring* wiring3;
			   wiring3 = new One2OneWiring ( new ColorUniformNoise () );
			   Agent* agent3;
			   agent3 = new Agent( NoPlot/*GuiLogger*/ );	   
			   agent3->init(controller3, robotersammlung.back () , wiring3 );
  
			   agents.push_back(agent3);
			   configs.push_back(controller3);
			   
			   
			   robotersammlung.push_back ( neuerRob2 );
			   
			   AbstractController* controller4;
			   controller4 = new InvertMotorSpace ( 10 );
			   One2OneWiring* wiring4;
			   wiring4 = new One2OneWiring( new ColorUniformNoise () );
			   Agent* agent4;
			   agent4 = new Agent( NoPlot/*GuiLogger*/);
			   agent4->init(controller4, robotersammlung.back () , wiring4 );
  
			   agents.push_back(agent4);
			   configs.push_back(controller4);
		break;
		
		case 'n' : atomsammlung.push_back ( new atomsimAtom ( 0 , &atomIDzaehler , world , space  , -12 , 0 , 1 , 0.3 , 0.5 , 1 , 1 , 20 , 4 , 20 , 20.0 , 40.0 , 0.5 , 0.2 , 0.8 ) );
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

void additionalLoopfunction ( bool draw , bool pause )
{
	
	//additional draw section
	if ( draw == 0 )
		for ( unsigned int n = 0; n < atomsammlung.size (); n++ )
			atomsammlung[n]->drawAtom ();

	//evolutionary section
	if ( pause == false )
	{
		//the simulationTime now is handled like an integer value
		if ( ( (int) ( simulationTime * ( 1 / simulationConfig.simStepSize ) ) ) % lifecycle == 0 )
		{
			//the pairs of the two fittest robots is recombined with each other
			
			//sorting: best robots first
			vector<atomsimRobot*> tmprobotersammlung;
			tmprobotersammlung.clear ();
			int tmprss = robotersammlung.size () / selektionsanzahl;
			while ( tmprss > maxpopulationsize/2 )
				tmprss--;
			if ( tmprss % 2 != 0 ) tmprss++;
			
			for ( int m = 0; m < tmprss; m++ )
			{
				vector<atomsimRobot*>::iterator it;
				vector<atomsimRobot*>::iterator it2;
				it = robotersammlung.begin ();
				it2 = robotersammlung.begin ();
				//for ( unsigned int i = 0; i < robotersammlung.size (); i++ )
				for ( it = robotersammlung.begin (); it != robotersammlung.end(); it++ )
				{
					
					if ( (*it)->getFitness () > (*it2)->getFitness () )
					{
						it2 = it;
					}
				}
				
				tmprobotersammlung.push_back ( (*it2) );
				
				robotersammlung.erase ( it2 );
			}
			//deletation of the bad (non fit) robots
				
			for ( unsigned int n = 0; n < robotersammlung.size (); robotersammlung[n++]->~atomsimRobot () );
				
			robotersammlung.clear ();
			
			robotersammlung = tmprobotersammlung;
			
			if ( robotersammlung.size () % 2 != 0 )
				dsPrint ( "Pupulation size is wrong->the half of it has to be an even number!\n" );
			else
			{	
				
				//recombination
				int tmprobotersammlungsize = robotersammlung.size ();
				for ( int n = 0; ( n + 1 ) < tmprobotersammlungsize; n = n + 2 )
				{
					
					atomsimRobot* neuerRob1;
					atomsimRobot* neuerRob2;
					Position posA , posB , posC , posD;
					int anzprozeile = (int) ( sqrt ( (double) maxpopulationsize ) );
					if ( anzprozeile < sqrt ( (double) maxpopulationsize ) )
						anzprozeile += 1;
								
					//positions for the old robots
					posA.x = ( 0 - 0.5 * playgroundx ) + ( n % anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
					posA.y = ( 0 - 0.5 * playgroundx ) + ( n / anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
					posA.z = playgroundheight/2;
					
					posB.x = ( 0 - 0.5 * playgroundx ) + ( (n+1) % anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
					posB.y = ( 0 - 0.5 * playgroundx ) + ( (n+1) / anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
					posB.z = playgroundheight/2;
					//placing of the old robots
					robotersammlung[n]->rekursivVerschieben ( robotersammlung[n]->getUrsprungsatom () , posA );
					robotersammlung[n+1]->rekursivVerschieben ( robotersammlung[n+1]->getUrsprungsatom () , posB );
					
					
					//positions for the new robots
					posC.x = ( 0 - 0.5 * playgroundx ) + ( (n+2) % anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
					posC.y = ( 0 - 0.5 * playgroundx ) + ( (n+2) / anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
					posC.z = playgroundheight/2;
					
					posD.x = ( 0 - 0.5 * playgroundx ) + ( (n+3) % anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
					posD.y = ( 0 - 0.5 * playgroundx ) + ( (n+3) / anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
					posD.z = playgroundheight/2;
					
					robotersammlung[n]->roboterRekombination ( 0 , 1.0/2 , robotersammlung [n+1] , &neuerRob1 , &neuerRob2 , posC , posD );
						
					robotersammlung.push_back ( neuerRob1 );
					robotersammlung.push_back ( neuerRob2 );
				}
				
				//deletes all agents, controllers and wirings, which are not linked to an robot from robotersammlung
				vector<Agent*>::iterator agentit = agents.begin ();
				//for ( unsigned int m = 0; m < agents.size (); m++ )
				for ( vector<Agent*>::iterator agentit = agents.begin (); agentit != agents.end (); agentit++ )
				{
					bool del = true;
					for ( vector<atomsimRobot*>::iterator robotit = robotersammlung.begin (); robotit != robotersammlung.end (); robotit++ )
					{
					
						if ( (*agentit)->getRobot () == (*robotit) )
						{
							dsPrint ( "Agent nicht gelöscht.\n" );
							del = false;
							break;
						}	
					}
					if ( del )
					{
						dsPrint ( "Agent wird gelöscht.\n" );
						
						
						dsPrint ( "Agents:%i Robots:%i\n" , agents.size (), robotersammlung.size () );
						delete (*agentit)->getController();
						delete (*agentit)->getWiring ();
						delete (*agentit);
						
						agents.erase ( agentit );
						//because the loop now is repeated fewer
						agentit--;
					}
				}
				//creates new agents, controllers and wirings if a robot is not linked to an agent
				
				for ( unsigned int n = 0; n < robotersammlung.size (); n++ )
				{
					bool create = true;
					for ( unsigned int m = 0; m < agents.size (); m++ )
					{
						if ( agents[m]->getRobot () == robotersammlung[n] )
						{
							create = false;
							break;
						}
					}
					if ( create )
					{
						dsPrint ( "ANLEGEN EINES AGENTS!\n" );
						AbstractController* controller;
						controller = new InvertMotorSpace ( 10 );
						One2OneWiring* wiring;
						wiring = new One2OneWiring ( new ColorUniformNoise () );
						Agent* agent;
						agent = new Agent( NoPlot/*GuiLogger*/ );
					
						agent->init(controller, robotersammlung[n] , wiring );
						agents.push_back(agent);
						configs.push_back(controller);
					}
				}
				
				dsPrint ("Eine neue Generation entsteht!\n");
			}
			
		}
		//adaptation of the fitness value of the robots
		else
		{
			for ( unsigned int n = 0; n < robotersammlung.size (); n ++ )
			{
				robotersammlung[n]->setFitness ( 1.0/(robotersammlung[n]->getAtomAnzahl ()) );
			}
		}
	dsPrint ("Weltzeit: %lf\n" , simulationTime );
	}
}


int main (int argc, char **argv)
{  
  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config, &command , &atomCallback , &additionalLoopfunction );
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
