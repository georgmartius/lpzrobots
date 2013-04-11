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
 *   Revision 1.16  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.15  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.14  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.13  2006/07/14 12:23:45  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.12.4.1  2005/11/15 12:29:37  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.12  2005/11/09 13:37:23  fhesse
 *   GPL added
 *
 *   Revision 1.7  2005/11/09 13:28:24  fhesse
 *   GPL added
 *                                                                *
 ***************************************************************************/
#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode-dbl/ode.h>

#include <selforg/noisegenerator.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>

//#include <ode_robots/playground.h>
#include <ode_robots/closedplayground.h>

//#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
//#include <selforg/sinecontroller.h>

#include <selforg/noisegenerator.h>

//#include <ode_robots/schlange.h>

#include <ode_robots/atomsimRobot.h>

//*****************************************************
vector<atomsimAtom*> atomsammlung;
vector<atomsimRobot*> robotersammlung;

int atomIDzaehler = 1;
int roboterIDzaehler = 1;

//*****************************************************
//world parameters
double playgroundx = 20;
double playgroundthickness = 1;
double playgroundheight =15;


//*****************************************************
//evolutionary parameters

int lifecycle = 500; //this is the intervall of one lifecyle of an robot
int startingpopulationsize = 4; //it have to be at least two robots
int maxpopulationsize = 16;
int selektionsanzahl = 2; //this parameter says how many robots are selected form the former generation

//variables for special fitness functions
Position evoarray[16];
//*****************************************************

//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start(const OdeHandle& odeHandle, GlobalData& global)
{
  dsPrint ( "\nWelcome to the virtual ODE - robot simulator of the Robot Group Leipzig\n" );
  dsPrint ( "------------------------------------------------------------------------\n" );
  dsPrint ( "Press Ctrl-C for an basic commandline interface.\n\n" );

  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ, KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  global.odeConfig.noise=0.1;

  ClosedPlayground* playground = new ClosedPlayground ( odeHandle );
  playground->setGeometry ( playgroundx, playgroundthickness, playgroundheight );
  playground->setPosition ( 0, 0, 0 ); // playground positionieren und generieren
  global.obstacles.push_back ( playground );

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
      dsPrint ( "x=%lf y=%lf z=%lf\n", posA.x, posA.y,posA.z );

      robotersammlung.push_back ( new atomsimRobot ( &roboterIDzaehler, odeHandle, &atomsammlung, new atomsimAtom ( roboterIDzaehler, &atomIDzaehler, odeHandle, posA.x + 0.0, posA.y + 2.0, posA.z + 1.0, 0.3, 0.5, 1, 1, 15,  4/*Maxatombindungszahl*/, 20/*getBindungsblockdauer*/, 20.0/*Maxmotorkraft*/, 40.0/*Motorgeschwindigkeitsfaktor*/, 1.0, 0.0, 0.0 ), 10, 1.0/2  ) );
      atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle, posA.x + 0.2, posA.y + 2, posA.z + 4, 0.3, 0.5, 1, 1, 15, 4, 20, 20.0, 40.0, 0, 1, 0 ) );
      atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle,  posA.x + 1, posA.y + 2, posA.z + 8, 0.3, 0.5, 1, 1, 15, 4, 20, 20.0, 40.0, 0, 0, 1 ) );
      atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle, posA.x + 2.4, posA.y + 2, posA.z + 13, 0.3, 0.5, 1, 1, 15, 4, 20, 20.0, 40.0, 1, 1, 0.0 ) );

      AbstractController *controller = new InvertMotorSpace ( 10 );
      One2OneWiring* wiring = new One2OneWiring( new ColorUniformNoise () );
      OdeAgent* agent = new OdeAgent( global, PlotOption(NoPlot)/*GuiLogger*/ );
      agent->init(controller, robotersammlung.back (), wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);

    }


  //******free atoms*********
  for ( int x = 0; x < anzprozeile; x ++ )
    {
      for ( int y = 0; y < anzprozeile; y ++ )
          {
          atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle , x-0.1, y-0.2, 1, 0.3, 0.5, 1, 1, 15, 4, 20, 20.0, 40.0, 0.2*x, 0.2*y, 0.2 ) );
          atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle , x+0.1, y + 0.3, 1, 0.3, 0.5, 1, 1, 15, 4, 20, 20.0, 40.0, 0.3*x, 0.3*y, 0.3 ) );
          //atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle , x-0.2, y + 0.5, 1, 0.3, 0.5, 1, 1, 15, 4, 20, 20.0, 40.0, 0.4*x, 0.4*y, 0.4 ) );
          //atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle , x+0.2, y + 0.0, 1, 0.3, 0.5, 1, 1, 15, 4, 20, 20.0, 40.0, 0.4*x, 0.4*y, 0.4 ) );
          //atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle , x-0.2, y - 0.1, 1, 0.3, 0.5, 1, 1, 15, 4, 20, 20.0, 40.0, 0.4*x, 0.4*y, 0.4 ) );
          //atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle , x-0.2, y + 0.4, 1, 0.3, 0.5, 1, 1, 15, 4, 20, 20.0, 40.0, 0.4*x, 0.4*y, 0.4 ) );
        }
    }

  //****************
  //****************



  //************


}

void end(GlobalData& global){
  for(ObstacleList::iterator i=global.obstacles.begin(); i != global.obstacles.end(); i++){
    delete (*i);
  }
  global.obstacles.clear();
  for(OdeAgentList::iterator i=global.agents.begin(); i != global.agents.end(); i++){
    delete (*i)->getRobot();
    delete (*i)->getController();
    delete (*i)->getWiring ();
    delete (*i);
  }
  global.agents.clear();
}


// this function is called if the user pressed Ctrl-C
void config(GlobalData& global){
  changeParams(global.configs);
}

//Funktion die eingegebene Befehle/kommandos verarbeitet
void command (const OdeHandle& odeHandle, GlobalData& global, int cmd)
{
  //dsPrint ( "Eingabe erfolgt %d (`%c')\n", cmd, cmd );
  switch ( (char) cmd )
    {
    case 'h' :        dsPrint ( "\n\n-------------------------------------------Help---------------------------------------\n" );
      dsPrint ( "y= Adding a konstant force to the black atom, so that it will collide\n" );
      break;
    case 'y' : dBodyAddForce ( (*atomsammlung.back()).getBody(), 0, 100, 0 ); break;
    case 'v' :
      robotersammlung.back ()->place ( Position ( -10, 0, robotersammlung.back ()->getUrsprungsatom ()->getZ () ), &Color ( robotersammlung.back ()->getUrsprungsatom ()->getColorR (),
                                                                                                                            robotersammlung.back ()->getUrsprungsatom ()->getColorG (),
                                                                                                                            robotersammlung.back ()->getUrsprungsatom ()->getColorB () ) );
      break;

    case 'k' : robotersammlung.push_back ( robotersammlung.back ()->rekursivKopieren ( robotersammlung.back ()->getUrsprungsatom (), true ) );

      robotersammlung.back ()->place (
                                      Position ( -10, 0, robotersammlung.back ()->getUrsprungsatom ()->getZ () ), &Color ( robotersammlung.back ()->getUrsprungsatom ()->getColorR (),
                                                                                                                           robotersammlung.back ()->getUrsprungsatom ()->getColorG (),
                                                                                                                           robotersammlung.back ()->getUrsprungsatom ()->getColorB () ) );



      AbstractController* controller;
      controller = new InvertMotorSpace ( 10 );
      One2OneWiring* wiring;
      wiring = new One2OneWiring( new ColorUniformNoise () );
      OdeAgent* agent;
      agent = new OdeAgent( global, PlotOption(NoPlot)/*GuiLogger*/ );

      agent->init(controller, robotersammlung.back (), wiring );

      global.agents.push_back(agent);
      global.configs.push_back(controller);
      break;

    case 't' : robotersammlung.back()->roboterAuftrennen ();

      break;

    case 'r' :
      atomsimRobot* neuerRob1;
      atomsimRobot* neuerRob2;

      robotersammlung[0]->roboterRekombination ( 0, 1.0/2, robotersammlung [1], &neuerRob1, &neuerRob2, Position ( 0, 20, 10 ), Position ( 0, -20, 10 ) );


      robotersammlung.push_back ( neuerRob1 );

      AbstractController* controller3;
      controller3 = new InvertMotorSpace ( 10 );
      One2OneWiring* wiring3;
      wiring3 = new One2OneWiring ( new ColorUniformNoise () );
      OdeAgent* agent3;
      agent3 = new OdeAgent( global, PlotOption(NoPlot)/*GuiLogger*/ );
      agent3->init(controller3, robotersammlung.back (), wiring3 );

      global.agents.push_back(agent3);
      global.configs.push_back(controller3);


      robotersammlung.push_back ( neuerRob2 );

      AbstractController* controller4;
      controller4 = new InvertMotorSpace ( 10 );
      One2OneWiring* wiring4;
      wiring4 = new One2OneWiring( new ColorUniformNoise () );
      OdeAgent* agent4;
      agent4 = new OdeAgent( global, PlotOption(NoPlot)/*GuiLogger*/);
      agent4->init(controller4, robotersammlung.back (), wiring4 );

      global.agents.push_back(agent4);
      global.configs.push_back(controller4);
      break;

    case 'n' : atomsammlung.push_back ( new atomsimAtom ( 0, &atomIDzaehler, odeHandle , -12, 0, 1, 0.3, 0.5, 1, 1, 20, 4, 20, 20.0, 40.0, 0.5, 0.2, 0.8 ) );
      break;

    }
}

//Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam
//Hier wird die Kollission untersucht
void atomCallback (const OdeHandle& odeHandle, void *data, dGeomID o1, dGeomID o2)
{
  int collision;
  unsigned int n, m;
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
              contact[i].surface.mu = 0.0; //normale Reibung von Reifen auf Asphalt = 0.8
              //contact[i].surface.slip1 = 0.0051;
              //contact[i].surface.slip2 = 0.0051;
              contact[i].surface.soft_erp = 1;
              contact[i].surface.soft_cfm = 0.0001; //Elastizität der Stoesse: klein keine                                                         Elastizität, groß viel Elsatizität
              dJointID c = dJointCreateContact (odeHandle.world ,odeHandle.jointGroup ,&contact[i]);
              dJointAttach ( c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));

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

void additionalLoopfunction ( GlobalData& global, bool draw, bool pause )
{

  //additional draw section
  if ( draw == 0 )
    for ( unsigned int n = 0; n < atomsammlung.size (); n++ )
      atomsammlung[n]->drawAtom ();

  //evolutionary section
  if ( pause == false )
    {
      //the simulationTime now is handled like an integer value
      if ( ( (int) ( global.time * ( 1 / global.odeConfig.simStepSize ) ) ) % lifecycle == 0 )
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
                  Position posA, posB, posC, posD;
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
                  robotersammlung[n]->rekursivVerschieben ( robotersammlung[n]->getUrsprungsatom (), posA );
                  robotersammlung[n+1]->rekursivVerschieben ( robotersammlung[n+1]->getUrsprungsatom (), posB );


                  //positions for the new robots
                  posC.x = ( 0 - 0.5 * playgroundx ) + ( (n+2) % anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
                  posC.y = ( 0 - 0.5 * playgroundx ) + ( (n+2) / anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
                  posC.z = playgroundheight/2;

                  posD.x = ( 0 - 0.5 * playgroundx ) + ( (n+3) % anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
                  posD.y = ( 0 - 0.5 * playgroundx ) + ( (n+3) / anzprozeile + 0.5 ) * (playgroundx / (maxpopulationsize / anzprozeile));
                  posD.z = playgroundheight/2;

                  robotersammlung[n]->roboterRekombination ( 0, 1.0/2, robotersammlung [n+1], &neuerRob1, &neuerRob2, posC, posD );

                  robotersammlung.push_back ( neuerRob1 );
                  robotersammlung.push_back ( neuerRob2 );
                }

              //deletes all global.agents, controllers and wirings, which are not linked to an robot from robotersammlung
              vector<OdeAgent*>::iterator agentit = global.agents.begin ();
              //for ( unsigned int m = 0; m < global.agents.size (); m++ )
              for ( vector<OdeAgent*>::iterator agentit = global.agents.begin (); agentit != global.agents.end (); agentit++ )
                {
                  bool del = true;
                  for ( vector<atomsimRobot*>::iterator robotit = robotersammlung.begin (); robotit != robotersammlung.end (); robotit++ )
                    {

                      if ( (*agentit)->getRobot () == (*robotit) )
                        {
                          dsPrint ( "OdeAgent nicht gelöscht.\n" );
                          del = false;
                          break;
                        }
                    }
                  if ( del )
                    {
                      dsPrint ( "OdeAgent wird gelöscht.\n" );


                      dsPrint ( "OdeAgents:%i Robots:%i\n", global.agents.size (), robotersammlung.size () );
                      for ( vector<Configurable*>::iterator configit = global.configs.begin(); configit != global.configs.end (); configit++ )
                        if ( (*configit) == (*agentit)->getController () )
                          {
                            global.configs.erase ( configit );
                            break;
                          }

                      delete (*agentit)->getController();
                      delete (*agentit)->getWiring ();
                      delete (*agentit);

                      global.agents.erase ( agentit );
                      //because the loop now is repeated fewer
                      agentit--;
                    }
                }
              //creates new global.agents, controllers and wirings if a robot is not linked to an agent

              for ( unsigned int n = 0; n < robotersammlung.size (); n++ )
                {
                  bool create = true;
                  for ( unsigned int m = 0; m < global.agents.size (); m++ )
                    {
                      if ( global.agents[m]->getRobot () == robotersammlung[n] )
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
                      OdeAgent* agent;
                      agent = new OdeAgent( global, PlotOption(NoPlot)/*GuiLogger*/ );

                      agent->init(controller, robotersammlung[n], wiring );
                      global.agents.push_back(agent);
                      global.configs.push_back(controller);
                    }
                }

              dsPrint ("Eine neue Generation entsteht!\n");
              for ( unsigned int n = 0; n < robotersammlung.size (); n ++ )
                {
                  robotersammlung[n]->setFitness ( 0 );

                }
            }

        }
      //adaptation of the fitness value of the robots
      else
        {
          //this is a simple evolution, where small robots are the fitesst
          /*for ( unsigned int n = 0; n < robotersammlung.size (); n ++ )
            {
            if ( robotersammlung[n]->getAtomAnzahl () == 1 )
            robotersammlung[n]->setFitness ( 0 ); //robots with only one atom could not replicate
            else
            robotersammlung[n]->setFitness ( 1.0/(robotersammlung[n]->getAtomAnzahl ()) );
            }*/


          for ( unsigned int n = 0; n < robotersammlung.size (); n ++ )
            {
              robotersammlung[n]->addFitness ( fabs ( robotersammlung[n]->getPosition ().x - evoarray[n].x ) );
              robotersammlung[n]->addFitness ( fabs ( robotersammlung[n]->getPosition ().y - evoarray[n].y ) );
              robotersammlung[n]->addFitness ( fabs ( robotersammlung[n]->getPosition ().z - evoarray[n].z ) );
              evoarray[n] = robotersammlung[n]->getPosition ();
            }

        }
      dsPrint ("Weltzeit: %lf\n", global.time );
    }
}


int main (int argc, char **argv)
{
  // initialise the simulation and provide the start, end, and config-function
  simulation_init(&start, &end, &config, &command, &atomCallback, &additionalLoopfunction );
  // start the simulation (returns, if the user closes the simulation)
  simulation_start(argc, argv);
  simulation_close();  // tidy up.
  return 0;
}
