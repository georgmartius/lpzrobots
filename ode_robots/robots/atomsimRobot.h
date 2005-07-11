/************************************************************************/
/*atomsimRobot.h							*/
/*molecular robot, which controls a group auf connected atoms		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

using namespace std;

#include "atomsimAtom.h"


/**
 *
 *@author Marcel Kretschmann
 *@version alpha 0.5
 **/
class atomsimRobot : public Roboter
{
private:
	//Eigenschaften
	int roboterID;
	int* roboterIDzaehler;
	
	dWorldID* welt;
	dSpaceID* raum;
	
	int atomanzahl;
	int maxatomanzahl;
	double rekombinationstrennverhaeltniss;
	
	atomsimAtom* ursprungsatom;
	
	vector<atomsimAtom*>* atomsammlung;

public:

	/**
 	 *Konstruktor
 	 *@author
 	 *@version
 	 **/
	atomsimRobot ( int* start_roboterIDzaehler , dWorldID* start_welt , dSpaceID* start_raum , dJointGroupID* start_contactgroup , vector<atomsimAtom*>* start_atomsammlung , atomsimAtom* start_ursprungsatom , int start_maxatomanzahl , double start_rekombinationstrennverhaeltniss )
	:Roboter::Roboter ( (*start_roboterIDzaehler) , welt , raum , start_contactgroup , 0 )
	{
		roboterIDzaehler = start_roboterIDzaehler;
		roboterID = (*roboterIDzaehler)++;
		
		welt = start_welt;
		raum = start_raum;
		
		atomsammlung = start_atomsammlung;
		
		atomanzahl = 1;
		maxatomanzahl = start_maxatomanzahl;
		ursprungsatom = start_ursprungsatom;
		rekombinationstrennverhaeltniss = start_rekombinationstrennverhaeltniss;
	}
	
	~atomsimRobot ()
	{
	}
	
	/**
	 *Calls the function drawRobot-function
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	void draw ()
	{
		drawRobot ( getUrsprungsatom () , true );
	}
	
	/**
	 *Draws recursive all Atoms, belonging to the robot
	 *@param i should be normaly true, false causes that the Origin Atom is not drawn
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	void drawRobot ( atomsimAtom* atom , bool i )
	{
		//can happen after a collision and binding or fission of a atom
		if ( i == true ) atom->drawAtom ();
		for ( int n = 0; n < (*atom).getAnzahlAtome () ; n++ )
		{
			if ( atom->getAtomAt ( n )->getAnzahlAtome () > 0 )
				drawRobot ( atom->getAtomAt ( n ) , false );
		}
	}
	
	/**
	 *Decides if some collisions of the robot should not be threated by by the collision management.
	 *This overwrides the function in the roboter class.
	 *@param o1 Geometrieobjekt 1, dass an der Kollision beteiligt ist
	 *@param o2 Geometrieobjekt 2, dass an der Kollision beteiligt ist
	 *@return true, if the collision should not be threated, false else
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	bool kollisionsermittlung ( dGeomID o1 , dGeomID o2 )
	{
		return false;
	}
	
	/**Sets the simatomRobot to position pos, sets color to c, and creates robot if necessary.
	 *This overwrides the function place of the class robot.
	 *@param pos desired position of the atomsimRobot in struct Position
	 *@param c desired color for the atomsimRobot in struct Color
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	void place (Position pos, Color *c)
	{
		rekursivVerschieben ( getUrsprungsatom () , pos );
		
		color.r = (*c).r;
		color.g = (*c).g;
		color.b = (*c).b;
	}
	
	/**
	*This is the collision handling function for snake robots.
	*This overwrides the function collisionCallback of the class robot.
	*@param data
	*@param o1 first geometrical object, which has taken part in the collision
	*@param o2 second geometrical object, which has taken part in the collision
	*@return true if the collision was threated  by the robot, false if not
	*@author Marcel Kretschmann
	*@version beta
	**/
	bool collisionCallback(void *data, dGeomID o1, dGeomID o2)
	{
		//checks if one of the collision objects is part of the robot
		/*bool tmp_kollisionsbeteiligung = false;
		for ( int n = 0; n < getAnzahlAtome (); n++ )
		{
			if ( getObjektAt ( n ).geom == o1 || getObjektAt ( n ).geom == o2 )
			{
				tmp_kollisionsbeteiligung = true;
				break;
			}
		}
	
		if ( tmp_kollisionsbeteiligung == true )		
		{
			int i,n;
			const int N = 10;
			dContact contact[N];
			bool kollission = false;
	
			//tests, if a special collision should not be threated
			if ( Schlange::kollisionsermittlung ( o1 , o2 ) == true )
				kollission = true;
	
			if ( kollission == false )
			{
				n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
				if (n > 0)
					for (i=0; i<n; i++)
					{
						contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
						dContactSoftERP | dContactSoftCFM | dContactApprox1;
						contact[i].surface.mu = 0.8;
						contact[i].surface.slip1 = 0.005;
						contact[i].surface.slip2 = 0.005;
						contact[i].surface.soft_erp = 1;
						contact[i].surface.soft_cfm = 0.00001;
	
						dJointID c = dJointCreateContact ( (*world) , (*contactgroup) , &contact[i] );
						dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
					}
			}
			
			
			
			
			return true; //if collision was threated by this robot
		}
		else return false; //if collision was not threated by this robot*/
		return false;
	}
	
	/**
	 *Writes the sensor values to an array in the memory.
	 *@param sensor* pointer to the array
	 *@param sensornumber length of the sensor array
	 *@return number of actually written sensors
	 *@author Marcel Kretschmann
	 *@version beta
	 **/
	int getSensors ( sensor* sensors, int sensornumber )
	{
		sensoraktualisierung ( getUrsprungsatom () , 0 , true );
		
		for ( int n = 0; n < sensornumber; n++ )
		{
			if ( n < getSensorfeldGroesse () )
				getWinkelDifferenz ( n , sensors++ );
			else
				*sensors++ = 0;
		}
		return sensornumber; //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden
	}
	
	/**
	*Reads the actual motor commands from an array, an sets all motors of the snake to this values.
	*It is an linear allocation.
	*@param motors pointer to the array, motor values are scaled to [-1,1] 
	*@param motornumber length of the motor array
	*@author Marcel Kretschmann
	*@version beta
	**/
	void setMotors ( const motor* motors, int motornumber )
	{
		motoraktualisierung ( getUrsprungsatom () , motors , motornumber , true );
	}
	
	/**
	 *Returns the number of sensors used by the robot.
	 *@return number of sensors
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	int getSensorNumber()
	{
		return maxatomanzahl - 1;
		//return getSensorfeldGroesse ();
	}
	
	/**
	*Returns the number of motors used by the snake.
	*@return number of motors
	*@author Marcel Kretschmann
	*@version final
	**/
	int getMotorNumber()
	{
		return maxatomanzahl - 1;
		//return rekursionszaehler ( getUrsprungsatom () , 1 ) - 1;
	}
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	//rekursive Funktion, welches das lineare Sensorfeld mittels Tiefensuche aktualisiert
	int sensoraktualisierung ( atomsimAtom* atom , int i , bool steuerung )
	{
		//can happen after a collision and binding or fission of a atom
		if ( steuerung == true )
		{
			if ( rekursionszaehler ( atom , 1 ) - 1 < getSensorfeldGroesse () )
				addSensor ();
			if ( rekursionszaehler ( atom , 1 ) - 1 > getSensorfeldGroesse () )
				delSensor ();
		}
		
		for ( int n = 0; n < (*atom).getAnzahlAtome (); n++ )
		{
			if ( i < getSensorfeldGroesse () )
			{
				sensorfeld[i].istwinkel_alt = sensorfeld[i].istwinkel;
				
				sensorfeld[i].istwinkel = dJointGetAMotorAngle ( (*atom).getMotorAt ( n ) , 0 );
				i++;
				
					if ( (*(*atom).getAtomAt ( n )).getAnzahlAtome () > 0 )
					i = sensoraktualisierung ( (*atom).getAtomAt ( n ) , i , false );
			}
			else
				return i;
		}
		return i;
		
	}
	
	/**
	*Returns the position of the snake. Here the position of the snake is the position of the first element of the snake.
	*@return Position (x,y,z)
	*@author Marcel Kretschmann
	*@version final
	**/
	Position getPosition ()
	{
		Position returnPos;
		returnPos.x = getUrsprungsatom ()->getX ();
		returnPos.y = getUrsprungsatom ()->getY ();
		returnPos.z = getUrsprungsatom ()->getZ ();
	
		return returnPos;
	}
	
	/**
	 *Returns a list with the positionvectors of all segments of the robot
	 *no function
	 *@param poslist list with positionvectors (of all robot segments) (free after use!)
	 *@return length of the list
	 *@author Marcel Kretschmann
	 *@version final
	 **/
	int getSegmentsPosition ( vector<Position> &poslist )
	{
		/*const dReal* tmp;
		for ( int n = 0; n < getObjektAnzahl (); n++ )
		{
			tmp = dBodyGetPosition ( getObjektAt ( n ).body );
			poslist[n].x = tmp[0];
			poslist[n].y = tmp[1];
			poslist[n].z = tmp[2];
		}*/
		return getObjektAnzahl ();
	}
	
	/**
	*Prints some internal robot parameters. Actualy it prints all sensor data of one callculation step.
	*@author Marcel Kretschmann
	*@version beta
	**/
	void getStatus ()
	{
		for ( int n = 0; n < getSensorfeldGroesse (); dsPrint ( "Sensor %i: %lf\n" , n , sensorfeld[n++].istwinkel ) );
	}
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	atomsimAtom* getUrsprungsatom ()
	{
		return ursprungsatom;
	}
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	double getRekombinationsTrennverhaeltniss ()
	{
		return rekombinationstrennverhaeltniss;
	}
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	//fuegt einen leeren Sensor zum Sensor-Array hinzu
	void addSensor ()
	{
		Sensor tmpSensor;	
		sensorfeld.push_back ( tmpSensor );
		atomanzahl++; //die ANzahl der dem atomsimRobot zugeordneten Atome wird hier erhoeht, da add Sensor sowieso bei jeder Atombindung aufgerufen wird, und um nicht noch eine externe Steuerungsvariable wie fuer die vergroesserung des Sensorfeldes einzufuehren, wird das gleich mit hier erledigt
	}
	
	//entfernt einen bestehenden Sensor aus dem Sensor-Array
	void delSensor ()
	{
		sensorfeld.pop_back ( );
		atomanzahl--; //die ANzahl der dem atomsimRobot zugeordneten Atome wird hier verringert, da delSensor sowieso bei jeder Atomabspaltung aufgerufen wird, und um nicht noch eine externe Steuerungsvariable wie fuer die verkleinerung des Sensorfeldes einzufuehren, wird das gleich mit hier erledigt

	}
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	//rekursive Funktion, welches das lineare Sensorfeld mittels Tiefensuche aktualisiert
	const motor* motoraktualisierung ( atomsimAtom* atom , const motor* motors , int motornumber , bool steuerung )
	{
		//can happen after a collision and binding or fission of a atom, but only at the first function call
		if ( steuerung == true )
		{
			if ( rekursionszaehler ( atom , 1 ) - 1 < getSensorfeldGroesse () )
				addSensor ();
			if ( rekursionszaehler ( atom , 1 ) - 1 > getSensorfeldGroesse () )
				delSensor ();
		}
	
		for ( int n = 0; n < (*atom).getAnzahlAtome () ; n++ )
		{
			if ( motornumber > 0 ) //only if the end of the value array is not reached
			//if there are motors left, which are not set, because the motors-array was not big enought, these motors keept their old value an are not updated
			{
				motornumber--;
				(*atom).setMotorWinkel ( n , (*motors++) );
				if ( (*(*atom).getAtomAt ( n )).getAnzahlAtome () > 0 )
				motors = motoraktualisierung ( (*atom).getAtomAt ( n ) , motors , motornumber , false );
			}
		}
		return motors;
	}
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	int getAtomAnzahl ()
	{
		return atomanzahl;
	}
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	int rekursionszaehler ( atomsimAtom* a , int z )
	{
		for ( int n = 0; n < (*a).getAnzahlAtome (); n++ )
			z = rekursionszaehler ( (*a).getAtomAt ( n ) , z + 1 );
		return z;
	
	}
	
	/**
 	 *no optimal performance, but so it is an easy way
 	 *@author
 	 *@version
 	 **/
	void rekursivVerschieben ( atomsimAtom* a , Position pos )
	{
		double dx , dy , dz;
		Position pos2;
		
		dx = pos.x - a->getX ();
		dy = pos.y - a->getY ();
		dz = pos.z - a->getZ ();

		for ( int n = 0; n < a->getAnzahlAtome (); n++ )
		{
			pos2.x = a->getAtomAt ( n )->getX () + dx;
			pos2.y = a->getAtomAt ( n )->getY () + dy;
			pos2.z = a->getAtomAt ( n )->getZ () + dz;
			
			rekursivVerschieben ( a->getAtomAt ( n ) , pos2 );
		}
		
		a->setXYZ ( pos.x , pos.y , pos.z );
	}
		
//********************************Evolutionsabschnitt*********************************

//*************************************Selektion**************************************
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	double getFitness ()
	{
	
		return 0.0;
	}

//************************************Rekombination***********************************

	
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	//rekursives Kopieren eines atomsimRobots ab dem Atom  a
	atomsimRobot* rekursivKopieren ( atomsimAtom* a , int status )
	{
		atomsimRobot* neueratomsimRobot;
		if ( status == 0 )
		{
			neueratomsimRobot = new atomsimRobot ( roboterIDzaehler , welt , raum , contactgroup , atomsammlung , new atomsimAtom ( (--(*roboterIDzaehler))++ , (*a).getAtomIDzaehler () , welt , raum , (*a).getX () ,(*a).getY () , (*a).getZ () , (*a).getRadius () , (*a).getHuellenradius () , (*a).getMasse (), (*a).getBindungsstaerke () , (*a).getAbspaltstaerke () ,  (*a).getMaxatombindungszahl () , (*a).getBindungsblockdauer () , (*a).getMaxmotorkraft () , (*a).getMotorgeschwindigkeitsfaktor () , (*a).getColorR (), (*a).getColorG () , (*a).getColorB () ) ,
			maxatomanzahl , getRekombinationsTrennverhaeltniss () );
		}
		
		for ( int n = 0; n < (*a).getAnzahlAtome (); n++ )
		{
			//Kopiert ein Atom welches an der Stelle n an a haengt
			atomsammlung->push_back ( new atomsimAtom (  0 , (*(*a).getAtomAt (n)).getAtomIDzaehler () , welt , raum , (*(*a).getAtomAt (n)).getX () ,(*(*a).getAtomAt (n)).getY () , (*(*a).getAtomAt (n)).getZ () , (*(*a).getAtomAt (n)).getRadius () , (*(*a).getAtomAt (n)).getHuellenradius () , (*(*a).getAtomAt (n)).getMasse (), (*(*a).getAtomAt (n)).getBindungsstaerke () , (*(*a).getAtomAt (n)).getAbspaltstaerke (), (*(*a).getAtomAt (n)).getMaxatombindungszahl () , (*(*a).getAtomAt (n)).getBindungsblockdauer () , (*(*a).getAtomAt (n)).getMaxmotorkraft () , (*(*a).getAtomAt (n)).getMotorgeschwindigkeitsfaktor () , (*(*a).getAtomAt (n)).getColorR (), (*(*a).getAtomAt (n)).getColorG () , (*(*a).getAtomAt (n)).getColorB () ) );
			
			(*a).atombindung ( (*a).getAtomAt ( n ) , (*a).getKollisionsvektor1 () , (*a).getKollisionsvektor2 () );
			rekursivKopieren ( (*a).getAtomAt ( n ) , 1 );
			
		}
		return neueratomsimRobot;
	}
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	bool roboterAuftrennen ( atomsimAtom* a )
	{
		int z;
		
		for ( int n = 0; n < (*a).getAnzahlAtome (); n++ )
		{
			if ( roboterAuftrennen ( (*a).getAtomAt ( n ) ) == false )
			{
				z = getAtomAnzahl () - rekursionszaehler ( a , 1 );
				if ( (z / getAtomAnzahl ()) == getRekombinationsTrennverhaeltniss () )
				{
					
					
					//Abspalten des Ursprungs-Joints
					
					dJointDestroy ( (*(*a).getAtomAt ( n )).getUrspungJoint () );
					//loeschen der einzellinks des Atoms und des Joints in den jewailigen Listen des Ursprungsatoms
					(*a).delAtomAt ( n );
					(*a).delJointAt ( n );
					(*a).delMotorAt ( n );
					
				}
			}
			else return true;
		}
		return false;
	}
	
	/**
 	 *
 	 *@author
 	 *@version
 	 **/
	void roboterRekombination ( int vermehrungsart , atomsimRobot* partner , atomsimRobot** speicherort1 , atomsimRobot** speicherort2 )
	{
		atomsimRobot* neueratomsimRobot1;
		atomsimRobot* neueratomsimRobot2;
		//Erzeugung von Kopien der bereits bestehenden atomsimRobotn
		neueratomsimRobot1 = rekursivKopieren ( getUrsprungsatom () , 0 );
		neueratomsimRobot2 = rekursivKopieren ( (*partner).getUrsprungsatom () , 0 );
		//Auftrennen der Kopien und erzeugen von 4 atomsimRobot-Schnipseln, hierbei muess sich die Atome gemerkt werden, die spaeter als neue Bindungsorte dienen sollen
		roboterAuftrennen ( getUrsprungsatom () );
		(*partner).roboterAuftrennen ( getUrsprungsatom () );
		
		//verschieben der Schnipsel, so dass ein Binden einen korrekten atomsimRobot erzeugt, also die Bindungsatome nicht zu weit auseinander liegen
		//Bindung der atomsimRoboteinzelteile; dies erfolgt im Bindungsraum, einem speziellen Gebiet der Simulationswelt, den gewoehnliche Atome nicht betreten koennen
		
		
		*speicherort1 = neueratomsimRobot1;
		*speicherort2 = neueratomsimRobot2;
		
	}
	
};
