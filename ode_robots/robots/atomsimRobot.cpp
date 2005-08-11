/************************************************************************/
/*atomsimRobot.cpp							*/
/*molecular robot, which controls a group auf connected atoms		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

using namespace std;

#include "atomsimRobot.h"


/**
 *Konstruktor
 *@author
 *@version
 **/
atomsimRobot::atomsimRobot ( int* start_roboterIDzaehler , dWorldID start_welt , dSpaceID start_raum , dJointGroupID start_contactgroup , vector<atomsimAtom*>* start_atomsammlung , atomsimAtom* start_ursprungsatom , int start_maxatomanzahl , double start_rekombinationstrennverhaeltniss )
:Roboter::Roboter ( (*start_roboterIDzaehler) , start_welt , start_raum , start_contactgroup , 0 )
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
	
	atomsammlung->push_back ( start_ursprungsatom );
	
	fitness = 0;
}

atomsimRobot::~atomsimRobot ()
{
	/*for ( int n = 0; n < getUrsprungsatom ()->getAnzahlAtome (); n++ )
		getUrsprungsatom ()->getAtomAt ( n )->atomabspaltung ( NULL , 1 );
	
	getUrsprungsatom ()->setRoboterID ( 0 );*/
	
	//deletion of all atoms, connected to the robot
	rekursiveAtomDeletation ( getUrsprungsatom () );
	dsPrint ( "ENDE Del!\n" );
}

void atomsimRobot::rekursiveAtomDeletation ( atomsimAtom* a )
{
	for ( int n = 0; n < a->getAnzahlAtome (); n++ )
	{
		rekursiveAtomDeletation ( a->getAtomAt ( n ) );
	}
	a->~atomsimAtom ();
	
	vector<atomsimAtom*>::iterator i = atomsammlung->begin();
	dsPrint ( "CHECKPOINT: size: %i\n" , atomsammlung->size () );
	for ( unsigned int n = 0; n < atomsammlung->size(); n++ )
	{
		
		if ( (*atomsammlung)[n]->getAtomID () == a->getAtomID () )
		{
			atomsammlung->erase ( i );
			break;
		}
		i++;
	}
	
	
}

/**
 *Calls the function drawRobot-function
 *@author Marcel Kretschmann
 *@version beta
 **/
void atomsimRobot::draw ()
{
	//drawRobot ( getUrsprungsatom () , true );
}

/**
 *Draws recursive all Atoms, belonging to the robot
 *@param i should be normaly true, false causes that the Origin Atom is not drawn
 *@author Marcel Kretschmann
 *@version beta
 **/
void atomsimRobot::drawRobot ( atomsimAtom* atom , bool i )
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
bool atomsimRobot::kollisionsermittlung ( dGeomID o1 , dGeomID o2 )
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
void atomsimRobot::place (Position pos, Color *c)
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
bool atomsimRobot::collisionCallback(void *data, dGeomID o1, dGeomID o2)
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
int atomsimRobot::getSensors ( sensor* sensors, int sensornumber )
{
	sensoraktualisierung ( getUrsprungsatom () , 0 , true );
	
	for ( int n = 0; n < sensornumber; n++ )
	{
		if ( n < getSensorfeldGroesse () )
			getWinkelDifferenz ( n , sensors++ );
		//if there are not enought Sensors active for the controler, the Sensor Value is set to zero
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
void atomsimRobot::setMotors ( const motor* motors, int motornumber )
{
	motoraktualisierung ( getUrsprungsatom () , motors , motornumber , true );
}

/**
 *Returns the number of sensors used by the robot.
 *@return number of sensors
 *@author Marcel Kretschmann
 *@version final
 **/
int atomsimRobot::getSensorNumber()
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
int atomsimRobot::getMotorNumber()
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
int atomsimRobot::sensoraktualisierung ( atomsimAtom* atom , int i , bool steuerung )
{
	//can happen after a collision and binding or fussion of a atom
	if ( steuerung == true )
	{
		if ( rekursionszaehler ( atom , 1 ) - 1 > getSensorfeldGroesse () )
			addSensor ();
		else
			if ( rekursionszaehler ( atom , 1 ) - 1 < getSensorfeldGroesse () )
				delSensor ();
	}

	//update of the sensor values
	for ( int n = 0; n < (*atom).getAnzahlAtome (); n++ )
	{
		//atom->setSpace ( raum ); //this is a call, that has nothing to do with the sensors, but this updates the membership of the atoms of a Robot to the robots dSpaces
		
		if ( i < getSensorfeldGroesse () )
		{
			sensorfeld[i].istwinkel_alt = sensorfeld[i].istwinkel;
	
			//sensorfeld[i].istwinkel = dJointGetAMotorAngle ( (*atom).getMotorAt ( n ) , 0 );
			sensorfeld[i].istwinkel = dJointGetHingeAngle ( (*atom).getJointAt ( n ) );
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
Position atomsimRobot::getPosition ()
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
int atomsimRobot::getSegmentsPosition ( vector<Position> &poslist )
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
void atomsimRobot::getStatus ()
{
	for ( int n = 0; n < getSensorfeldGroesse (); dsPrint ( "Sensor %i: %lf\n" , n , sensorfeld[n++].istwinkel ) );
}

/**
 *
 *@author
 *@version
 **/
atomsimAtom* atomsimRobot::getUrsprungsatom ()
{
	return ursprungsatom;
}

/**
 *
 *@author
 *@version
 **/
double atomsimRobot::getRekombinationsTrennverhaeltniss ()
{
	return rekombinationstrennverhaeltniss;
}

/**
 *
 *@author
 *@version
 **/
//fuegt einen leeren Sensor zum Sensor-Array hinzu
void atomsimRobot::addSensor ()
{
	Sensor tmpSensor;	
	sensorfeld.push_back ( tmpSensor );
	atomanzahl++; //die ANzahl der dem atomsimRobot zugeordneten Atome wird hier erhoeht, da add Sensor sowieso bei jeder Atombindung aufgerufen wird, und um nicht noch eine externe Steuerungsvariable wie fuer die vergroesserung des Sensorfeldes einzufuehren, wird das gleich mit hier erledigt
}

//entfernt einen bestehenden Sensor aus dem Sensor-Array
void atomsimRobot::delSensor ()
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
const motor* atomsimRobot::motoraktualisierung ( atomsimAtom* atom , const motor* motors , int motornumber , bool steuerung )
{
	//can happen after a collision and binding or fission of a atom, but only at the first function call
	if ( steuerung == true )
	{
		if ( rekursionszaehler ( atom , 1 ) - 1 > getSensorfeldGroesse () )
			addSensor ();
		else
			if ( rekursionszaehler ( atom , 1 ) - 1 < getSensorfeldGroesse () )
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
int atomsimRobot::getAtomAnzahl ()
{
	return atomanzahl;
}

/**
 *Calculates the difference off the actual angle and the last calculated angle of a sensor.
 *This overwrites the function from Roboter, because an tanh-operation is used to set the sensor value maxima to 1 and -1
 *@param motor number of the motor in the motor-list
 *@param X pointer to a variable, to save the calculated difference
 *@author Marcel Kretschmann
 *@version final
 **/
void atomsimRobot::getWinkelDifferenz ( int motor , double* X )
{
	//Eingangsdaten sind die Winkeldifferenzen eines Roboterberechnungsschrittes
	//Abfangen des Winkeldifferenzsprunges bei ueberschreiten er 2PI-Marke
	//tritt auf wenn die -PI- oder die PI-Marke ueberschritten wird	
	
	double w = sensorfeld[motor].istwinkel - sensorfeld[motor].istwinkel_alt;
	
	if ( ( w < -M_PI )
	|| ( w >  M_PI ) )
	{
		//1. Fall(PI-Marke wird ueberschritten, also annaeherung vom Positiven) -> es ergibt sich eine negative Winkeldifferenz
		if ( w > M_PI )
			*X = tanh ( -(2*M_PI - w) );
		//2. Fall(-PI-Marke wird ueberschritten, also Annaeherung vom Negativen) -> es ergibt sich eine positive Winkeldifferenz
		if ( w < -M_PI )
			*X = tanh ( (2*M_PI + w) );
		}
	else
		*X = tanh ( w );
	}

/**
 *
 *@author
 *@version
 **/
int atomsimRobot::rekursionszaehler ( atomsimAtom* a , int z )
{
	for ( int n = 0; n < (*a).getAnzahlAtome (); n++ )
		z = rekursionszaehler ( (*a).getAtomAt ( n ) , z + 1 );
	return z;

}
	
/**
 *Sets a new Position for an atom an all atom, which are connected as substructure to that atom.
 *no optimal performance, but so it is an easy way
 *@author
 *@version
 **/
void atomsimRobot::rekursivVerschieben ( atomsimAtom* a , Position pos )
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
double atomsimRobot::getFitness ()
{
	return fitness;
}

/**
 *This is the standart addfitness-function, not specialized to an evolution.
 *@author
 *@version
 **/
void atomsimRobot::setFitness ( double neue_fitness)
{
	fitness = neue_fitness;
}

/**
 *Fitenss Belohnung
 *@author
 *@version
 **/
void atomsimRobot::addFitness ( double fitnessplus )
{
	fitness += fitnessplus;
}

/**
 *Fitenss Strafe
 *@author
 *@version
 **/
void atomsimRobot::delFitness ( double fitnessminus )
{
	fitness -= fitnessminus;
}

//************************************Rekombination***********************************

/**
 *
 *@author
 *@version
 **/
//rekursives Kopieren eines atomsimRobots ab dem Atom  a
atomsimRobot* atomsimRobot::rekursivKopieren ( atomsimAtom* a , bool firstcall )
{
	atomsimRobot* neueratomsimRobot;
	neueratomsimRobot = NULL;
	if ( firstcall == true )
	{
		neueratomsimRobot = new atomsimRobot ( roboterIDzaehler , welt , raum , contactgroup , atomsammlung , new atomsimAtom ( (--(*roboterIDzaehler))++ , a->getAtomIDzaehler () , welt , raum , a->getX () ,a->getY () , a->getZ () , a->getRadius () , a->getHuellenradius () , a->getMasse (), a->getBindungsstaerke () , a->getAbspaltstaerke () ,  a->getMaxatombindungszahl () , a->getBindungsblockdauer () , a->getMaxmotorkraft () , a->getMotorgeschwindigkeitsfaktor () , a->getColorR (), a->getColorG () , a->getColorB () ) , maxatomanzahl , getRekombinationsTrennverhaeltniss () );
		
	}
	
	for ( int n = 0; n < a->getAnzahlAtome (); n++ )
	{
		//Kopiert ein Atom welches an der Stelle n an a haengt
		atomsammlung->push_back ( new atomsimAtom (  0 , (*a->getAtomAt (n)).getAtomIDzaehler () , welt , raum , (*a->getAtomAt (n)).getX () ,(*a->getAtomAt (n)).getY () , (*a->getAtomAt (n)).getZ () , (*a->getAtomAt (n)).getRadius () , (*a->getAtomAt (n)).getHuellenradius () , (*a->getAtomAt (n)).getMasse (), (*a->getAtomAt (n)).getBindungsstaerke () , (*a->getAtomAt (n)).getAbspaltstaerke (), (*a->getAtomAt (n)).getMaxatombindungszahl () , (*a->getAtomAt (n)).getBindungsblockdauer () , (*a->getAtomAt (n)).getMaxmotorkraft () , (*a->getAtomAt (n)).getMotorgeschwindigkeitsfaktor () , (*a->getAtomAt (n)).getColorR (), (*a->getAtomAt (n)).getColorG () , (*a->getAtomAt (n)).getColorB () ) );

		(*atomsammlung)[ atomsammlung->size () - 2]->atombindung ( atomsammlung->back () , a->getAtomAt (n)->getKollisionsvektor1 () , a->getAtomAt (n)->getKollisionsvektor2 () );
		rekursivKopieren ( a->getAtomAt ( n ) , false );

	}
	
	//if it is the first recursive step, then the number of atoms of the new robot is calculated
	if ( firstcall == true )
	{
		neueratomsimRobot->atomanzahl = rekursionszaehler ( neueratomsimRobot->getUrsprungsatom () , 1 );
	}
	return neueratomsimRobot;
}
	
/**
 *
 *@author
 *@version
 **/
bool atomsimRobot::roboterAuftrennen ( atomsimAtom* a/*=0*/ , atomsimAtom** newrobotpart /*=0*/ , atomsimAtom** endofpieceone /*=0*/ , double trennverhaeltniss /*= 0*/ )
{
	if ( a == 0 )
		a = getUrsprungsatom ();
	if ( trennverhaeltniss == 0 )
		trennverhaeltniss = getRekombinationsTrennverhaeltniss ();
	for ( int faktor = 1; faktor < getAtomAnzahl (); faktor++ )
		for ( int n = 0; n < (*a).getAnzahlAtome (); n++ )
		{
			//does not count the atom a here
			if ( fabs ( ( ((double) rekursionszaehler ( a->getAtomAt (n) , 1 ) ) / getAtomAnzahl ()) - trennverhaeltniss ) < ( (1.0*faktor)/getAtomAnzahl ()) )
			{
				//Abspalten des Ursprungs-Joints
				
				if ( endofpieceone != 0 )
					*endofpieceone = a;
				if ( endofpieceone != 0 )
					*newrobotpart = a->getAtomAt ( n );
				
				dJointDestroy ( a->getAtomAt (n)->getUrsprungJoint () );
	
				//loeschen der einzellinks des Atoms und des Joints in den jewailigen Listen des Ursprungsatoms
				a->delAtomAt ( n );
				a->delJointAt ( n );
	
				a->getAtomAt (n)->setUrsprung ( NULL );
				return true;
			}
			else
				return roboterAuftrennen ( (*a).getAtomAt ( n ) , newrobotpart , endofpieceone , trennverhaeltniss );
		}
	//if there it is not possible to make a clean cut between the robots (mostly if the robot only consists of one atom)
	
	atomsammlung->push_back ( new atomsimAtom ( 0 , a->getAtomIDzaehler() , world , space  , a->getX () + 2*a->getRadius () , a->getY () , a->getZ () , 0.3 , 0.5 , 1 , 1 , 15 , 4 , 20 , 20.0 , 40.0 , 1 , 1 , 1 ) );
	
	if ( endofpieceone != 0 )
	{
		*endofpieceone = getUrsprungsatom ();
		*newrobotpart = atomsammlung->back ();
		return true;
	}
	
		
	return false;
}
	
/**
 *
 *@author
 *@version
 **/
void atomsimRobot::roboterRekombination ( int vermehrungsart , double trennverhaeltniss , atomsimRobot* partner , atomsimRobot** speicherort1 , atomsimRobot** speicherort2 , Position newpos1 , Position newpos2 )
{
	atomsimRobot* neueratomsimRobot1;
	atomsimRobot* neueratomsimRobot2;
	//Erzeugung von Kopien der bereits bestehenden atomsimRobotn
	neueratomsimRobot1 = rekursivKopieren ( getUrsprungsatom () , true );
	neueratomsimRobot2 = rekursivKopieren ( (*partner).getUrsprungsatom () , true );

	//Auftrennen der Kopien und erzeugen von 4 atomsimRobot-Schnipseln, hierbei muess sich die Atome gemerkt werden, die spaeter als neue Bindungsorte dienen sollen
	atomsimAtom* schnipsel1;
	atomsimAtom* schnipsel2;
	atomsimAtom* schnipsel3;
	atomsimAtom* schnipsel4;
	atomsimAtom* bindungsatom1;
	atomsimAtom* bindungsatom2;
	
		if ( neueratomsimRobot1->roboterAuftrennen ( 0 , &schnipsel2 , &bindungsatom1 , trennverhaeltniss ) == true )
		{
			dsPrint ( "TRENNUNG VON ROBOTER 1 ERFOLGREICH!\n"  );
			schnipsel1 = neueratomsimRobot1->getUrsprungsatom ();
				
				if ( neueratomsimRobot2->roboterAuftrennen ( 0 , &schnipsel4 , &bindungsatom2 , trennverhaeltniss ) == true )
				{
					dsPrint ( "TRENNUNG VON ROBOTER 2 ERFOLGREICH!\n" );
					schnipsel3 = neueratomsimRobot2->getUrsprungsatom ();
					
					//verschieben der Schnipsel, so dass ein Binden einen korrekten atomsimRobot erzeugt, also die Bindungsatome nicht zu weit auseinander liegen
					rekursivVerschieben ( schnipsel1 , newpos1 );
					Position newpos1_2;
					newpos1_2.x = bindungsatom1->getX () + 2*bindungsatom1->getHuellenradius ();
					newpos1_2.y = bindungsatom1->getY ();// + 2*bindungsatom1->getHuellenradius ();
					newpos1_2.z = bindungsatom1->getZ ();// + 2*bindungsatom1->getHuellenradius ();
					rekursivVerschieben ( schnipsel4 , newpos1_2 );
					
					rekursivVerschieben ( schnipsel3 , newpos2 );
					Position newpos2_1;
					newpos2_1.x = bindungsatom2->getX () + 2*bindungsatom2->getHuellenradius ();
					newpos2_1.y = bindungsatom2->getY ();// + 2*bindungsatom2->getHuellenradius ();
					newpos2_1.z = bindungsatom2->getZ ();// + 2*bindungsatom2->getHuellenradius ();
					rekursivVerschieben ( schnipsel2 , newpos2_1 );
					
					
					//Bindung der atomsimRoboteinzelteile; dies erfolgt im Bindungsraum, einem speziellen Gebiet der Simulationswelt, den gewoehnliche Atome nicht betreten koennen
					raumvektor kraftraumvektor1 , kraftraumvektor2;
					//bisher feste Axen, alternative gesucht
					kraftraumvektor1.x = 2;
					kraftraumvektor1.y = 0;
					kraftraumvektor1.z = 0;
					kraftraumvektor2.x = 0;
					kraftraumvektor2.y = 0;
					kraftraumvektor2.z = 2;
					
					
					bindungsatom1->atombindung ( schnipsel4 , kraftraumvektor1 , kraftraumvektor2 );
					
					bindungsatom2->atombindung ( schnipsel2 , kraftraumvektor1 , kraftraumvektor2 );
					
				
					*speicherort1 = neueratomsimRobot1;
					*speicherort2 = neueratomsimRobot2;
					return;
				}
				else
				{
					dsPrint ( "Keine Rekombination: keine Trennung von Roboter 2 möglich!\n" );
					dsPrint ( "AtomAnzahl=%i\n" , neueratomsimRobot2->getAtomAnzahl () );
				}
		}
		else
		{
			dsPrint ( "Keine Rekombination: keine Trennung von Roboter 1 möglich!\n" );
			dsPrint ( "AtomAnzahl=%i\n" , neueratomsimRobot1->getAtomAnzahl () );
		}	
}

