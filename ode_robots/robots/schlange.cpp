/************************************************************************/
/*schlange.cpp								*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

#include "schlange.h"



/**
 *Konstruktor
 *@param int RoboterID
 *@param dWorldID Referenz auf die ODE-Simulationswelt in der der Roboter angelegt werden soll
 *@param dSpaceID Referenz auf die den ODE-Simualtions-Kolissionsraum, in dem der Roboter arbeiten soll.
 *@param double anfaengliche X-Koordinatenposition
 *@param double anfaengliche Y-Koordinatenposition
 *@param double anfaengliche Z-Koordinatenposition
 *@param int Anzahl der Schlangenglieder
 *@param double Schlangengliedlaenge
 *@param double Schlangenglieddurchmesser
 *@param double Schlanngengliedabstand zwischen zwei benachbarten Schlangengliedern
 *@param double Masse eines Schlangengliedes
 *@param double Maximale Kraft der Motor-Joints in den Schlangengelenken
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
Schlange::Schlange ( int startRoboterID , dWorldID* welt , dSpaceID* raum , dJointGroupID* start_contactgroup , int start_Sensoranzahl , double start_x , double start_y , double start_z , int armanzahl , double glieder_laenge , double glieder_durchmesser , double glieder_abstand , double glieder_masse , double start_maxmotorkraft , double start_geschwindigkeitsfaktor ) :
Roboter::Roboter ( startRoboterID , welt , raum , start_contactgroup , start_Sensoranzahl )
{
	Object tmp_body;
	
	dMass masse;
	dMatrix3 R;//Matrix fuer Koerper-Rotationen


	gliederdurchmesser = glieder_durchmesser;
	gliederlaenge = glieder_laenge;
	schlangenarmanzahl = armanzahl;
	geschwindigkeitsfaktor = start_geschwindigkeitsfaktor;
	maxmotorkraft = start_maxmotorkraft;
	

	//*************Koerperdefinitionsabschnitt**************
	
	//Aufbau der Massenverteilungsmatrix fuer die einzelnen Schlangenglieder
	dMassSetCappedCylinderTotal ( &masse , glieder_masse , 2 , glieder_laenge , glieder_durchmesser );
	//zuordnung der Rotationsmatizenwerte zur Rotationsmatrix R
	dRFromAxisAndAngle ( R , 0 , 1 , 0 , M_PI/2 );//hier drehung um 90° um die y-Achse

	for ( int n = 0; n < armanzahl; n++ )
	{
		tmp_body.body = dBodyCreate ( *world );
		objektliste.push_back ( tmp_body );
		
	
		dBodySetPosition ( (objektliste.back ()).body , start_x + (n + 0.5 )*glieder_laenge + n * glieder_abstand, start_y ,  start_z );

		dBodySetMass ( (objektliste.back ()).body , &masse );
	
		(objektliste.back ()).geom = dCreateCCylinder ( *space , glieder_durchmesser , glieder_laenge );
		dGeomSetBody ( (objektliste.back ()).geom , (objektliste.back ()).body );

		//dBodySetRotation ( (objektliste.back ()).body , R );
		dGeomSetRotation ( (objektliste.back ()).geom , R );
	}

	//*****************Join-Generierungsabschnitt***********
	for ( int n = 0; n < armanzahl-1; n++ )
	{
		jointliste.push_back ( dJointCreateUniversal ( *world , 0 ) );
		
		dJointAttach ( jointliste.back () , objektliste[n].body , objektliste[n+1].body );
			
		dJointSetUniversalAnchor ( jointliste.back () , dBodyGetPositionAll ( objektliste[n].body , 1 ) + ( dBodyGetPositionAll ( objektliste[n+1].body , 1 ) - dBodyGetPositionAll ( objektliste[n].body , 1 ) )/2 , dBodyGetPositionAll ( objektliste[n].body , 2 ) + ( dBodyGetPositionAll ( objektliste[n+1].body , 2 ) - dBodyGetPositionAll ( objektliste[n].body , 2 ) )/2 , dBodyGetPositionAll ( objektliste[n].body , 3 ) );

		dJointSetUniversalAxis1 ( jointliste.back () , 0 , 1 , 0 );
		dJointSetUniversalAxis2 ( jointliste.back () , 0 , 0 , 1 );
	}	

	//Anfangsbelegung der sensorfelder
 	for ( int n = 0; n < 2*(armanzahl-1); n++ )
	{
		sensorfeld[n].sollwinkel = 0;
		sensorfeld[n].istwinkel_alt = sensorfeld[n].istwinkel;
	}
}
	
/**
*Destruktor
*@author Marcel Kretschmann
*@version alpha 1.0
**/
Schlange::~Schlange()
{
}
	
/**
*Zeichnet die Koerper-GeometrieObjekte.
*@author Marcel Kretschmann
*@version alpha 1.0
**/
void Schlange::draw()
{
	double box [3];
	dsSetTexture (DS_WOOD);
	dsSetColor ( color.r , color.g , color.b );

	box[0] = gliederlaenge/10; box[1] = gliederdurchmesser/10; box[2] = gliederdurchmesser/10;
	for ( int n = 0; n < schlangenarmanzahl; n++ )
	{
			dsDrawCappedCylinder ( dGeomGetPosition ( getObjektAt ( n ).geom ) , dGeomGetRotation ( getObjektAt ( n ).geom ) , gliederlaenge , gliederdurchmesser );
	}
}
	
/**
 *Diese Funktion ermittelt ob es zwischen bestimmten Elementen des Roboters eine kollision gibt,
 *und verhindert, dass diese Kollision in die globale Kollisionsbehandlung mit einfließt.
 *In diesem Konkreten Fall wird verhindert, dass die außenhuelle von zwei benachbarten Schlangengliedern
 *miteinader kollidieren, und dass auch die inneren Schlangenhuelle nicht mit den benachbarten Schlangenhuellen kollidieren.
 *@param dGeomID Geometrieobjekt 1, dass an der Kollision beteiligt ist
 *@param dGeomID Geometrieobjekt 2, dass an der Kollision beteiligt ist
 *@return bool true, wenn keine Kollision zwischen den beiden Geometrieobjekten erfolgt, false sonst
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
bool Schlange::kollisionsermittlung ( dGeomID o1 , dGeomID o2 )
{
	for ( unsigned int n = 0; n < objektliste.size (); n++ )
	{
		if 
		(
		( getObjektAt ( n ).geom == o1 && getObjektAt ( n + 1 ).geom == o2 ) || ( getObjektAt ( n ).geom == o2 && getObjektAt ( n + 1 ).geom == o1 )		
		)
			//if ( ( schlangenhuellenliste[n] == o1 ) || ( schlangenhuellenliste[n] == o2 ) )
			return true;
	}
	
	return false;
}
	
/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
@param pos desired position of the robot in struct Position
@param c desired color for the robot in struct Color
*/
void Schlange::place (Position pos, Color *c)
{
	double dx , dy , dz;
	
	dx = pos.x - getPosition ().x;
	dy = pos.y - getPosition ().y;
	dz = pos.z - getPosition ().z;
	
	for ( int n = 0; n < getObjektAnzahl (); n++ )
		dBodySetPosition ( getObjektAt(n).body , getPosition ( n ).x + dx , getPosition ( n ).y + pos.y ,getPosition ( n ).z +  pos.z );
	
	color.r = (*c).r;
	color.g = (*c).g;
	color.b = (*c).b;
}

/**
*Hier wird die zu setzende Winkeldifferenz zum aktuellen Winkel des Motors hinzugegeben.
*Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam.
*Hier wird die Kollission untersucht.
*@param void*
*@param dGeomID erstes GeometrieObject das an der Kollission beteiligt ist
*@param dGeomID zweites GeometrieObject das an der Kollission beteiligt ist
*@author Marcel Kretschmann
*@version development
**/
 bool Schlange::collisionCallback(void *data, dGeomID o1, dGeomID o2)
{
	//Ueberpruefung ob  die Kollision mit dem Roboter zusammenhing
	bool tmp_kollisionsbeteiligung = false;
	for ( int n = 0; n < getObjektAnzahl (); n++ )
	{
		if ( getObjektAt ( n ).geom == o1 || getObjektAt ( n ).geom == o2 )
		{
			tmp_kollisionsbeteiligung = true;
			break;
		}
	}
	//wenn eine Beteiligung des Roboters der Fall ist, erfolgt die Kollisionsbehandlung 
	if ( tmp_kollisionsbeteiligung == true )		
	{
		int i,n;
		const int N = 10;
		dContact contact[N];
		bool kollission = false;

		//Test ob einige der Roboterkollisionen eventuell nicht behandelt werden sollen
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
					contact[i].surface.mu = 0.8; //normale Reibung von Reifen auf Asphalt
					contact[i].surface.slip1 = 0.005;
					contact[i].surface.slip2 = 0.005;
					contact[i].surface.soft_erp = 1;
					contact[i].surface.soft_cfm = 0.00001;
/*>>>>>>>>>>>>>>>>>>>>>(*world) zu world*/
					dJointID c = dJointCreateContact ( (*world) , (*contactgroup) , &contact[i] );
					dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
				}
		}
		return true; //wenn die Kollision durch diesen Roboter beahndelt wurde
	}
	else return false; //wenn die Kollision nicht durch diesen Roboter beahndelt wurde
}

/**Gibt die Sensorwerte aus dem Roboterinternensensorfeld an einen uebergebenen Speicherort aus.
 *@param sensor* sensors Zeiger auf den Zielort der Sensordaten
 *@param int sensornumber Laenge des Sensorenarrays in das gespeichert werden soll
 *@return int Anzahl der Sensoren in die schon ein aktueller Wert geschrieben wurde
 **/
 int Schlange::getSensors ( sensor* sensors, int sensornumber )
{
	sensoraktualisierung ();
	for ( int n = 0; n < sensornumber; n++ )
		getWinkelDifferenz ( n , sensors++ );
	
	return getSensorfeldGroesse (); //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden
}

/**
 *Setzt den Winkelgeschwindigkeisparameter eines Motor-Joints auf einen bestimmten Wert.
 *@param motors Zeiger auf das Array mit Werten zwischen [-1,1] 
 *@param motornumber Laenge des Arrays aus dem die neuen Motorwerte gelesen werden.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
void Schlange::setMotors ( const motor* motors, int motornumber )
{
	for ( int n = 0; n < motornumber; n++ )
		if ( n % 2 == 0 )
		{
			dJointSetUniversalParam ( getJointAt(n/2) , dParamVel , *(motors++)*geschwindigkeitsfaktor );
			dJointSetUniversalParam ( jointliste[n/2] , dParamFMax , maxmotorkraft );
		}
		else
		{
			dJointSetUniversalParam ( jointliste[n/2] , dParamVel2 , *(motors++)*geschwindigkeitsfaktor );
			dJointSetUniversalParam ( jointliste[n/2] , dParamFMax2 , maxmotorkraft );
		}
}	

/**
 *Gibt die Anzahl der Motoren an, die zu einem Roboter gehören.
 *@return Anzahl der Motoren
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
 int Schlange::getMotorNumber()
{
	return 2*getJointAnzahl ();
}
	
/**
 *Ließt die aktuellen Sensordaten erneut in die Sensorspeicherfelder.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
void Schlange::sensoraktualisierung ( )
{
	for ( int n = 0; n < getSensorfeldGroesse (); n++ )
	{
		sensorfeld[n].istwinkel_alt = sensorfeld[n].istwinkel;
		
		if ( n % 2 == 0 )
			sensorfeld[n].istwinkel = dJointGetUniversalAngle1 ( getJointAt (n/2) );
		else
			sensorfeld[n].istwinkel = dJointGetUniversalAngle2 ( getJointAt (n/2) );
	}
}

/** returns position of robot 
Die Position der Schlange wird durch die Position des ersten Schlangengliedes bestimmt.
@param pos vector of desired position (x,y,z)
*/
Position Schlange::getPosition ()
{
	const dReal* tmpPos;
	Position returnPos;
	tmpPos = dBodyGetPosition ( getObjektAt(0).body );
	returnPos.x = tmpPos[0];
	returnPos.y = tmpPos[1];
	returnPos.z = tmpPos[2];

	return returnPos;
}

/** returns position of robot 
Die Position der Schlange wird durch die Position des ersten Schlangengliedes bestimmt.
@param pos vector of desired position (x,y,z)
*/
Position Schlange::getPosition ( int n )
{
	const dReal* tmpPos;
	Position returnPos;
	tmpPos = dBodyGetPosition ( getObjektAt (n).body );
	returnPos.x = tmpPos[0];
	returnPos.y = tmpPos[1];
	returnPos.z = tmpPos[2];

	return returnPos;
}

/**
 *Gibt die aktuellen Controler-Steuerungsparameter als Text aus.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
void Schlange::getStatus ()
{
	for ( int n = 0; n < getSensorfeldGroesse (); dsPrint ( "Sensor %i: %lf\n" , n , sensorfeld[n++].istwinkel ) );
}
