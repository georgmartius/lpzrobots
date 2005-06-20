/************************************************************************/
/*roboter.cpp								*/
/*Robotergrundkonstrukt fuer das eigene ODE-Robotersystem des Authors	*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.9							*/
/*									*/
/************************************************************************/

#include "roboter.h"


/***************************Hilfsfunktionen******************************/
double dBodyGetPositionAll ( dBodyID basis , int para )
{
    const dReal* pos;

    pos = dBodyGetPosition ( basis );

    switch (para)
    {
        case 1: return pos[0]; break; //X
        case 2: return pos[1]; break; //Y
        case 3: return pos[2]; break; //Z
    }
	return 0;
}
/***********************************************************************/



/**
 *Konstruktor
 *@param int Roboterkennummer
 *@param dWorldID Referenz auf die ODE-Simulationsworld, in der der Roboter angelegt werden soll
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
 Roboter::Roboter ( int startRoboterID , dWorldID* welt , dSpaceID* raum , dJointGroupID* start_contactgroup , int start_Sensoranzahl ) :
    AbstractRobot::AbstractRobot ( welt , raum , start_contactgroup )
{
	roboterID = startRoboterID;
	for ( int n = 0; n++ < start_Sensoranzahl; addSensor() );
}

/**
 *Destruktor
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
Roboter::~Roboter()
{
}


/*************************************************************/

/**
 *Zeichnet die Koerper-GeometrieObjekte.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
 void Roboter::draw()
{

}

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
@param pos desired position of the robot in struct Position
@param c desired color for the robot in struct Color
*/
void Roboter::place (Position pos, Color *c)
{
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
 bool Roboter::collisionCallback(void *data, dGeomID o1, dGeomID o2)
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
		if ( Roboter::kollisionsermittlung ( o1 , o2 ) == true )
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
 int Roboter::getSensors ( sensor* sensors, int sensornumber )
{
	Roboter::sensoraktualisierung ();
	for ( int n = 0; n < sensornumber; n++ )
		getWinkelDifferenz ( sensornumber , sensors++ );
		return getSensorfeldGroesse (); //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden
}

/**
 *Setzt den Winkelgeschwindigkeisparameter eines Motor-Joints auf einen bestimmten Wert.
 *@param motors Zeiger auf das Array mit Werten zwischen [-1,1] 
 *@param motornumber Laenge des Arrays aus dem die neuen Motorwerte gelesen werden.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
 void Roboter::setMotors ( const motor* motors, int motornumber )
{
	for ( int n = 0; n < motornumber; n ++ )
		dJointSetAMotorParam ( motorliste[n] , dParamVel , (*(motors++))*10 );
}


/** returns number of sensors
*/
 int Roboter::getSensorNumber()
{
	return getSensorfeldGroesse ();
}

/** returns number of motors
*/
 int Roboter::getMotorNumber()
{
	return getMotorAnzahl ();
}

/** returns a list with the positionvectors of all segments of the robot
@param poslist list with positionvectors (of all robot segments) (free after use!)
@return length of the list
*/
int Roboter::getSegmentsPosition ( vector<Position> &poslist )
{
	const dReal* tmp;
	for ( int n = 0; n < getObjektAnzahl (); n++ )
	{
		 tmp = dBodyGetPosition ( getObjektAt ( n ).body );
		 poslist[n].x = tmp[0];
		 poslist[n].y = tmp[1];
		 poslist[n].z = tmp[2];
	}
	return getObjektAnzahl ();
}



/***********************************************************/




/**
 *Gibt die Anzahl der Objekte aus denen der Roboter aufgebaut ist.
 *@return int Anzahl der Objekte aus denen der Roboter besteht
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
 int Roboter::getObjektAnzahl ()
{
	return objektliste.size ();
}

/**
 *Gibt die Anzahl der Joints aus denen der Roboter aufgebaut ist an.
 *@return int Anzahl der Joints
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
int Roboter::getJointAnzahl ()
{
	return jointliste.size ();
}

/**
 *Gibt die Anzahl der Motoren an, die zu einem Roboter gehören.
 *@return Anzahl der Motoren
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
int Roboter::getMotorAnzahl ()
{
	return motorliste.size ();
}

/**
 *Gibt ein Objekt zurueck, welches sowohl einen Verweis auf ODE-Body- als auch ODE-Geom-Objekte enthaellt.
 *@param int Die Position in der Liste der Objekte ueber die ein Roboter verfuegt.
 *@return objekt
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
Object Roboter::getObjektAt ( int n )
{
	return objektliste[n];
}

/**
 *Gibt einen Joint zurueck, welcher zur Roboterkonstruktion gehoert.
 *@param int Die Position in der Liste der Joints die zum Roboterkonstrukt gehoeren.
 *@return dJointID
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
dJointID Roboter::getJointAt ( int n )
{
	return jointliste[n];
}

/**
 *Gibt einen Motor-Joint zurueck, welcher zwischen zwei Objekten des Roboters anliegt,
 *die mit einem Joint verbunden sind.
 *@param int Die Position in er Liste der Motor-Joints an.
 *@return dJointID
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
dJointID Roboter::getMotorAt ( int n )
{
	return motorliste[n];
}

/**
 *Ermittelt die Differenz von zwei Sensormesswerten eines Motor-Joints
 *zwischen zwei Berechnungszeitschritten.
 *Diese Differenz wird an eine uebergebene Stelle im Speicher gespeichert.
 *@param int Position des anzupassenden Motor-Joints in der Liste der Motor-Joints.
 *@param double* Speicherort fuer den ermittelten Sensorwert.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
void Roboter::getWinkelDifferenz ( int motor , double* X )
{
	//Eingangsdaten sind die Winkeldifferenzen eines Roboterberechnungsschrittes
	//Abfangen des Winkeldifferenzsprunges bei ueberschreiten er 2PI-Marke
	//tritt auf wenn die -PI- oder die PI-Marke ueberschritten wird
	if ( ( sensorfeld[motor].istwinkel - sensorfeld[motor].istwinkel_alt < -M_PI )
	|| ( sensorfeld[motor].istwinkel - sensorfeld[motor].istwinkel_alt >  M_PI ) )
	{
		//1. Fall(PI-Marke wird ueberschritten, also annaeherung vom Positiven) -> es ergibt sich eine negative Winkeldifferenz
		if ( sensorfeld[motor].istwinkel-sensorfeld[motor].istwinkel_alt > M_PI )
			*X = (( M_PI - sensorfeld[motor].istwinkel_alt ) + ( -M_PI - sensorfeld[motor].istwinkel ));
		//2. Fall(-PI-Marke wird ueberschritten, also Annaeherung vom Negativen) -> es ergibt sich eine positive Winkeldifferenz
		if ( sensorfeld[motor].istwinkel-sensorfeld[motor].istwinkel_alt < -M_PI )
			*X = -(( -M_PI - sensorfeld[motor].istwinkel_alt ) + ( M_PI - sensorfeld[motor].istwinkel ));
	}
	else
		*X=(sensorfeld[motor].istwinkel-sensorfeld[motor].istwinkel_alt);
}

/**
 *Fuegt einen leeren Sensor zum Sensor-Array hinzu.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
void Roboter::addSensor ()
{
	Sensor tmpSensor;	
	sensorfeld.push_back ( tmpSensor );
}
	
/**
 *Entfernt einen bestehenden Sensor aus dem Sensor-Array, dies ist immer der letzte Sensor
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
void Roboter::delSensor ()
{
	sensorfeld.pop_back ( );
}
	
/**
 *Gibt die Anzahl der Sensorbloecke die existieren zurueck.
 *@return int Anzahl der Sensorfeldelemente
 *@author Marcel Kretschmann
 *@version alpha 
 **/
int Roboter::getSensorfeldGroesse ()
{
	return sensorfeld.size ();
}
	
/**
 *Ließt die aktuellen Sensordaten erneut in die Sensorspeicherfelder.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
void Roboter::sensoraktualisierung ( )
{
	for ( int n = 0; n < getSensorfeldGroesse (); n++ )
	{
		sensorfeld[n].istwinkel_alt = sensorfeld[n].istwinkel;
		
		sensorfeld[n].istwinkel = dJointGetAMotorAngle ( getMotorAt ( n ) , 0 );
	}
}

/**
 *Diese Funktion ermittelt ob es zwischen bestimmten Elementen des Roboters eine kollision gibt,
 *und verhindert, dass diese Kollision in die globale Kollisionsbehandlung mit einfließt.
 *@param dGeomID Geometrieobjekt 1, dass an der Kollision beteiligt ist
 *@param dGeomID Geometrieobjekt 2, dass an der Kollision beteiligt ist
 *@return bool true, wenn keine Kollision zwischen den beiden Geometrieobjekten erfolgt, false sonst
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
bool Roboter::kollisionsermittlung ( dGeomID o1 , dGeomID o2 )
{
	return false;
}
	
/**
 *Gibt die aktuellen Controler-Steuerungsparameter als Text aus.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
void Roboter::getParameterStatus ()
{

}
