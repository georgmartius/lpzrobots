/************************************************************************/
/*Robotergrundkonstrukt fuer das eigene ODE-Robotersystem des Authors	*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.3							*/
/*									*/
/************************************************************************/

#include <stdio.h>
#include <math.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <vector>

/****************************************************/




#define SCHLANGENGLIEDLAENGE 0.6
#define SCHLANGENGLIEDDICKE 0.3
#define MOTOR_WINKELGESCHWINDIGKEITSFAKTOR 5
#define DIFFERENZVERSTAERKUNGSFAKTOR 1




#define MAX_CONTROLERANZAHL 8

/******************************************Typendeklarationen*****************************************/
typedef struct
{
	double x;
	double y;
	double z;
} raumvektor;

typedef struct
{
   	double istwinkel; //aktueller Winkelwert, im Bogenmass
	double istwinkel_alt;
	double sollwinkel;
	//double winkelgeschwindigkeit; //Geschwindigkeit und Richtung des winkels

        double x;
        double y;
        double z;
} Sensor;


/***********************************Hilfsfunktionendefinitionen***************************************/


/**
 *Dies ist eine universelle Basisklasse, welche die Grundfunktionalitaet eines Roboters.
 *@author Marcel Kretschmann
 *@version development
 **/
class Roboter : public AbstractRobot
{

public:
	//Sensoren sind allgemein zugänglich, da ein eventuelles schreiben oder loeschen sowieso nur eine Runde bestehen bleibt
	vector <Sensor> sensorfeld;

protected:
	//Eigenschaften
	int roboterID;
	
	vector <objekt> objektliste;
	vector <dJointID> jointliste;
	vector <dJointID> motorliste;
	
	//Controlervariablen
	RobotLearnControl_Gnu<MAX_CONTROLERANZAHL,10>  roboter_controller;
	NoiseGenerator<MAX_CONTROLERANZAHL> noise_gen;
	
	double epsilon;
	double noise;
	double rho;
	double Delta;
	double a_faktor;
	double mue;
	int NumberStepsForAveraging;
	int NumberStepsOfDelay;
	double m;
	
	double  y[MAX_CONTROLERANZAHL-1];
	int t;

public:

	/**
	 *Stanndartkonstruktor
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	Roboter ()
	{
	
	}

	/**
	 *Konstruktor
	 *@param int Roboterkennummer
	 *@param dWorldID Referenz auf die ODE-Simulationswelt, in der der Roboter angelegt werden soll
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	Roboter ( int startRoboterID , dWorldID welt , dSpaceID raum )
	{
		roboterID = startRoboterID;
		
		//**********************Controler-Abschnitt********************
		for (int i=0; i<MAX_CONTROLERANZAHL;i++)
		{
			y[i]=0.0;
		}
		
		noise=0.1;
		
		epsilon=roboter_controller.getEps();
		rho=roboter_controller.getRho();
		Delta=roboter_controller.getDelta();
		NumberStepsForAveraging=roboter_controller.getNumberStepsForAveraging();
		NumberStepsOfDelay=roboter_controller.getNumberStepsOfDelay();
		m=roboter_controller.getM();


	}
	
	/**
	 *Gibt die Anzahl der Objekte aus denen der Roboter aufgebaut ist.
	 *@return int Anzahl der Objekte aus denen der Roboter besteht
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual int getObjektAnzahl ()
	{
		return objektliste.size ();
	}
	
	/**
	 *Gibt die Anzahl der Joints aus denen der Roboter aufgebaut ist an.
	 *@return int Anzahl der Joints
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	int getJointAnzahl ()
	{
		return jointliste.size ();
	}
	
	/**
	 *Gibt die Anzahl der Motoren an, die zu einem Roboter gehören.
	 *@return Anzahl der Motoren
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	int getMotorenAnnzahl ()
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
	objekt getObjektAt ( int n )
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
	dJointID getJointAt ( int n )
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
	dJointID getMotorAt ( int n )
	{
		return motorliste[n];
	}
	
	/**
	 *Setzt den Noise-Parameter des Roboters.
	 *@param double neuer Noise-Wert
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	void setNoise ( double neues_noise )
	{
		noise = neues_noise;
	}
	
	/**
	 *Setzt den Winkelgeschwindigkeisparameter eines Motor-Joints auf einen bestimmten Wert.
	 *@param int Nummer des Motors in der Motorliste
	 *@param double neue Winkelgeschwindigkeit des Motors
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	void setMotorWinkel ( int motornummer , double winkelgeschwindigkeit )
	{
		dJointSetAMotorParam ( motorliste[motornummer] , dParamVel , winkelgeschwindigkeit*MOTOR_WINKELGESCHWINDIGKEITSFAKTOR );
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
	void getWinkelDifferenz ( int motor , double* X )
	{
		//Eingangsdaten sind die Winkeldifferenzen eines Roboterberechnungsschrittes
		//Abfangen des Winkeldifferenzsprunges bei ueberschreiten er 2PI-Marke
		//tritt auf wenn die -PI- oder die PI-Marke ueberschritten wird
		if ( ( sensorfeld[motor].istwinkel - sensorfeld[motor].istwinkel_alt < -PI )
		|| ( sensorfeld[motor].istwinkel - sensorfeld[motor].istwinkel_alt >  PI ) )
		{
			//1. Fall(PI-Marke wird ueberschritten, also annaeherung vom Positiven) -> es ergibt sich eine negative Winkeldifferenz
			if ( sensorfeld[motor].istwinkel-sensorfeld[motor].istwinkel_alt > PI )
				*X = (( PI - sensorfeld[motor].istwinkel_alt ) + ( -PI - sensorfeld[motor].istwinkel ))*DIFFERENZVERSTAERKUNGSFAKTOR;
			//2. Fall(-PI-Marke wird ueberschritten, also Annaeherung vom Negativen) -> es ergibt sich eine positive Winkeldifferenz
			if ( sensorfeld[motor].istwinkel-sensorfeld[motor].istwinkel_alt < -PI )
				*X = -(( -PI - sensorfeld[motor].istwinkel_alt ) + ( PI - sensorfeld[motor].istwinkel ))*DIFFERENZVERSTAERKUNGSFAKTOR;
		}
		else
			*X=(sensorfeld[motor].istwinkel-sensorfeld[motor].istwinkel_alt)*DIFFERENZVERSTAERKUNGSFAKTOR;
	}
	
	/**
	 *Fuegt einen leeren Sensor zum Sensor-Array hinzu.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	void addSensor ()
	{
		Sensor tmpSensor;	
		sensorfeld.push_back ( tmpSensor );
	}
	
	/**
	 *Entfernt einen bestehenden Sensor aus dem Sensor-Array, dies ist immer der letzte Sensor
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	void delSensor ()
	{
		sensorfeld.pop_back ( );
	}
	
	/**
	 *Gibt die Anzahl der Sensorbloecke die existieren zurueck.
	 *@return int Anzahl der Sensorfeldelemente
	 *@author Marcel Kretschmann
	 *@version alpha 
	 **/
	int getSensorfeldGroesse ()
	{
		return sensorfeld.size ();
	}
	
	/**
	 *Ließt die aktuellen Sensordaten erneut in die Sensorspeicherfelder.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	void sensoraktualisierung ( )
	{
		for ( int n = 0; n < getSensorfeldGroesse (); n++ )
		{
			
			sensorfeld[n].istwinkel_alt = sensorfeld[n].istwinkel;
			
			sensorfeld[n].istwinkel = dJointGetAMotorAngle ( getMotorAt ( n ) , 0 );
		}
	}
	
	/**
	 *Setzt bei allen dem Roboter zugeordneten Motoren die neue Winkelgeschwindigkeit
	 *auf den Ausgabewert des Controlers
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	void motorAktualisierung ()
	{	
		for ( int n = 0; n < getSensorfeldGroesse (); n++ )
		{
			setMotorWinkel ( n , ((y[n])/DIFFERENZVERSTAERKUNGSFAKTOR) );
		}
	}

	/**
	 *Dies führt einen Controlerberechnungsschritt durch, bei dem das zugrundeliegende neuronale Netz
	 *eine Takt weiter geführt wird.
	 *@return bool
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	bool StepRobot()
	{
		double X[MAX_CONTROLERANZAHL];

		// Sensordaten auslesen
		//_________________________
	
		//Auslesen aller existenten Sensoren
		for ( int n = 0; n < getSensorfeldGroesse (); n++ )
			getWinkelDifferenz ( n , &X[n] );
		//alle anderen Controlereingabedaten werden direkt aus den vorhergehenden Ausgabedaten bestimmt
		for ( int n = getSensorfeldGroesse (); n < MAX_CONTROLERANZAHL; n++ )
			X[n] = y[n];
		
	
		noise_gen.addColoredNormallyDistributedNoise(X, noise*0.5); //normally distributed noise -> noise=0.1 soll variance 0.05 sein, dehalb mit 0.5 multiplizieren
	
		noise_gen.addColoredUniformlyDistributedNoise(X, -noise, noise);   // uniformly distributed noise -> using min=-noise, max=noise
	
		roboter_controller.makeStep(X,y);
	
		// Motorkommando setzen
		//_____________________
	
		motorAktualisierung ( );
		
		t+=1   ;
		//pas++  ;
	
		return( 1 );
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
	bool kollisionsermittlung ( dGeomID o1 , dGeomID o2 )
	{
		return false;
	}
	
	/**
	 *Zeichnet die Koerper-Geometrieobjekte.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	void zeichneRoboter ()
	{
		
	}
	
	/**
	 *Gibt die aktuellen Controler-Steuerungsparameter als Text aus.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	void getParameterStatus ()
	{
		dsPrint ( "\nSteuerungsparameter Roboter: %i\n-------------------------------------------------\n" , roboterID );
		dsPrint ( "(e)psilon = %lf\n(r) rho = %lf\n(n)noise = %lf\n(d)Delta = %lf\n(a)a_faktor = %lf\n(m)mue = %lf\n" , epsilon , rho , noise , Delta , a_faktor , mue );
	}
	
};

/**
 *Hier wird die zu setzende Winkeldifferenz zum aktuellen Winkel des Motors hinzugegeben.
 *Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam.
 *Hier wird die Kollission untersucht.
 *@param void*
 *@param dGeomID erstes Geometrieobjekt das an der Kollission beteiligt ist
 *@param dGeomID zweites Geometrieobjekt das an der Kollission beteiligt ist
 *@author Marcel Kretschmann
 *@version development
 **/
void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
 	int i,n;
 	const int N = 10;
 	dContact contact[N];
 	bool kollission = false;
 	
		for ( unsigned int n = 0; n < robotersammlung.size (); n++ )
			if ( (*robotersammlung.back ()).kollisionsermittlung ( o1 , o2 ) == true )
				kollission = true;
	
		if ( kollission == false )
		{
			n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
			if (n > 0) {
			for (i=0; i<n; i++)
			{
				contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
				dContactSoftERP | dContactSoftCFM | dContactApprox1;
				contact[i].surface.mu = 0.8; //normale Reibung von Reifen auf Asphalt
				contact[i].surface.slip1 = 0.005;
				contact[i].surface.slip2 = 0.005;
				contact[i].surface.soft_erp = 1;
				contact[i].surface.soft_cfm = 0.00001;
				dJointID c = dJointCreateContact (welt,contactgroup,&contact[i]);
				dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
			}
		}
 	
	}
}

