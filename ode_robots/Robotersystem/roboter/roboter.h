/************************************************************************/
/*roboter.h								*/
/*Robotergrundkonstrukt fuer das eigene ODE-Robotersystem des Authors	*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.3							*/
/*									*/
/************************************************************************/

#include <stdio.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <vector>

using namespace std;

#include "../../abstractrobot.h"

/****************************************************/


#define SCHLANGENGLIEDLAENGE 0.6
#define SCHLANGENGLIEDDICKE 0.3


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
 *Dies ist eine universelle Basisklasse, welche die Grundfunktionalitaet eines Roboters beinhaltet.
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
	
	vector <Object> objektliste;
	vector <dJointID> jointliste;
	vector <dJointID> motorliste;
	
	//Controlervariablen	
	double epsilon;
	double noise;
	double rho;
	double Delta;
	double a_faktor;
	double mue;
	int NumberStepsForAveraging;
	int NumberStepsOfDelay;
	double m;

	int t;

public:

	/**
	 *Stanndartkonstruktor
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	Roboter ();

	/**
	 *Konstruktor
	 *@param int Roboterkennummer
	 *@param dWorldID Referenz auf die ODE-Simulationswelt, in der der Roboter angelegt werden soll
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	Roboter ( int startRoboterID , dWorldID welt , dSpaceID raum , int startSensoranzahl );
	
	/**
	 *Destruktor
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual Roboter::~Roboter();
	/*************************************************************/
	
	/**
	 *Zeichnet die Koerper-GeometrieObjekte.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual void draw();
	
	/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
  	@param pos desired position of the robot in struct Position
    	@param c desired color for the robot in struct Color
	*/
	virtual void place(Position pos, Color *c);
	
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
	virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
	
	/**Gibt die Sensorwerte aus dem Roboterinternensensorfeld an einen uebergebenen Speicherort aus.
	 *@param sensor* sensors Zeiger auf den Zielort der Sensordaten
	 *@param int sensornumber Laenge des Sensorenarrays in das gespeichert werden soll
	 *@return int Anzahl der Sensoren in die schon ein aktueller Wert geschrieben wurde
	 **/
	virtual int getSensors(sensor* sensors, int sensornumber);
	
	/**Setzt die Motorwerte auf die Werte im uebergebenen Werte-Array
	 *@param motor* motors motors scaled to [-1,1] 
	 *@param motornumber length of the motor array
	 **/
	virtual void setMotors(motor* motors, int motornumber);	
	
	
	/** returns number of sensors
	*/
	virtual int getSensorNumber();
	
	/** returns number of motors
	*/
	virtual int getMotorNumber();
	
	
	/** returns position of robot 
	@param pos vector of desired position (x,y,z)
	*/
	virtual void getPosition(dVector3& pos) = 0;
	
	
	/** returns a list with the positionvectors of all segments of the robot
	@param poslist list with positionvectors (of all robot segments) (free after use!)
	@return length of the list
	*/
	virtual int getSegmentsPosition(dVector3*& poslist);
	
	
	
	/***********************************************************/
	/**
	 *Gibt die Anzahl der Objekte aus denen der Roboter aufgebaut ist.
	 *@return int Anzahl der Objekte aus denen der Roboter besteht
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual int getObjektAnzahl ();
	
	/**
	 *Gibt die Anzahl der Joints aus denen der Roboter aufgebaut ist an.
	 *@return int Anzahl der Joints
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual int getJointAnzahl ();
	
	/**
	 *Gibt die Anzahl der Motoren an, die zu einem Roboter gehören.
	 *@return Anzahl der Motoren
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual int getMotorAnzahl ();
	
	/**
	 *Gibt ein Objekt zurueck, welches sowohl einen Verweis auf ODE-Body- als auch ODE-Geom-Objekte enthaellt.
	 *@param int Die Position in der Liste der Objekte ueber die ein Roboter verfuegt.
	 *@return Object
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual Object getObjektAt ( int n );
	
	/**
	 *Gibt einen Joint zurueck, welcher zur Roboterkonstruktion gehoert.
	 *@param int Die Position in der Liste der Joints die zum Roboterkonstrukt gehoeren.
	 *@return dJointID
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual dJointID getJointAt ( int n );
	
	/**
	 *Gibt einen Motor-Joint zurueck, welcher zwischen zwei Objekten des Roboters anliegt,
	 *die mit einem Joint verbunden sind.
	 *@param int Die Position in er Liste der Motor-Joints an.
	 *@return dJointID
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual dJointID getMotorAt ( int n );
	
	/**
	 *Setzt den Winkelgeschwindigkeisparameter eines Motor-Joints auf einen bestimmten Wert.
	 *@param int Nummer des Motors in der Motorliste
	 *@param double neue Winkelgeschwindigkeit des Motors
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual void setMotorWinkel ( int motornummer , double winkelgeschwindigkeit );
	
	/**
	 *Ermittelt die Differenz von zwei Sensormesswerten eines Motor-Joints
	 *zwischen zwei Berechnungszeitschritten.
	 *Diese Differenz wird an eine uebergebene Stelle im Speicher gespeichert.
	 *@param int Position des anzupassenden Motor-Joints in der Liste der Motor-Joints.
	 *@param double* Speicherort fuer den ermittelten Sensorwert.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual void getWinkelDifferenz ( int motor , double* X );
	
	/**
	 *Fuegt einen leeren Sensor zum Sensor-Array hinzu.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual void addSensor ();
	
	/**
	 *Entfernt einen bestehenden Sensor aus dem Sensor-Array, dies ist immer der letzte Sensor
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual void delSensor ();
	
	/**
	 *Gibt die Anzahl der Sensorbloecke die existieren zurueck.
	 *@return int Anzahl der Sensorfeldelemente
	 *@author Marcel Kretschmann
	 *@version alpha 
	 **/
	virtual int getSensorfeldGroesse () = 0;
	
	/**
	 *Ließt die aktuellen Sensordaten erneut in die Sensorspeicherfelder.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual void sensoraktualisierung ( ) = 0;
	
	/**
	 *Diese Funktion ermittelt ob es zwischen bestimmten Elementen des Roboters eine kollision gibt,
	 *und verhindert, dass diese Kollision in die globale Kollisionsbehandlung mit einfließt.
	 *@param dGeomID Geometrieobjekt 1, dass an der Kollision beteiligt ist
	 *@param dGeomID Geometrieobjekt 2, dass an der Kollision beteiligt ist
	 *@return bool true, wenn keine Kollision zwischen den beiden Geometrieobjekten erfolgt, false sonst
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual bool kollisionsermittlung ( dGeomID o1 , dGeomID o2 );
	
	/**
	 *Gibt die aktuellen Controler-Steuerungsparameter als Text aus.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual void getParameterStatus ();
	
};


