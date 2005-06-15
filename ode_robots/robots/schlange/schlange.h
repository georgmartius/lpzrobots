/************************************************************************/
/*schlange.h								*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

using namespace std;

#include "../roboter/roboter.h"


//dadurch wird mit den Double-Genauigkeitszeichenmethoden gearbeitet
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

/****************************************************/


#define SCHLANGENGLIEDLAENGE 0.6
#define SCHLANGENGLIEDDICKE 0.3


#define MAX_CONTROLERANZAHL 8

class Schlange : public Roboter
{
private:
	int schlangenarmanzahl;
	double gliederdurchmesser;
	double gliederlaenge;

public:
	
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
	Schlange ( int startRoboterID , dWorldID* welt , dSpaceID* raum , dJointGroupID* start_contactgroup , int start_Sensoranzahl , double start_x , double start_y , double start_z , int armanzahl , double glieder_laenge , double glieder_durchmesser , double glieder_abstand , double glieder_masse , double maxMotorKraft );
	
	/**
	*Destruktor
	*@author Marcel Kretschmann
	*@version alpha 1.0
	**/
	virtual ~Schlange();
	
	/**
	*Zeichnet die Koerper-GeometrieObjekte.
	*@author Marcel Kretschmann
	*@version alpha 1.0
	**/
	virtual void draw();
	
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
	bool kollisionsermittlung ( dGeomID o1 , dGeomID o2 );
	
	/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
	@param pos desired position of the robot in struct Position
	@param c desired color for the robot in struct Color
	*/
	virtual void place (Position pos, Color *c);
	
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

	/** returns position of robot 
	Die Position der Schlange wird durch die Position des ersten Schlangengliedes bestimmt.
	@param pos vector of desired position (x,y,z)
	*/
	virtual Position getPosition ();
	
	/** returns position of robot 
	Die Position der Schlange wird durch die Position des ersten Schlangengliedes bestimmt.
	@param pos vector of desired position (x,y,z)
	*/
	virtual Position Schlange::getPosition ( int n );
};
