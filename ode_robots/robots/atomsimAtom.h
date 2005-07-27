/************************************************************************/
/*atomsimAtom.h								*/
/*atom modell which tries to repesent some aspekts of molecular atoms	*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/
#ifndef __ATOMSIMATOM_H
#define __ATOMSIMATOM_H


using namespace std;

#include "roboter.h"


#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

/******************************type declarations*************************/

/******************************class declarations*************************/
/**
 *Instanzen der Klasse Atom, sind die elementaren Bestandteile eines Roboters in dieser Simulation. Sie haben 
 *verschiedene Eigenschaften, wie etwa eine Masse oder einen Radius. Atome besitzen ebenso eine Atomhuelle, deren Radius 
 *bestimmt inwieweit sie man anderen Atomen zusammenbinden koennen, diese Abspalten oder sich von diesen abstossen.
 *Ein Atom hat immer einen Ursprung, also ein weiteres Atom, an dem es gebunden wurde. Nur das erste Atom eines Roboters 
 *hat keinen Ursprung. Desweiteren hat jedes Atom eine Liste von allen anderen Atomen, die an es gebunden haben.
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
class atomsimAtom
{
private:
	int roboterID;
	int atomID;
	int* atomIDzaehler;
	
	dWorldID welt;
	dSpaceID raum;
	
	dBodyID body;
	dGeomID atom_geom;
	dGeomID atomhuelle_geom;
	
	atomsimAtom* ursprung;
	vector <atomsimAtom*> atomliste; //Liste aller von diesem Atom ausgehenden Atome
	
	dJointID ursprungjoint;
	dJointID ursprungmotor;
	
public:
	vector <dJointID> jointliste; //Liste aller Joints zwischen allen diesem Atom und allen von diesem Atom ausgehenden Atome

	vector <dJointID> motorliste; //Liste aller Motor-Joints zwischen diesem und allen verbundenen Atomen
private:	
	//*****chemische Kennwerte
	raumvektor farbe; //um verschiedene Atome darzustellen
	double atomradius;
	double atomhuelleradius;
	double masse;
	double bindungsstaerke;
	double abspaltstaerke;
	unsigned int maxatombindungszahl;
	unsigned int bindungsblockdauer;
	double maxmotorkraft;
	double motorgeschwindigkeitsfaktor;
	
	raumvektor kollisionsvektor1 , kollisionsvektor2; //Stossverktoren bei der Bindung des Atoms an ein anderes
	int bindungsblock;
	
public:
	
	/**
 	 *Konstruktor
	 *@param int Diese Nummer ordnet ein Atom schon bei dessen erstellung einem bestimmten Roboter zur. Allerdings funktioniert dies nur fuer das erste Atom eines Roboters. Das alleinige setzen dieses Parameters genuegt nicht um eine zuordnung zu einem Roboter vorzunehmen. Im Normalfall sollte also hier der Wert 0 (also keine Roboterzuordnung) gesetzt werden.
	 *@param double x-Anteil der Weltkoordinaten
	 *@param double y-Anteil der Weltkoordinaten
	 *@param double z-Anteil der Weltkoordinaten
	 *@param double Radius des Atoms
	 *@param double Radius der unsichtbaren Atomhuelle
	 *@param double Masse des Atoms
	 *@param double Bindungsstaerke des Atoms, also der Wert, der bei einer Kollision mindestens erreicht werden muss, damit die beiden kollisdierenden Atome miteinander eine Bindung eingehen
	 *@param double Abspaltstaerke des Atoms, also der Wert, der bei einer Kollision mindestens erreicht werden muss, damit das Atom, welches von beiden Stosspartnern zu einem Roboter gehoert, von diesem Abgespalten werden kann
	 *@param double roter Farbanteil an der Atomdarstellungsfarbe
	 *@param double gruener Farbanteil an der Atomdarstellungsfarbe
	 *@param double blauer Farbanteil an der Atomdarstellungsfarbe
 	 *@author Marcel Kretschmann
 	 *@version alpha 1.0
 	 **/
	atomsimAtom ( int start_roboterID , int* atomIDzaehler , dWorldID start_welt , dSpaceID start_raum , double x ,double y , double z , double start_radius , double start_huelleradius , double start_masse , double start_bindungsstaerke , double start_abspaltstaerke , unsigned int start_maxatombindungszahl , unsigned int start_bindungsblockdauer , double start_maxmotorkraft , double start_motorgeschwindigkeitsfaktor , double r , double g , double b );
	
	/**
 	 *Destruktor
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual ~atomsimAtom ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual int* getAtomIDzaehler ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual dBodyID getBody ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual dGeomID getAtomGeom ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual dGeomID getAtomhuelleGeom ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setSpace ( dSpaceID neuer_raum );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getX ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getY ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getZ ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setXYZ ( double neuesX , double neuesY , double neuesZ );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual atomsimAtom* getUrsprung ( );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setUrsprung ( atomsimAtom* neuer_ursprung );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual dJointID getUrsprungJoint ( );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setUrsprungJoint ( dJointID neuer_ursprungjoint );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual dJointID getUrsprungMotor ( );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setUrsprungMotor ( dJointID neuer_ursprungmotor );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual int getRoboterID ( );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setRoboterID ( int neue_roboterID );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual int getAtomID ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getRadius ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getHuellenradius ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getMasse ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual int getBindungsblock ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setBindungsblock ( int neuer_bindungsblock );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getBindungsstaerke ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setBindungsstaerke ( double neue_bindungsstaerke );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getAbspaltstaerke ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setAbspaltstaerke ( double neue_abspaltstaerke );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual unsigned int getMaxatombindungszahl ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setMaxatombindungszahl ( unsigned int neue_maxatombindungszahl  );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual unsigned int getBindungsblockdauer ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setBindungsblockdauer ( unsigned int neue_bindungsblockdauer  );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getMaxmotorkraft ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setMaxmotorkraft ( double neue_maxmotorkraft  );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getMotorgeschwindigkeitsfaktor ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setMotorgeschwindigkeitsfaktor ( double neue_motorgeschwindigkeitsfaktor  );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual raumvektor getKollisionsvektor1 ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual raumvektor getKollisionsvektor2 ();

	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual int getAnzahlAtome ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual int getAnzahlJoints ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual int getAnzahlMotoren ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual atomsimAtom* getAtomAt ( int n );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual dJointID getJointAt ( int n );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual dJointID getMotorAt ( int n );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void delAtomAt ( int n );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void delJointAt ( int n );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void delMotorAt ( int n );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getColorR ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getColorG ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual double getColorB ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual raumvektor getKraftraumvektor ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void atomInfo ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	//bezieht sich auf das Atom, an welches das Stoßende Atom gebunden wird
	virtual bool atombindung ( atomsimAtom* a2 , raumvektor kraftraumvektor1 , raumvektor kraftraumvektor2 );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	//bezieht sich auf das Atom was abgespalten wird
	virtual bool atomabspaltung ( atomsimAtom* a2 , int rekursionsZaehler );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual int kollision ( atomsimAtom* a2 );
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void drawAtom ();
	
	/**
 	 *
 	 *@author Marcel Kretschmann
 	 *@version
 	 **/
	virtual void setMotorWinkel ( int motornummer , double winkelgeschwindigkeit );

};

#endif

