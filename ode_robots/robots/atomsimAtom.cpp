/************************************************************************/
/*atomsimAtom.h								*/
/*atom modell which tries to repesent some aspekts of molecular atoms	*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

using namespace std;

#include "atomsimAtom.h"

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
atomsimAtom::atomsimAtom ( int start_roboterID , int* start_atomIDzaehler , dWorldID* start_welt , dSpaceID* start_raum , double x ,double y , double z , double start_radius , double start_huelleradius , double start_masse , double start_bindungsstaerke , double start_abspaltstaerke , unsigned int start_maxatombindungszahl , unsigned int start_bindungsblockdauer , double start_maxmotorkraft , double start_motorgeschwindigkeitsfaktor , double r , double g , double b )
{
	atomIDzaehler = start_atomIDzaehler;
	atomID = *atomIDzaehler;
	(*atomIDzaehler)++;
	
	welt = start_welt;
	raum = start_raum;
	
	masse = start_masse;
	atomradius = start_radius;
	atomhuelleradius = start_huelleradius;
	bindungsstaerke = start_bindungsstaerke;
	abspaltstaerke = start_abspaltstaerke;
	maxatombindungszahl = start_maxatombindungszahl;
	bindungsblockdauer = start_bindungsblockdauer;
	maxmotorkraft = start_maxmotorkraft;
	motorgeschwindigkeitsfaktor = start_motorgeschwindigkeitsfaktor;
		
	farbe.x = r; farbe.y = g; farbe.z = b;
	
	ursprung = NULL;
	//ursprungsjoint muesste automatisch nicht gesetzt sein
	
	roboterID = start_roboterID;
	
	bindungsblock = 0;

	dMass mass;
	dMassSetSphereTotal ( &mass , masse , atomradius );
	
	//Anlegen des Atoms
	body = dBodyCreate ( *welt );
	dBodySetPosition ( body , x , y , z );
	
	//Massenzuweisung
	dBodySetMass ( body , &mass );
	
	//Anlegen und zuweisen der Atomoberflaeche
	atom_geom = dCreateSphere ( *raum , atomradius );
	dGeomSetBody ( atom_geom , body );
	
	//Anlegen und zuweisen der Atomoberflaeche
	atomhuelle_geom = dCreateSphere ( *raum , atomhuelleradius );
	dGeomSetBody ( atomhuelle_geom , body );
}
	
/**
 *Destruktor
 *@author Marcel Kretschmann
 *@version
 **/
atomsimAtom::~atomsimAtom ()
{
	/*dBodyDestroy ( body );
	dJointDestroy ( ursprungjoint );
	dJointDestroy ( ursprungmotor );
	dGeomDestroy ( atom_geom );
	dGeomDestroy ( atomhuelle_geom );*/
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
int* atomsimAtom::getAtomIDzaehler ()
{
	return atomIDzaehler;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
dBodyID atomsimAtom::getBody ()
{
	return body;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
dGeomID atomsimAtom::getAtomGeom ()
{
	return atom_geom;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
dGeomID atomsimAtom::getAtomhuelleGeom ()
{
	return atomhuelle_geom;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getX ()
{
	return dBodyGetPositionAll ( body , 1 );
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getY ()
{
	return dBodyGetPositionAll ( body , 2 );
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getZ ()
{
	return dBodyGetPositionAll ( body , 3 );
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setXYZ ( double neuesX , double neuesY , double neuesZ )
{
	dBodySetPosition ( body , neuesX , neuesY , neuesZ );
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
atomsimAtom* atomsimAtom::getUrsprung ( )
{
	return ursprung;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setUrsprung ( atomsimAtom* neuer_ursprung )
{
	ursprung = neuer_ursprung;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
dJointID atomsimAtom::getUrsprungJoint ( )
{
	return ursprungjoint;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setUrsprungJoint ( dJointID neuer_ursprungjoint )
{
	ursprungjoint = neuer_ursprungjoint;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
dJointID atomsimAtom::getUrsprungMotor ( )
{
	return ursprungmotor;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setUrsprungMotor ( dJointID neuer_ursprungmotor )
{
	ursprungmotor = neuer_ursprungmotor;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
int atomsimAtom::getRoboterID ( )
{
	return roboterID;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setRoboterID ( int neue_roboterID )
{
	roboterID = neue_roboterID;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
int atomsimAtom::getAtomID ()
{
	return atomID;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getRadius ()
{
	return atomradius;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getHuellenradius ()
{
	return atomhuelleradius;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getMasse ()
{
	return masse;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
int atomsimAtom::getBindungsblock ()
{
	return bindungsblock;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setBindungsblock ( int neuer_bindungsblock )
{
	bindungsblock = neuer_bindungsblock;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getBindungsstaerke ()
{
	return bindungsstaerke;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setBindungsstaerke ( double neue_bindungsstaerke )
{
	bindungsstaerke = neue_bindungsstaerke;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getAbspaltstaerke ()
{
	return abspaltstaerke;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setAbspaltstaerke ( double neue_abspaltstaerke )
{
	abspaltstaerke = neue_abspaltstaerke;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
unsigned int atomsimAtom::getMaxatombindungszahl ()
{
	return maxatombindungszahl;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setMaxatombindungszahl ( unsigned int neue_maxatombindungszahl  )
{
	maxatombindungszahl = neue_maxatombindungszahl;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
unsigned int atomsimAtom::getBindungsblockdauer ()
{
	return bindungsblockdauer;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setBindungsblockdauer ( unsigned int neue_bindungsblockdauer  )
{
	bindungsblockdauer = neue_bindungsblockdauer;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getMaxmotorkraft ()
{
	return maxmotorkraft;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setMaxmotorkraft ( double neue_maxmotorkraft  )
{
	maxmotorkraft = neue_maxmotorkraft;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getMotorgeschwindigkeitsfaktor ()
{
	return motorgeschwindigkeitsfaktor;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setMotorgeschwindigkeitsfaktor ( double neue_motorgeschwindigkeitsfaktor  )
{
	motorgeschwindigkeitsfaktor = neue_motorgeschwindigkeitsfaktor;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
raumvektor atomsimAtom::getKollisionsvektor1 ()
{
	return kollisionsvektor1;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
raumvektor atomsimAtom::getKollisionsvektor2 ()
{
	return kollisionsvektor2;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
int atomsimAtom::getAnzahlAtome ()
{
	return atomliste.size ();
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
int atomsimAtom::getAnzahlJoints ()
{
	return jointliste.size ();
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
int atomsimAtom::getAnzahlMotoren ()
{
	return motorliste.size ();
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
atomsimAtom* atomsimAtom::getAtomAt ( int n )
{
	return atomliste[n];
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
dJointID atomsimAtom::getJointAt ( int n )
{
	return jointliste[n];
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
dJointID atomsimAtom::getMotorAt ( int n )
{
	return motorliste[n];
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::delAtomAt ( int n )
{
	vector<atomsimAtom*>::iterator it;
	for ( it = atomliste.begin (); it <= atomliste.end (); it++ )
	{
		if ( *it == getAtomAt ( n )  )
		{
			atomliste.erase ( it );
			break;
		}
	}
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::delJointAt ( int n )
{
	vector<dJointID>::iterator it;
	for ( it = jointliste.begin (); it <= jointliste.end (); it++ )
	{
		if ( *it == getJointAt ( n )  )
		{
			jointliste.erase ( it );
			break;
		}
	}
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::delMotorAt ( int n )
{
	vector<dJointID>::iterator it;
	for ( it = motorliste.begin (); it <= motorliste.end (); it++ )
	{
		if ( *it == getMotorAt ( n )  )
		{
			motorliste.erase ( it );
			break;
		}
	}
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getColorR ()
{
	return farbe.x;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getColorG ()
{
	return farbe.y;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
double atomsimAtom::getColorB ()
{
	return farbe.z;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
raumvektor atomsimAtom::getKraftraumvektor ()
{
	const dReal * kv;
	raumvektor kraftraumvektor;

	kv = dBodyGetLinearVel ( body );
	
	kraftraumvektor.x = kv[0];
	kraftraumvektor.y = kv[1];
	kraftraumvektor.z = kv[2];
	
	return kraftraumvektor;
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::atomInfo ()
{
	dsPrint ( ">>>>>>>>>>AtomInfo<<<<<<<<<<\n" );
	dsPrint ( "AtomID: %i\n" , getAtomID () );
	dsPrint ( "RoboterID: %i\n" , getRoboterID () );
	dsPrint ( "Atom-Liste: %i\n" , atomliste.size () );
	dsPrint ( "Joint-Liste: %i\n" , jointliste.size () );
	dsPrint ( "Motor-Liste: %i\n" , motorliste.size () );		
	dsPrint ( "reale Jointanzahl(=Gelenke+Motoren): %i\n" , dBodyGetNumJoints ( getBody () ) );
	dsPrint ( ">>>>>>>>AtomInfoEnde<<<<<<<<\n" );
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
//bezieht sich auf das Atom, an welches das Stoßende Atom gebunden wird
bool atomsimAtom::atombindung ( atomsimAtom* a2 , raumvektor kraftraumvektor1 , raumvektor kraftraumvektor2 )
{
	bool rueckgabewert = false;

	dJointID tmp;
	(*a2).kollisionsvektor1 = kraftraumvektor1;
	(*a2).kollisionsvektor2 = kraftraumvektor2;
		
	if ( (*a2).getBindungsblock () > 0 ) dsPrint ( ">Bindungsblock aktiv = %i\n" , (*a2).getBindungsblock () );	
	
	//Wenn die maximal erlaubte Anzahl an Bindungen pro Atom noch nicht erreicht ist
	if ( ( atomliste.size () < maxatombindungszahl ) && ( (*a2).getBindungsblock () <= 0 ) )
	{
		raumvektor achse_rechtwinklig_zur_Kollision;
		//Achse, die senkrecht auf der Ebene, die von Kraftvektor1 und Kraftvektor2 aufgespannt wird, stehen
		//also gilt es das Kreuzprodukt der beiden Vektoren zu berechnen
		achse_rechtwinklig_zur_Kollision.x = 
		kraftraumvektor1.y * kraftraumvektor2.z - kraftraumvektor1.z * kraftraumvektor2.y;

		achse_rechtwinklig_zur_Kollision.y =
		kraftraumvektor1.z * kraftraumvektor2.x - kraftraumvektor1.x * kraftraumvektor2.z;

		achse_rechtwinklig_zur_Kollision.z =
		kraftraumvektor1.x * kraftraumvektor2.y - kraftraumvektor1.y * kraftraumvektor2.x;
		//Um das Problem zu umgehen, das keine Ebene existiert, wenn einer der Vektoren den Betrag 0 hat, wird in diesem Fall einfach ein passender Wert angenommen, dies sollte aber eher selten auftreten
		if ( ( achse_rechtwinklig_zur_Kollision.x == 0 ) && ( achse_rechtwinklig_zur_Kollision.y == 0 )
		&& ( achse_rechtwinklig_zur_Kollision.z == 0 ) )
		{
			//es wird in diesen Fällen immer die Ache 1-0-0 verwendet
			achse_rechtwinklig_zur_Kollision.x = 0;
			achse_rechtwinklig_zur_Kollision.y = 1;
			achse_rechtwinklig_zur_Kollision.z = 0;
		}
	
		dsPrint ( "Bindung von %i an Roboter %i\n" , (*a2).getRoboterID () , getRoboterID ());
		dsPrint ( "mit Achse: ( %.30lf , %.30lf , %.30lf )\n" , achse_rechtwinklig_zur_Kollision.x , achse_rechtwinklig_zur_Kollision.y , achse_rechtwinklig_zur_Kollision.z );
	
		atomliste.push_back ( a2 );
	
		(*a2).setUrsprung ( this ); //das Atom an dem die kollision erfolgte wird zum Ursprung für das welches angestossen ist

		(*a2).setRoboterID ( (*this).getRoboterID () ); //das anstoßende Atom erhällt die Kennung des Roboters zu dem es nun gehört

		//Anlegen des Joints
		tmp = dJointCreateHinge ( *welt , 0 ); //diese Joints duerfen in keiner Gruppe liegen, damit sie einzeln zerstört werden koennen
		dJointAttach ( tmp , (*this).getBody () , (*a2).getBody () );
		dJointSetHingeAnchor ( tmp , (*this).getX () , (*this).getY () , (*this).getZ () );

		dJointSetHingeAxis ( tmp , achse_rechtwinklig_zur_Kollision.x , achse_rechtwinklig_zur_Kollision.y , achse_rechtwinklig_zur_Kollision.z );

		jointliste.push_back ( tmp );
		(*a2).setUrsprungJoint ( tmp );


		//Anlegen des Motors
		motorliste.push_back ( dJointCreateAMotor ( *welt , 0 ) );
		(*a2).setUrsprungMotor ( motorliste.back () );
		dJointAttach ( motorliste.back () , (*this).getBody () , (*a2).getBody () );
		dJointSetAMotorMode ( motorliste.back () , dAMotorEuler );
		dJointSetAMotorAxis ( motorliste.back () , 0 , 1 , achse_rechtwinklig_zur_Kollision.x , achse_rechtwinklig_zur_Kollision.y , achse_rechtwinklig_zur_Kollision.z );
			
		dJointSetAMotorAxis ( motorliste.back () , 2 , 2 , kraftraumvektor1.x , kraftraumvektor1.y , kraftraumvektor1.z );

		dJointSetAMotorParam ( motorliste.back () , dParamFMax , maxmotorkraft );
		
		atomInfo ();
		(*a2).atomInfo ();
		rueckgabewert = true;
	}
	
	(*a2).setBindungsblock ( (*a2).getBindungsblock () - 1 );
	
	return rueckgabewert;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
//bezieht sich auf das Atom was abgespalten wird
bool atomsimAtom::atomabspaltung ( atomsimAtom* a2 , int rekursionsZaehler )
{	
	bool rueckgabewert = false;
	
	if ( ursprung != NULL ) //die Ursprungsatome eines Roboters koennen also nicht abgespalten werden
	{
		dsPrint ( "Abspaltung von Roboter %i\n" , getRoboterID () );

		//rekursives Aufrufen der Methode atomabspaltung für alle Atome, in der Atomliste
		for ( unsigned int n = 0; n < atomliste.size(); n++ )
			(*atomliste.operator[](n) ).atomabspaltung ( a2 , 0 );
			
		//zerstören des Joints jedes Jointlistenelements
		for ( unsigned int n = 0; n < jointliste.size(); n++ )
			dJointDestroy ( jointliste.operator[](n) );
			
		//zerstören des Motors jedes Motorlistenelements
		for ( unsigned int n = 0; n < motorliste.size(); n++ )
			dJointDestroy ( motorliste.operator[](n) );

		//ACHTUNG!!!!!!!!!!! Kann eventuell aufgrund Doppelloeschung noch zu Problemen fuehren -> behoben
		if ( rekursionsZaehler == 1 )
		{
			dJointDestroy ( ursprungjoint );
			dJointDestroy ( ursprungmotor );
			//loeschen der einzellinks des Atoms und des Joints in den jewailigen Listen des Ursprungsatoms
			vector<atomsimAtom*>::iterator it1;
			for ( it1 = (*ursprung).atomliste.begin (); it1 <= (*ursprung).atomliste.end (); it1++ )
				if ( *it1 == this  ) (*ursprung).atomliste.erase ( it1 );

			vector<dJointID>::iterator it2;
			for ( it2 = (*ursprung).jointliste.begin (); it2 <= (*ursprung).jointliste.end (); it2++ )
				if ( *it2 == ursprungjoint  )
				{
					(*ursprung).jointliste.erase ( it2 );
				}

			vector<dJointID>::iterator it3;
			for ( it3 = (*ursprung).motorliste.begin (); it3 <= (*ursprung).motorliste.end (); it3++ )
				if ( *it3 == ursprungmotor  )
				{
					(*ursprung).motorliste.erase ( it3 );
				}
			
		}

		//NULL setzen jedes Joint- und Motorlistenelementes
		jointliste.clear ();
		motorliste.clear ();

		//NULL setzen jedes Atomlistenelementes
		atomliste.clear ();

		setRoboterID ( 0 );

		//setzt für dieses Teilchen eine Anzahl von ODE-Berechnungsschritten, welche es nicht mehr binden kann
		setBindungsblock ( bindungsblockdauer ); 
		
		atomInfo ();
		(*a2).atomInfo ();
		
		ursprung = NULL;
		
		rueckgabewert = true;

	}
	
	return rueckgabewert;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
int atomsimAtom::kollision ( atomsimAtom* a2 )
{
	int rueckgabewert = 0;

	raumvektor kollisionskraftvektor;
	double kollisionskraft = 0;
	
	kollisionskraftvektor.x = (*this).getKraftraumvektor().x + (*a2).getKraftraumvektor().x;
	kollisionskraftvektor.y = (*this).getKraftraumvektor().y + (*a2).getKraftraumvektor().y;
	kollisionskraftvektor.z = (*this).getKraftraumvektor().z + (*a2).getKraftraumvektor().z;
	
	kollisionskraft = sqrt ( pow ( kollisionskraftvektor.x , 2 ) + pow ( kollisionskraftvektor.y , 2 ) + pow ( kollisionskraftvektor.z , 2 ) );
	
	//dies erfolgt nur wenn zwei Einzelatomezusammenstoßen (spaeter soll es so werden)
	
	dsPrint ( "Stoss mit Kollisionskraft:%lf\n" , kollisionskraft );
	
	atomInfo ();
	(*a2).atomInfo ();
	
	if ( getBindungsstaerke () < kollisionskraft )
	{
			if ( /*(*ursprung).*/getAbspaltstaerke () < kollisionskraft )
			{
				if ( atomabspaltung ( a2 , 1 ) == true )
					rueckgabewert = 2;
			}
			else
			{
				if ( atombindung ( a2 , (*this).getKraftraumvektor() , (*a2).getKraftraumvektor() ) == true)
					rueckgabewert = 1;
			}
	}
	else dsPrint ( "Nichts geschieht, da Kollisionskraft unter Bindungsschwellwert\n" );/*sonst passiert nichts, nur normale ODE-Kollisionsbehandlung*/;
	
	return rueckgabewert;
}
	
/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::drawAtom ()
{
	//dsSetTexture (DS_WOOD);
	dsSetColor ( farbe.x , farbe.y , farbe.z );
	dsDrawSphere ( dBodyGetPosition ( body ) , dBodyGetRotation ( body ) , atomradius );
}

/**
 *
 *@author Marcel Kretschmann
 *@version
 **/
void atomsimAtom::setMotorWinkel ( int motornummer , double winkelgeschwindigkeit )
{
	dJointSetAMotorParam ( motorliste[motornummer] , dParamVel , winkelgeschwindigkeit*motorgeschwindigkeitsfaktor );
}
