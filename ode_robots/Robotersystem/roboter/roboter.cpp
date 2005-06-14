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


#define MAX_CONTROLERANZAHL 8


/**
 *Stanndartkonstruktor
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
virtual Roboter::Roboter ()
{

}

/**
 *Konstruktor
 *@param int Roboterkennummer
 *@param dWorldID Referenz auf die ODE-Simulationswelt, in der der Roboter angelegt werden soll
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
virtual Roboter::Roboter ( int startRoboterID , dWorldID welt , dSpaceID raum )
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
 *Destruktor
 *@author Marcel Kretschmann
 *@version alpha 1.0
 **/
virtual ~Roboter::Roboter()
{
}




/*************************************************************/
	
	/**
	 *Zeichnet die Koerper-GeometrieObjekte.
	 *@author Marcel Kretschmann
	 *@version alpha 1.0
	 **/
	virtual void draw()
	{
	
	}
	
	/// creates the robot at the given position and with the given color.
	virtual void create(double& x, double& y, double& z, Color& color)
	{
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
	virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2)
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
			if ( (*robotersammlung.back ()).kollisionsermittlung ( o1 , o2 ) == true )
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
						dJointID c = dJointCreateContact (welt,contactgroup,&contact[i]);
						dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
					}
			}
			return true; //wenn die Kollision durch diesen Roboter beahndelt wurde
		}
		else return false; //wenn die Kollision nicht durch diesen Roboter beahndelt wurde
	}
	
	/** returns actual sensorvalues
	@param sensors sensors scaled to [-1,1] 
	@param sensornumber length of the sensor array
	@return number of actually written sensors
	*/
	virtual int getSensors(sensor* sensors, int sensornumber)
	{
		for ( int n = 0; n < sensornumber; n++ )
			getWinkelDifferenz ( sensornumber , sensors++ );

		return getSensorfelGroesse (); //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden
	}
	
	/** sets actual motorcommands
	@param motors motors scaled to [-1,1] 
	@param motornumber length of the motor array
	*/
	virtual void setMotors(motor* motors, int motornumber)
	{
		for ( int n = 0; n < motornumber; n ++ )
			setMotorWinkel ( motornumber , *(motors++) );
	}
	
	
	/** returns number of sensors
	*/
	virtual int getSensorNumber()
	{
		return getSensorfeldGroesse ();
	}
	
	/** returns number of motors
	*/
	virtual int getMotorNumber()
	{
		return getMotorAnzahl ();
	}
	
	/** sets the position of robot to pos
	@param pos vector of desired position (x,y,z)
	*/
	virtual void setPosition(const dVector3& pos);
	
	/** returns position of robot 
	@param pos vector of desired position (x,y,z)
	*/
	virtual void getPosition(dVector3& pos);
	
	
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
virtual int Roboter::getObjektAnzahl ()
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
		dJointSetAMotorParam ( motorliste[motornummer] , dParamVel , winkelgeschwindigkeit );
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



