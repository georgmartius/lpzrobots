/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   $Log$
 *   Revision 1.12.4.1  2005-12-12 22:36:28  martius
 *   indentation
 *
 *   Revision 1.12  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/
/************************************************************************/
/*schlangeforce.cpp								*/
/*Schlangenkonstrukt fuer das ODE-Robotersystem des Authors		*/
/*@author Marcel Kretschmann						*/
/*@version alpha 0.1							*/
/*									*/
/************************************************************************/

#include "schlangeforce.h"
//#include "configurable.h"

/**
 *constructor
 *@param startRoboterID ID, which should be managed clearly
 *@author Marcel Kretschmann
 *@version beta
 **/
SchlangeForce::SchlangeForce ( int startRoboterID , const OdeHandle& odeHandle, 
			       const SchlangenConf& conf ) 
  :Schlange ( startRoboterID , odeHandle, conf ) 
{
  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile$", 
			      "$Revision$");
  gamma=0.4;
}
	
/**
 *Destruktor
 *@author Marcel Kretschmann
 *@version beta
 **/
SchlangeForce::~SchlangeForce()
{
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
void SchlangeForce::doInternalStuff(const GlobalData& global){}
bool SchlangeForce::collisionCallback(void *data, dGeomID o1, dGeomID o2)
{
  //checks if one of the collision objects is part of the robot
  bool tmp_kollisionsbeteiligung = false;
  for ( int n = 0; n < getObjektAnzahl (); n++ )
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
      if ( SchlangeForce::kollisionsermittlung ( o1 , o2 ) == true )
	kollission = true;

      if ( kollission == false )
	{
	  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	  if (n > 0)
	    for (i=0; i<n; i++)
	      {
		contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
		  dContactSoftERP | dContactSoftCFM | dContactApprox1;
		contact[i].surface.mu = conf.frictionGround;
		contact[i].surface.slip1 = 0.005;
		contact[i].surface.slip2 = 0.005;
		contact[i].surface.soft_erp = 1;
		contact[i].surface.soft_cfm = 0.00001;

		dJointID c = dJointCreateContact ( world , contactgroup , &contact[i] );
		dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
	      }
	}
      return true; //if collision was threated by this robot
    }
  else return false; //if collision was not threated by this robot
}




/**
 *Reads the actual motor commands from an array, an sets all motors (forces) of the snake to this values.
 *It is an linear allocation.
 *@param motors pointer to the array, motor values are scaled to [-1,1] 
 *@param motornumber length of the motor array
 **/
void SchlangeForce::setMotors ( const motor* motors, int motornumber )
{

  // controller output as torques 
  for (int i=0; i<motornumber/2; i++){
    dJointAddUniversalTorques( jointliste[i],conf.factorForce* motors[2*i],
			       conf.factorForce*motors[2*i +1]);  // motorcommand
    dJointAddUniversalTorques( jointliste[i], -(gamma)*dJointGetUniversalAngle1Rate(jointliste[i]),
			       -(gamma) *dJointGetUniversalAngle2Rate(jointliste[i]) ); // friction
  }


  /*  // controller outputs as wheel velocity
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
  */
}	

/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the arrays

 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 *@author Marcel Kretschmann
 *@version beta
 **/
int SchlangeForce::getSensors ( sensor* sensors, int sensornumber )
{
  sensoraktualisierung ();
  for ( int n = 0; n < sensornumber; n++ )
    {
      if ( conf.ausgabeArt == angle )
	*sensors = sensorfeld[n].istwinkel/(2*M_PI);
      if ( conf.ausgabeArt == anglerate )
	getWinkelDifferenz ( n , sensors );
      *sensors *= conf.factorSensors;
      //		*sensors = 3*tanh((1/3) * (*sensors));  // keep sensorvalues in the range [-3,3]
      sensors++;
    }
	
  return getSensorfeldGroesse (); //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden
}



/** The list of all parameters with there value as allocated lists.
    @param keylist,vallist will be allocated with malloc (free it after use!)
    @return length of the lists
*/
Configurable::paramlist SchlangeForce::getParamList() const{
  paramlist list;
  list.push_back(pair<paramkey, paramval> (string("gamma"), gamma));
  list.push_back(pair<paramkey, paramval> (string("frictionGround"), conf.frictionGround));
  list.push_back(pair<paramkey, paramval> (string("factorForce"), conf.factorForce));
  list.push_back(pair<paramkey, paramval> (string("factorSensors"), conf.factorSensors));
  return list;
}


Configurable::paramval SchlangeForce::getParam(const paramkey& key) const{
  
  if(key == "gamma") return gamma; 
  else if(key == "frictionGround") return conf.frictionGround; 
  else if(key == "factorForce")    return conf.factorForce;  	
  else if(key == "factorSensors")  return conf.factorSensors;  	
  else return Configurable::getParam(key) ;
}

bool SchlangeForce::setParam(const paramkey& key, paramval val){
  
  if(key == "gamma") gamma=val;
  else if(key == "frictionGround") conf.frictionGround = val; 
  else if(key == "factorForce")    conf.factorForce    = val;
  else if(key == "factorSensors")  conf.factorSensors  = val;
  else return Configurable::setParam(key, val);
  return true;
}
