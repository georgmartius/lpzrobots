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
 *   Revision 1.1  2009-09-22 08:21:49  fhesse
 *   world is a schmitt trigger neuron
 *   only 1 DOF so far
 *
 *
 ***************************************************************************/
#include <assert.h>

#include "ode_robots/simulation.h"

#include "neuronworld.h"
using namespace std;

namespace lpzrobots {

  NeuronWorld::NeuronWorld(const OdeHandle& odeHandle, 
			     const OsgHandle& osgHandle, int sensornumber, int motornumber, const NeuronWorldConf& conf)
    : OdeRobot(odeHandle, osgHandle, "NeuronWorld", "$Id$"),conf(conf){

    assert(sensornumber == motornumber);

    sensorno = sensornumber; 
    motorno  = motornumber;  
    motors = (motor*)malloc(motorno * sizeof(motor));
    for(int i=0; i < motorno; i++){
      motors[i]=0.0;
    }
    dummy = new DummyPrimitive();
    dummy->init(odeHandle,0, osgHandle);

    a.set(sensornumber, 1);
    W.set(sensornumber, sensornumber);
    theta.set(sensornumber, 1);
    theta_const.set(sensornumber, 1);
    gamma.set(sensornumber, 1);

/*
  Initialisierung
  testhalber erstmal nur für ein Neuron!
*/
assert(sensornumber == 1);
gamma.val(0,0)=0;   // Dissipation
W.val(0,0)=1.2;   // Wheight Matrix
theta_const.val(0,0)=-1.0;//0.0;  // constant part of bias theta

  }


  NeuronWorld::~NeuronWorld(){
    if(motors) free(motors); 
  }

  /** sets actual motorcommands
      @param _motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  void NeuronWorld::setMotors(const motor* _motors, int motornumber){
    assert(motornumber == motorno);
    memcpy(motors, _motors, sizeof(motor) * motornumber);
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int NeuronWorld::getSensors(sensor* sensors, int sensornumber){
    assert(sensornumber == sensorno); 

    for (int i=0; i< motorno; i++){
       // many DOF:
      //theta.val(i,0)= theta_const.val(i,0) +motors[i]; 
      // singel DOF !!!
      assert(sensornumber == 1);
      theta.val(i,0)= conf.theta_const +motors[i]; 
    }

    // many DOF:
    //a=gamma*a + theta + W*a.map(g);

    // singel DOF !!!
    assert(sensornumber == 1);
    a.val(0,0)=((double)conf.gamma)*a.val(0,0) + theta.val(0,0) + ((double)conf.w)*g(a.val(0,0));
 
    int mini = min(sensorno,motorno); 
    for (int i=0; i< mini; i++){
      sensors[i]=a.val(i,0); // %motorno
    }
    return sensorno;
  };


  /** The list of all parameters with there value as allocated lists.
      @param keylist,vallist will be allocated with malloc (free it after use!)
      @return length of the lists
  */
  Configurable::paramlist NeuronWorld::getParamList() const{
    paramlist list;
    list += pair<paramkey, paramval> (string("theta_const"), conf.theta_const);
    list += pair<paramkey, paramval> (string("gamma"), conf.gamma);
    list += pair<paramkey, paramval> (string("w"),   conf.w);
    return list;
  }
  
  
  Configurable::paramval NeuronWorld::getParam(const paramkey& key) const{    
    if(key == "theta_const") return conf.theta_const; 
    else if(key == "gamma") return conf.gamma; 
    else if(key == "w") return conf.w; 
    else  return Configurable::getParam(key) ;
  }
  
  bool NeuronWorld::setParam(const paramkey& key, paramval val){    
    if(key == "theta_const") conf.theta_const = val; 
    else if(key == "gamma") conf.gamma = val; 
    else if(key == "w") conf.w = val; 
    else 
      return Configurable::setParam(key, val);    
    return true;
  }


}
