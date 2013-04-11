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
 *   Revision 1.5  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.4  2009/12/04 18:51:59  fhesse
 *   invertnchannelcontroller has bias (changeable in constructor) now
 *   neuronworld has linear neuron now (changeable in conf)
 *
 *   Revision 1.2  2009/12/01 13:35:50  fhesse
 *   minor changes
 *
 *   Revision 1.1  2009/09/22 08:21:49  fhesse
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
                             const OsgHandle& osgHandle, int sensornumber, int motornumber, const NeuronWorldConf& conf, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"),conf(conf){

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

    addParameter("theta_const", &conf.theta_const);
    addParameter("gamma", &conf.gamma);
    addParameter("w", &conf.w);

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

    if (conf.neuron_type == schmitt_trigger){
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
        sensors[i]=g(a.val(i,0)); // %motorno
      }
    }

    if (conf.neuron_type == linear){
      // singel DOF !!!
      assert(sensornumber == 1);
      a.val(0,0)= ((double)conf.w)*motors[0]  + theta.val(0,0);
      int mini = min(sensorno,motorno);
      for (int i=0; i< mini; i++){
        sensors[i]=a.val(i,0); // %motorno
      }
    }

    return sensorno;
  };


}
