/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-08-05 23:25:57  martius
 *   adapted small things to compile with changed Plotoptions
 *
 *   Revision 1.1  2009/04/22 14:39:02  guettler
 *   moved layeredcontroller, layer2_incc and layer1_incc to ode_robots/simulations/nimm2_hebb and nimm2_layered
 *
 *   Revision 1.1  2009/04/20 07:28:32  robot12
 *   guettler: bugfix for: invertnchannelcontrollerhebb* moved to simulations (compile problems: moved layer2_incc.cpp+.h to hand and nimm2_hebb)
 *
 *   Revision 1.5  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.4  2007/10/10 19:00:26  fhesse
 *   testing
 *
 *   Revision 1.3  2007/10/10 08:44:27  fhesse
 *   testing
 *
 *   Revision 1.2  2007/10/09 16:47:13  fhesse
 *   further testing
 *
 *   Revision 1.1  2007/10/08 20:15:33  fhesse
 *   initial version of layered controller
 *
 *                           *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include "layer2_incc.h"
using namespace matrix;
using namespace std;

Layer2_INCC::Layer2_INCC(int _buffersize, bool _update_only_1/*=false*/)
  : InvertNChannelControllerHebbH(_buffersize, _update_only_1 ){

};
/*
Layer2_INCC::~Layer2_INCC(){

}
*/

void Layer2_INCC::init(int sensornumber, int motornumber, RandGen* randGen){
    InvertNChannelControllerHebbH::init(sensornumber, motornumber, randGen);
  //InvertNChannelControllerHebbH::init(motornumber, motornumber );

  L1_dh.set(motornumber,1);
};










/// same as in InvertNChannelControllerHebbH,
/// except that Hebb is learned here with dH from layer 1 of LayeredController
/// learn values C,A as normal
/// learn h with additional hebb
void Layer2_INCC::learn(const Matrix& x_delay, const Matrix& y_delay){

  Matrix C_update(number_channels,number_channels);
  // clear h_update
  for (int i=0; i<number_channels; i++){
    h_update.val(i,0)=0.0;
  }
  double E_0 = calculateE(x_delay,  y_delay);

  x_delay_=x_delay;


  //std::cout<<"x_delay 0,0="<<x_delay.val(0,0)<<std::endl;

  // calculate updates for h
  for (unsigned int i = 0; i < number_motors; i++){
    h.val(i,0) += delta;
    //    h_update.val(i,0) = -eps * fact_eps_h * (calculateE(x_delay, y_delay) - E_0) / delta;
    //h_update[i] = -2*eps *eita[i]*eita[i]*g(y_delay[i]);
    h.val(i,0) -= delta;
  }

  // only weights of one channel adapted in one time step
  unsigned int start=0;
  unsigned int end=number_channels;
  if(update_only_1) {
    start = t%number_channels;
    end = (t%number_channels) + 1;
  }
  for (unsigned int i = start; i < end; i++){
    for (unsigned int j = 0; j < number_motors; j++)
    // TEST!
    // Nur Diagonalelemente lernen!
    //if (i==j)
    {
      C.val(i,j) += delta;
      C_update.val(i,j)  = - eps *  (calculateE(x_delay, y_delay) - E_0) / delta ;
      C_update.val(i,j) -= damping_c*C.val(i,j) ;  // damping term
      C.val(i,j) -= delta;
      //A[i][j] += delta;
      //A_update[i][j] = -eps * (calculateE(x_delay, y_delay,eita) - E_0) / delta;
      //A[i][j] -= delta;
    }
  }
  // apply updates to C
  C += C_update.map(squash);

  // apply updates to h
  h += h_update.map(squash);
  //  std::cout<<"h_u 0,0"<<h_update.val(0,0)<<std::endl;

  //----------------------------------------------------------------------------------------------------------------------------

  if(!hebb_inactive){
    /////////////////////
    /**
     * learn dH
     */
    // learn hebb layer to predict dH (h_update.val)
    Matrix context_effective = calculateDelayedValues(context_buffer, int(s4delay));
    learnHebb(context_effective, L1_dh);
    /////////////////////

    /**
     * learn Xi
     * /
    // learn hebb layer to predict Xi
    xsi_org = x_buffer[t%buffersize] - A * y_delay;
    Matrix context_effective = calculateDelayedValues(context_buffer, int(s4delay));
    learnHebb(context_effective, xsi_org);
    /////////////////////
    */
    // predict dH (or Xi)
    h_pred_update=predictHebb(context_buffer[t%buffersize]);


    // add predicted dH
    if (use_hebb==1){ // only if use_hebb==1
      h += h_pred_update.map(squash);
    }

    /*
     * choose if and how to limit H
     */
    bool cutAt0_80=false;
    bool useTanhForH=true;

    if (cutAt0_80) {
      // h should not be larger than 0.8
      for (unsigned int i = 0; i < number_motors; i++){
        if (h.val(i,0)>0.8){
          h.val(i,0)=0.8;
        }
        if (h.val(i,0)<-0.8){
          h.val(i,0)=-0.8;
        }
      }
    }

    if (useTanhForH){
      for (unsigned int i = 0; i < number_motors; i++){
        h.val(i,0)=tanh(h.val(i,0));
      }
    }




//  does not work, condition never fulfilled for all sensors
//     // set h to 0 if no context sensor is active,
//     // but at least 1 context sensor was active in the previous time step
//     bool set_zero=true;
//     for (int i=0; i<number_context_sensors; i++){
//       if ( (context_buffer[(t-1)%buffersize].val(i,0)>0.15) && (context_buffer[(t)%buffersize].val(i,0)<0.15) ){
//         // previous contextsensorvalue was active, current one inactive -> set_zero should remain true
//         std::cout<<"reset_h "<<i<<std::endl;
//       } else {
//         set_zero=false;
//       }
//     }


    // set h to 0 if one context sensor is deactived,
    // even if other sensors stay active
    bool set_zero=false;
    for (int i=0; i<number_context_sensors; i++){
      if ( (context_buffer[(t-1)%buffersize].val(i,0)>0.15) && (context_buffer[(t)%buffersize].val(i,0)<0.15) ){
        // previous contextsensorvalue was active, current one inactive -> set_zero true
        set_zero=true;
      }
    }

    // only set H back if setHbackto0 is set
    if ( (setHbackto0==1) && (set_zero) ){
      for (unsigned int i = 0; i < number_motors; i++){
        h.val(i,0)=0.0;
      }
    }

  }

  //----------------------------------------------------------------------------------------------------------------------------
};







void Layer2_INCC::setL1_dH(Matrix tmp){
  L1_dh=tmp;
};
