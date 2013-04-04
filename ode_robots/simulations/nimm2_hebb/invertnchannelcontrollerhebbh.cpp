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
 *   Revision 1.1  2009-04-03 10:43:42  fhesse
 *   invertnchennelcontrollerhebb* moved to
 *   simulations (hand and nimm2_hebb)
 *
 *   Revision 1.7  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.6  2007/12/07 09:08:16  fhesse
 *   warnings removed
 *
 *   Revision 1.5  2007/10/10 19:00:26  fhesse
 *   testing
 *
 *   Revision 1.4  2007/10/10 08:44:26  fhesse
 *   testing
 *
 *   Revision 1.3  2007/10/08 20:14:43  fhesse
 *   switch setHbackto0 added
 *   now different methods to limit
 *
 *   Revision 1.2  2007/10/05 19:51:10  fhesse
 *   use_hebb switch added
 *   setting H to zero when context switch off
 *
 *   Revision 1.1  2007/09/28 15:51:48  fhesse
 *   initial version
 *                           *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include "invertnchannelcontrollerhebbh.h"
using namespace matrix;
using namespace std;

InvertNChannelControllerHebbH::InvertNChannelControllerHebbH(int _buffersize, bool _update_only_1/*=false*/, bool inactivate_hebb/*=false*/)
  : InvertNChannelController(_buffersize, _update_only_1 ){

  hebb_inactive=inactivate_hebb;
  if (hebb_inactive){
    std::cout<<"\nHebb learning inactive! (pure invertnchannelcontroller!)\n\n";
  }

  context_buffer=0;

};

InvertNChannelControllerHebbH::~InvertNChannelControllerHebbH(){
  if(context_buffer) delete[] context_buffer;
}

void InvertNChannelControllerHebbH::init(int sensornumber, int motornumber, RandGen* randGen){
  InvertNChannelController::init(motornumber, motornumber, randGen );

  number_hom_sensors=motornumber;
  number_context_sensors= sensornumber-motornumber;
  number_motors=motornumber;

  //p.set(16, 1);
  //  int number_context_sensors = (sensornumber-motornumber); // number of context sensors (not used by InvertNChannelController)
  //int number_hebb_weights = number_context_sensors * motornumber;

  p.set(number_motors, number_context_sensors);
  h_update.set(number_channels,1);
  h_pred_update.set(number_motors,1);
  xsi_org.set(number_motors,1);

  x_delay_.set(number_motors,1);
  lay2_nchan_sens.set(number_motors,1);

  std::cout<<"number_context_sensors="<<number_context_sensors<<std::endl;

  context_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    context_buffer[k].set(number_context_sensors,1);
  }

  eps_hebb = 0.003; //0.01;
  use_hebb=1;
  setHbackto0=1;
  fact_eps_h =1;

};


/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void InvertNChannelControllerHebbH::step(const sensor* x_, int number_sensors,
                                    motor* y_, int number_motors){
  sensor sensors[number_motors];
  sensor context_sensors[number_context_sensors];
  for (int i=0; i<number_sensors; i++){
    if (i<number_motors){
      sensors[i]=x_[i];
    } else {
      context_sensors[i-number_motors]=x_[i];
    }
  }
  lay2_nchan_sens=Matrix(number_motors,1,sensors);

  Matrix c_sensors(number_context_sensors,1,context_sensors);
  // put new input vector in ring buffer x_buffer
  putInBuffer(context_buffer,c_sensors );

  InvertNChannelController::step(sensors,number_motors,y_, number_motors);



};


/*
/// performs one step without learning. Calulates motor commands from sensor inputs.
void InvertNChannelControllerHebbH::stepNoLearning(const sensor* x_, int number_sensors,
                                              motor* y_, int number_motors){
  sensor sensors[number_motors];
  sensor context_sensors[number_context_sensors];

  for (int i=0; i<number_sensors; i++){
    if (i<number_motors){
      sensors[i]=x_[i];
    } else {
     context_sensors[i-number_motors]=x_[i];
    }
  }

  InvertNChannelController::stepNoLearning(sensors,number_motors,y_, number_motors);
};
*/

/*
Matrix InvertNChannelControllerHebbH::hebb(Matrix& xsi, sensor* sensors){


  //sensor tmp_sensors[10];

  for (int i=number_motors;i<number_all_sensors;i++){
  //for (int i=2;i<10;i++){
    if (sensors[i]<0.15) {
      sensors[i]=0; // IR's should only have positive values
    }
    //tmp_sensors[i]=sensors[i];
    //if (old_sensors[i]>sensors[i]){
    //  sensors[i]=0; // only increase of sensors "counts"
    //  //std::cout<<"sensor "<<i<<" decreasing "<<"x["<<i<<"]="<<sensors[i]<<" x_old["<<i<<"]="<<old_sensors[i]<<"\n";
    //}
  }

  Matrix v=xsi; // v consists of original xsi^2 plus xsi_hebb^2 (before return make sqrt!)
  //v.val(0,0)=fabs(xsi.val(0,0));
  //v.val(1,0)=fabs(xsi.val(1,0));
/ *
  v.val(0,0) *= xsi.val(0,0);
  if (v.val(0,0)>1) v.val(0,0)=1;
  if (v.val(0,0)<-1) v.val(0,0)=-1;

  v.val(1,0) *= xsi.val(1,0);
  if (v.val(1,0)>1) v.val(1,0)=1;
  if (v.val(1,0)<-1) v.val(1,0)=-1;
* /

  for (int i=number_motors;i<number_all_sensors;i++){
  //for (int i=2; i<10; i++){
    //double dp=  eps_hebb* v.val(0,0) * sensors[i] - p.val(i-2,0)*p.val(i-2,0);
    double dp=  eps_hebb* v.val(0,0) * sensors[i]*(1 - pow(p.val(i-2,0),2));
    p.val(i-2,0)+=dp;
  }
  for (int i=number_motors;i<number_all_sensors;i++){
  //for (int i=2; i<10; i++){
    // double dp=  eps_hebb* v.val(1,0) * sensors[i] - p.val(i+6,0)*p.val(i+6,0);
    double dp=  eps_hebb* v.val(1,0) * sensors[i]*(1 - pow(p.val(i+6,0),2));
    p.val(i+6,0)+=dp;
    //std::cout<<p.val(i+6,0)*p.val(i+6,0)<<"  ";
  }
  //std::cout<<"\n  ";

  / *
  for (int i=2;i<10;i++){
    old_sensors[i]=tmp_sensors[i];
  }
  * /
  for (int i=number_motors;i<number_all_sensors;i++){
  //for (int i=2; i<10; i++){
    v.val(0,0)+= p.val(i-2,0) *  sensors[i];
  }
  for (int i=number_motors;i<number_all_sensors;i++){
  //for (int i=2; i<10; i++){
    v.val(1,0)+= p.val(i+6,0) * sensors[i];
  }

  return v;
};
*/

/*
double InvertNChannelControllerHebbH::calculateE_(const Matrix& x_delay,
                                            const Matrix& y_delay){
  // Calculate z based on the delayed inputs since the present input x is
  // produced by the outputs tau time steps before
  // which on their hand are y = K(x_D)
  // due to the delay in the feed back loop.
  Matrix z = C * x_delay + h;

  xsi_org = x_buffer[t%buffersize] - A * y_delay;
  //Matrix xsi = x_buffer[t%buffersize] - A * z.map(g);

  xsi_hebb = hebb( xsi_org, all_sensors );
  //xsi_hebb = hebb_if( xsi_org, all_sensors );
  //if (fabs(xsi_hebb.val(0,0))>0.2)
  //  std::cout<<"calcE("<<t<<")  xsi_org="<<xsi_org.val(0,0)<<"    xsi_hebb="<<xsi_hebb.val(0,0)<<"\n";


  Matrix Cg = C.multrowwise(z.map(g_s)); // Cg_{ij} = g'_i * C_{ij}
  L = A*Cg;                   // L_{ij}  = \sum_k A_{ik} g'_k c_{kj}

  Matrix v = (L^-1)*xsi_hebb;

  double E = ((v^T)*v).val(0, 0);
  double Es = 0.0;
  if(desens!=0){
    Matrix diff_x = x_buffer[t%buffersize] - A*( (C*x_buffer[t%buffersize]+h).map(g) );
    Es = ((diff_x^T)*diff_x).val(0, 0);
  }
  return (1-desens)*E + desens*Es;
};
  */



/**
 * learn hebb layer to predict dH (h_update.val)
 * @param context_sensors input tu hebbian layer
 * @param h_update desired outputother input of hebbian layer
 */

void InvertNChannelControllerHebbH::learnHebb(const matrix::Matrix& context_sensors, const matrix::Matrix& h_update){

  // preprocess context sensors
  Matrix c_sensors = context_sensors;
  for (int i=0;i<number_context_sensors;i++){
   if (c_sensors.val(i,0)<0.15) {
      c_sensors.val(i,0)=0; // IR's should only have positive values
    }
  }

  // adapt hebbian weights
  for (uint i=0; i<number_motors; i++){
    for (uint j=0; j<(uint)number_context_sensors; j++){
      if (i==j){ // TODO: remove (it is just for testing)
      double dp=  eps_hebb* h_update.val(i,0) * c_sensors.val(j,0) *(1 - pow(p.val(i,j),2));
      //      std::cout<<eps_hebb<<"*"<<h_update.val(i,0)<<" * "<<c_sensors.val(j,0)<<std::endl;
      p.val(i,j)+=dp;
      }
    }
  }
  /*
  // remove this !!! (just a test)
  for (int i=0; i<number_motors; i++){
    for (int j=0; j<number_context_sensors; j++){
      if ((j==0) || (j==1)){
        p.val(i,j)=-0.1;
      } else {
        p.val(i,j)=0.1;
      }
    }
  }
  */

}

/**
 * predict the update of h based on the actual context sensors
 * @param context_sensors prediction is based on these sensors
 */
matrix::Matrix InvertNChannelControllerHebbH::predictHebb(const matrix::Matrix& context_sensors){
  // preprocess context sensors
  Matrix c_sensors = context_sensors;
  for (int i=0;i<number_context_sensors;i++){
   if (c_sensors.val(i,0)<0.15) {
      c_sensors.val(i,0)=0; // IR's should only have positive values
    }
  }


  Matrix pred_h_update(number_motors,1) ;
  for (unsigned int k = 0; k < number_motors; k++) {
    pred_h_update.val(k,0)=0;
  }

  for (uint i=0; i<number_motors; i++){
    for (uint j=0; j<(uint)number_context_sensors; j++){
      pred_h_update.val(i,0)+= p.val(i,j) *  context_sensors.val(j,0);
    }
  }


  return pred_h_update;

}


/// learn values C,A as normal
/// learn h with additional hebb
void InvertNChannelControllerHebbH::learn(const Matrix& x_delay, const Matrix& y_delay){

  Matrix C_update(number_channels,number_channels);
  // clear h_update
  for (int i=0; i<number_channels; i++){
    h_update.val(i,0)=0.0;
  }

  double E_0 = calculateE(x_delay,  y_delay);


  // calculate updates for h
  for (unsigned int i = 0; i < number_motors; i++){
    h.val(i,0) += delta;
    h_update.val(i,0) = -eps * fact_eps_h * (calculateE(x_delay, y_delay) - E_0) / delta;
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
    if (i==j)
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

  if(!hebb_inactive){
    /////////////////////
    /**
     * learn dH
     * /
    // learn hebb layer to predict dH (h_update.val)
    Matrix context_effective = calculateDelayedValues(context_buffer, int(s4delay));
    learnHebb(context_effective, h_update);
    /////////////////////
    */
    /**
     * learn Xi
     */
    // learn hebb layer to predict Xi
    xsi_org = x_buffer[t%buffersize] - A * y_delay;
    Matrix context_effective = calculateDelayedValues(context_buffer, int(s4delay));
    learnHebb(context_effective, xsi_org);
    /////////////////////

    // predict dH (or Xi)
    h_pred_update=predictHebb(context_buffer[t%buffersize]);

    /*
    for (unsigned int i = 0; i < number_motors; i++){
      h_pred_update.val(i,0)=tanh(h_pred_update.val(i,0));
    }
    */

    // add predicted dH
    if (use_hebb==1){ // only if use_hebb==1
      //h += h_pred_update.map(squash);
      h -= h_pred_update.map(squash);
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

};




list<Inspectable::iparamkey> InvertNChannelControllerHebbH::getInternalParamNames() const {
  list<iparamkey> keylist;
  keylist+=InvertNChannelController::getInternalParamNames();
  keylist+=storeMatrixFieldNames(h_update,"h_u");
  keylist+=storeMatrixFieldNames(h_pred_update,"h_pu");
  keylist+=storeMatrixFieldNames(xsi_org,"xsi_org");
  //  keylist+=storeMatrixFieldNames(xsi_hebb,"xsi_hebb");
  keylist+=storeMatrixFieldNames(p,"p");
  keylist+=storeMatrixFieldNames(x_delay_,"x_delay");
  keylist+=storeMatrixFieldNames(lay2_nchan_sens,"lay2_nchan_sens");
  return keylist;
}

list<Inspectable::iparamval> InvertNChannelControllerHebbH::getInternalParams() const {
  list<iparamval> l;
  l+=InvertNChannelController::getInternalParams();
  l+=h_update.convertToList();
  l+=h_pred_update.convertToList();
  l+=xsi_org.convertToList();
  //  l+=xsi_hebb.convertToList();
  l+=p.convertToList();
  l+=x_delay_.convertToList();
  l+=lay2_nchan_sens.convertToList();
  return l;
}



