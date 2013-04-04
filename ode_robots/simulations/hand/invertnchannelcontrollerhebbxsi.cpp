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
 *   Revision 1.1  2009-04-03 10:43:16  fhesse
 *   invertnchennelcontrollerhebb* moved to
 *   simulations (hand and nimm2_hebb)
 *
 *   Revision 1.2  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.1  2007/09/17 19:25:47  fhesse
 *   initial version
 *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include "invertnchannelcontrollerhebbxsi.h"
using namespace matrix;
using namespace std;

InvertNChannelControllerHebbXsi::InvertNChannelControllerHebbXsi(int _buffersize, bool _update_only_1/*=false*/, bool inactivate_hebb/*=false*/)
  : InvertNChannelController(_buffersize, _update_only_1 ){

  hebb_inactive=inactivate_hebb;
  if (hebb_inactive){
  std::cout<<"\nHebb learning inactive! (pure invertnchannelcontroller!)\n\n";
  }
};

void InvertNChannelControllerHebbXsi::init(int sensornumber, int motornumber, RandGen* randGen){
 InvertNChannelController::init(motornumber, motornumber, randGen );
 number_all_sensors=sensornumber;
 number_motors=motornumber;
 xsi_org.set(motornumber, 1);
 xsi_hebb.set(motornumber, 1);
 all_sensors = new sensor[sensornumber];

 //p.set(16, 1);
 int number_hebb_sensors = (sensornumber-motornumber); // number of context sensors not used by InvertNChannelController
 int number_hebb_weights = number_hebb_sensors * motornumber;
 p.set(number_hebb_weights, 1);



 /*
 for (int i=0; i<10; i++){
   old_sensors[i]=0;
 }
 */

 eps_hebb = 0.003; //0.01;
 fact_eps_h =1;

};


/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void InvertNChannelControllerHebbXsi::step(const sensor* x_, int number_sensors,
                                    motor* y_, int number_motors){

  sensor sensors[number_motors];
  for (int i=0; i<number_sensors; i++){
    if (i<number_motors){
      sensors[i]=x_[i];
    }
    all_sensors[i]=x_[i];
  }

  InvertNChannelController::step(sensors,number_motors,y_, number_motors);

};


/// performs one step without learning. Calulates motor commands from sensor inputs.
void InvertNChannelControllerHebbXsi::stepNoLearning(const sensor* x_, int number_sensors,
                                              motor* y_, int number_motors){
  sensor sensors[number_motors];
  for (int i=0; i<number_sensors; i++){
    if (i<number_motors){
      sensors[i]=x_[i];
    }
    all_sensors[i]=x_[i];
  }

  InvertNChannelController::stepNoLearning(sensors,number_motors,y_, number_motors);
};


Matrix InvertNChannelControllerHebbXsi::hebb(Matrix& xsi, sensor* sensors){


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
/*
  v.val(0,0) *= xsi.val(0,0);
  if (v.val(0,0)>1) v.val(0,0)=1;
  if (v.val(0,0)<-1) v.val(0,0)=-1;

  v.val(1,0) *= xsi.val(1,0);
  if (v.val(1,0)>1) v.val(1,0)=1;
  if (v.val(1,0)<-1) v.val(1,0)=-1;
*/

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

  /*
  for (int i=2;i<10;i++){
    old_sensors[i]=tmp_sensors[i];
  }
  */
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



double InvertNChannelControllerHebbXsi::calculateEHebb(const Matrix& x_delay,
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



/// learn values C,A as normal
/// learn h with additional hebb
void InvertNChannelControllerHebbXsi::learn(const Matrix& x_delay, const Matrix& y_delay){

  Matrix C_update(number_channels,number_channels);
  Matrix h_update(number_channels,1);

  double E_0 = calculateE(x_delay,  y_delay);
  double E_0_Hebb = calculateEHebb(x_delay,  y_delay);

    // calculate updates for h
  if (hebb_inactive){
    for (unsigned int i = 0; i < number_motors; i++){
      h.val(i,0) += delta;
      h_update.val(i,0) = -eps * fact_eps_h * (calculateE(x_delay, y_delay) - E_0) / delta;
      //h_update[i] = -2*eps *eita[i]*eita[i]*g(y_delay[i]);
      h.val(i,0) -= delta;
    }
  } else{
    for (unsigned int i = 0; i < number_motors; i++){
      h.val(i,0) += delta;
      h_update.val(i,0) = -eps * fact_eps_h * (calculateEHebb(x_delay, y_delay) - E_0_Hebb) / delta;
      //h_update[i] = -2*eps *eita[i]*eita[i]*g(y_delay[i]);
      h.val(i,0) -= delta;
    }
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
  // apply updates to h,C
  h += h_update.map(squash);
  C += C_update.map(squash);
};




list<Inspectable::iparamkey> InvertNChannelControllerHebbXsi::getInternalParamNames() const {
  list<iparamkey> keylist;
  keylist+=InvertNChannelController::getInternalParamNames();
  keylist+=storeMatrixFieldNames(xsi_org,"xsi_org");
  keylist+=storeMatrixFieldNames(xsi_hebb,"xsi_hebb");
  keylist+=storeMatrixFieldNames(p,"p");
  return keylist;
}

list<Inspectable::iparamval> InvertNChannelControllerHebbXsi::getInternalParams() const {
  list<iparamval> l;
  l+=InvertNChannelController::getInternalParams();
  l+=xsi_org.convertToList();
  l+=xsi_hebb.convertToList();
  //std::cout<<"getInternalParams("<<t<<")      xsi_hebb="<<xsi_hebb.val(0,0)<<"\n";
  l+=p.convertToList();
  return l;
}



