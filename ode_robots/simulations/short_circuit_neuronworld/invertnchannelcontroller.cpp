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
 *   Revision 1.4  2009-12-04 18:51:59  fhesse
 *   invertnchannelcontroller has bias (changeable in constructor) now
 *   neuronworld has linear neuron now (changeable in conf)
 *
 *   Revision 1.1  2009/09/22 08:21:49  fhesse
 *   world is a schmitt trigger neuron
 *   only 1 DOF so far
 *
 *   Revision 1.7  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.6  2007/03/26 13:13:47  martius
 *   store and restore with params
 *
 *   Revision 1.5  2006/12/11 18:14:14  martius
 *   delete for BNoise
 *   delete of buffers
 *
 *   Revision 1.4  2006/07/20 17:14:35  martius
 *   removed std namespace from matrix.h
 *   storable interface
 *   abstract model and invertablemodel as superclasses for networks
 *
 *   Revision 1.3  2006/07/14 12:23:58  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.5  2006/07/10 13:05:16  martius
 *   NON-COMMERICAL LICENSE added to controllers
 *
 *   Revision 1.1.2.4  2006/07/10 11:59:24  martius
 *   Matrixlib now in selforg
 *   no namespace std in header files
 *
 *   Revision 1.1.2.3  2006/06/25 21:56:07  martius
 *   configureable has name and revision
 *
 *   Revision 1.1.2.2  2006/01/18 16:48:35  martius
 *   stored and restore
 *
 *   Revision 1.1.2.1  2005/11/14 17:37:29  martius
 *   moved to selforg
 *
 *   Revision 1.15  2005/10/27 15:46:38  martius
 *   inspectable interface is expanded to structural information for network visualiser
 *
 *   Revision 1.14  2005/10/27 15:02:06  fhesse
 *   commercial use added
 *                                  *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include "invertnchannelcontroller.h"
using namespace matrix;
using namespace std;

InvertNChannelController::InvertNChannelController(int _buffersize, bool _update_only_1/*=false*/, ModelNeuronProperties _model_type/*=nobias*/)
  : InvertController("InvertNChannelController", "$Id$"){
  t=0;
  update_only_1 = _update_only_1;
  buffersize    = _buffersize;
  x_buffer=0;
  y_buffer=0;
  model_type =_model_type;

};

InvertNChannelController::~InvertNChannelController(){
  if(x_buffer) delete[] x_buffer;
  if(y_buffer) delete[] y_buffer;
}

void InvertNChannelController::init(int sensornumber, int motornumber, RandGen* randGen){
  assert(sensornumber == motornumber);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak

  number_channels=sensornumber;
  A.set(number_channels, number_channels);
  s.set(number_channels, 1);
  C.set(number_channels, number_channels);
  h.set(number_channels, 1);
  L.set(number_channels, number_channels);

  A.toId(); // set a to identity matrix;
  C.toId(); // set a to identity matrix;
  C*=0.1;
  A*=0.1;
  x_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_channels,1);
    y_buffer[k].set(number_channels,1);
  }
}

/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void InvertNChannelController::step(const sensor* x_, int number_sensors,
                                    motor* y_, int number_motors){
  stepNoLearning(x_, number_sensors, y_, number_motors);
  if(t<=buffersize) return;
  t--;

  // calculate effective input/output, which is (actual-steps4delay) element of buffer
  Matrix x_effective = calculateDelayedValues(x_buffer, int(s4delay));
  Matrix y_effective = calculateDelayedValues(y_buffer, int(s4delay));

  // learn controller with effective input/output
  learn(x_effective, y_effective);
  learnmodel(y_effective);

  // update step counter
  t++;
};


/// performs one step without learning. Calulates motor commands from sensor inputs.
void InvertNChannelController::stepNoLearning(const sensor* x_, int number_sensors,
                                              motor* y_, int number_motors){
  assert((unsigned)number_sensors <= number_channels
         && (unsigned)number_motors <= number_channels);
  Matrix x(number_channels,1,x_);

  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);

  // averaging over the last s4avg values of x_buffer
  Matrix x_smooth = calculateSmoothValues(x_buffer, int(s4avg));

  // calculate controller values based on smoothed input values
  Matrix y = calculateControllerValues(x_smooth);

  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);

  // convert y to motor*
  y.convertToBuffer(y_, number_motors);

  // update step counter
  t++;
};


// void iteration(double *colmn,double dommy[number_channels][number_channels],double *improvment){
//   double sum[number_channels]  ;
//   double norm=0.0 ;


//   for (int k= 0; k< number_channels; k++)
//     {
//       norm+=colmn[k]*colmn[k]  ;
//     }
//   norm=sqrt(norm)   ;
//   for(int t = 0; t <number_it ; t++)
//     {

//       //initialization
//       if(t==0)
//         {

//           for (int i = 0; i < number_channels; i++)
//             {
//               improvment[i]=0.0 ;

//             }
//         }


//       for (int k= 0; k< number_channels; k++)
//         {
//           sum[k]=colmn[k]/norm ;

//           for (int l= 0; l<number_channels; l++)
//             {
//               sum[k]-=dommy[k][l]*improvment[l]  ;
//             }
//         }


//       for (int j = 0; j< number_channels; j++){
//         for (int i = 0; i < number_channels; i++){
//           improvment[j]+=epsilon_it*dommy[i][j]*sum[i]       ;
//         }
//       }

//     }//endof-t-loop


//   for (int j = 0; j< number_channels; j++){
//     improvment[j]*=norm  ;
//   }
// };


double InvertNChannelController::calculateE(const Matrix& x_delay,
                                            const Matrix& y_delay){
  // Calculate z based on the delayed inputs since the present input x is
  // produced by the outputs tau time steps before
  // which on their hand are y = K(x_D)
  // due to the delay in the feed back loop.
  Matrix z = C * x_delay + h;

  Matrix xsi;
  if (model_type== nobias){//no bias used in model
     xsi = x_buffer[t%buffersize] - A * y_delay;
  }
  if (model_type== bias){// model bias s is used
    xsi = x_buffer[t%buffersize] - A * y_delay + s;
  }

  //Matrix xsi = x_buffer[t%buffersize] - A * z.map(g);

  Matrix Cg = C.multrowwise(z.map(g_s)); // Cg_{ij} = g'_i * C_{ij}
  L = A*Cg;                   // L_{ij}  = \sum_k A_{ik} g'_k c_{kj}
////////
// if model bias s is used -> change L ?
//  L=dx(t+1)/dx(t)=d ag(cx+h) +s / dx(t)
//  d.h. s faellt bei Ableitung weg !
//  d.h. es aendert sich nichts
////////



  Matrix v = (L^-1)*xsi;

  double E = ((v^T)*v).val(0, 0);
  double Es = 0.0;
  if(desens!=0){
    if (model_type== nobias){//no bias used in model
      Matrix diff_x = x_buffer[t%buffersize] - A*( (C*x_buffer[t%buffersize]+h).map(g) );
      Es = ((diff_x^T)*diff_x).val(0, 0);
    }
  if (model_type== bias){// bias used in model
      Matrix diff_x = x_buffer[t%buffersize] - (A*( (C*x_buffer[t%buffersize]+h).map(g) ) + s);
      Es = ((diff_x^T)*diff_x).val(0, 0);
    }

  }
  return (1-desens)*E + desens*Es;

//   iteration(xsi,A,eita_zero)  ;
//   for (int i = 0; i < number_channels; i++)
//     {
//       eita[i]=(1/(g_s(z[i])))*eita_zero[i];
//     }

//   iteration(eita,C,shift_value) ;
//   double E=0.0  ;
//   for (int i=0;i<number_channels;i++)
//     {
//       E+=shift_value[i]*shift_value[i];
//     }
//   for (int i = 0; i < number_channels; i++)
//     {
//       eita_sup[i]=eita[i];
//     }


//   // Berechnung des z mit aktuellem Sensorwert
//   for (int i = 0; i < number_channels; i++)
//     {
//       z[i] = h[i];
//       for (int j = 0; j < number_channels; j++)
//         {
//           z[i] += C[i][j] *x_buffer[(t+buffersize)%buffersize][j];
//         }
//       //y[i] = g(z[i]);
//     }

//   double E_s=0;
//   for (int i = 0; i < number_channels; i++)
//     {
//       for (int j = 0; j < number_channels; j++)
//         {
//           E_s += (A[i][j]*g(z[j]) - x_buffer[(t+buffersize)%buffersize][i]) * (A[i][j]*g(z[j]) - x_buffer[(t+buffersize)%buffersize][i]);
//         }
//     }


//   E=(1-m)*E+ m*E_s;

//   return E;
};


/// learn values h,C,A
void InvertNChannelController::learn(const Matrix& x_delay, const Matrix& y_delay){

  Matrix C_update(number_channels,number_channels);
  Matrix h_update(number_channels,1);

  double E_0 = calculateE(x_delay,  y_delay);

  // calculate updates for h,C,A
  for (unsigned int i = 0; i < number_channels; i++)
  {
      h.val(i,0) += delta;
      h_update.val(i,0) = -eps * (calculateE(x_delay, y_delay) - E_0) / delta;
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
      for (unsigned int j = 0; j < number_channels; j++)
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

// normal delta rule
void InvertNChannelController::learnmodel(const Matrix& y_delay){
  if (model_type== nobias){ // if no model bias is used
    Matrix xsi = x_buffer[t%buffersize] -  A * y_delay;
    A += (( xsi*(y_delay^T) ) * eps * factor_a).map(squash);
  }
  if (model_type== bias){//if model bias s is used
    Matrix xsi = x_buffer[t%buffersize] -  (A * y_delay + s);
    A += (( xsi*(y_delay^T) ) * eps * factor_a).map(squash);
    s += (xsi* eps * factor_a).map(squash);
  }
};

/// calculate delayed values
Matrix InvertNChannelController::calculateDelayedValues(const Matrix* buffer,
                                                       unsigned int number_steps_of_delay_){
  // number_steps_of_delay must not be smaller than buffersize
  assert (number_steps_of_delay_ < buffersize);
  return buffer[(t - number_steps_of_delay_ + buffersize) % buffersize];
};

Matrix InvertNChannelController::calculateSmoothValues(const Matrix* buffer,
                                                       unsigned int number_steps_for_averaging_){
  // number_steps_for_averaging_ must not be larger than buffersize
  assert (number_steps_for_averaging_ <= buffersize);

  Matrix result(number_channels,1); // initialised with 0
  for (unsigned int k = 0; k < number_steps_for_averaging_; k++) {
    result += buffer[(t - k + buffersize) % buffersize];
  }
  result *= 1/((double) (number_steps_for_averaging_)); // scalar multiplication
  return result;
};


/// calculate controller outputs
/// @param x_smooth Matrix(number_channels,1)
Matrix InvertNChannelController::calculateControllerValues(const Matrix& x_smooth){
  return (C*x_smooth+h).map(g);
};


// put new value in ring buffer
void InvertNChannelController::putInBuffer(Matrix* buffer, const Matrix& vec){
  buffer[t%buffersize] = vec;
}

/** stores the controller values to a given file. */
bool InvertNChannelController::store(FILE* f) const{
  // save matrix values
  C.store(f);
  h.store(f);
  A.store(f);
  if (model_type== bias) s.store(f);
  Configurable::print(f,0);
  return true;
}

/** loads the controller values from a given file. */
bool InvertNChannelController::restore(FILE* f){
  // save matrix values
  C.restore(f);
  h.restore(f);
  A.restore(f);
  if (model_type== bias) s.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}


list<Inspectable::iparamkey> InvertNChannelController::getInternalParamNames() const {
  list<iparamkey> keylist;
  keylist+=store4x4AndDiagonalFieldNames(A,"A");
  if (model_type== bias) keylist+=storeMatrixFieldNames(s,"s");
  keylist+=store4x4AndDiagonalFieldNames(C,"C");
  keylist+=storeMatrixFieldNames(h,"h");
  return keylist;
}

list<Inspectable::iparamval> InvertNChannelController::getInternalParams() const {
  list<iparamval> l;
  l+=store4x4AndDiagonal(A);
  if (model_type== bias) l+=s.convertToList();
  l+=store4x4AndDiagonal(C);
  l+=h.convertToList();
  return l;
}

list<Inspectable::ILayer> InvertNChannelController::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  l+=ILayer("x",  "",  number_channels, 0, "Sensors");
  l+=ILayer("y",  "H", number_channels, 1, "Motors");
  l+=ILayer("xP", "",  number_channels, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> InvertNChannelController::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  return l;
}







