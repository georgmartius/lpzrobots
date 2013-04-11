/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
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
 ***************************************************************************/

#include "invertmotorbigmodel.h"
#include "regularisation.h"

using namespace matrix;
using namespace std;

InvertMotorBigModel::InvertMotorBigModel( const InvertMotorBigModelConf& conf)
  : InvertMotorController(conf.buffersize, "InvertMotorBigModel", "$Id$"), conf(conf) {

  assert(conf.model != NULL);
  addConfigurable(conf.model);
  addParameterDef("inhibition",&inhibition,0);
  addParameterDef("kwta",&kwta,2);
  addParameterDef("limitrf",&limitRF,0);
  addParameterDef("dampS",&dampS,0);
  addParameterDef("modelcompl",&(this->conf.modelCompliant),0);

  managementInterval=10;
  useTeaching=false;
  BNoiseGen = 0;
};


InvertMotorBigModel::~InvertMotorBigModel(){
  if(x_buffer && y_buffer && eta_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] eta_buffer;
  }

  if(BNoiseGen) delete BNoiseGen;

}


void InvertMotorBigModel::init(int sensornumber, int motornumber, RandGen* randGen){
  assert(sensornumber>=motornumber);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak

  number_motors  = motornumber;
  number_sensors = sensornumber;
  assert(number_motors && number_sensors);

  conf.model->init(number_motors, number_sensors, conf.modelInit);
  A = conf.model->response(Matrix(number_motors,1));

  if (conf.useS) S.set(number_sensors, number_sensors*2); // S gets first and second derivative
  C.set(number_motors,  number_sensors);
  H.set(number_motors,  1);
  R.set(number_motors, number_motors);
  SmallID.set(number_motors, number_motors);
  SmallID.toId();
  SmallID *= 0.001;

  xsi.set(number_sensors,1);
  xsi_norm=0;
  xsi_norm_avg=0.2;
  pain=0;

  BNoiseGen = new WhiteUniformNoise();
  BNoiseGen->init(number_sensors);

  A.toId(); // set A to identity matrix;
  if (conf.useS) S.mapP(randGen, random_minusone_to_one)*0.01; // set S to small random matrix;

  // initialise the C matrix with identity + noise (-conf.cNonDiag, conf.cNonDiag) scaled to cInit value
  C = ((C^0) + C.mapP(randGen, random_minusone_to_one) * conf.cNonDiag) * conf.cInit;

  x_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  eta_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
    eta_buffer[k].set(number_motors,1);
  }
  y_teaching.set(number_motors, 1);

  zero_eta.set(number_motors, 1);

  t_rand = int(randGen->rand()*10);
  initialised = true;
}


/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void InvertMotorBigModel::step(const sensor* x_, int number_sensors,
                            motor* y_, int number_motors){
  fillBuffersAndControl(x_, number_sensors, y_, number_motors);
  if(t>buffersize){
    int delay = max(int(s4delay)-1,0);
    // learn Model with actual sensors and with effective motors,
    // calc xsi and A;
    learnModel(delay);
    calcEtaAndBufferIt(delay);
    // learn controller with effective input/output
    learnController();

  }
  // update step counter
  t++;
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void InvertMotorBigModel::stepNoLearning(const sensor* x, int number_sensors,
                                            motor*  y, int number_motors ){
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
};

void InvertMotorBigModel::fillBuffersAndControl(const sensor* x_, int number_sensors,
                                              motor* y_, int number_motors){
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);

  Matrix x(number_sensors,1,x_);

  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);

  // averaging over the last s4avg values of x_buffer
  x_smooth = calculateSmoothValues(x_buffer, t < s4avg ? 1 : int(max(1.0,s4avg)));

  // calculate controller values based on smoothed input values
  Matrix y = calculateControllerValues(x_smooth);

  // from time to time call management function. For example damping and inhibition is done.
  if((t+t_rand)%managementInterval==0) management();

  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);

  // convert y to motor*
  y.convertToBuffer(y_, number_motors);
}


double regularizedInverse_bigmodel(double v){
  return 1/(fabs(v)+0.1);
}

/// calculates Eta which corrensponds to xsi_t which means eta_{t-1}
//  @param delay 0 for no delay and n>0 for n timesteps delay in the SML
void InvertMotorBigModel::calcEtaAndBufferIt(int delay){
  // eta = A^-1 xsi (first shift in motor-space at current time)
  //
  //  we use pseudoinverse U=A^T A -> eta = U^-1 A^T xsi // TODO add 0.01*I
  Matrix eta = (A.multTM()^-1) * ( (A^T) * xsi );
  if(relativeE != 0) { // divide eta by |y|  == relative error
    const Matrix& y = y_buffer[t % buffersize];
    eta = eta.multrowwise(y.map(regularizedInverse_bigmodel));
  }
  eta_buffer[(t-1)%buffersize] = eta; // Todo: work with the reference
}


/// learn values H,C
// This is the implementation uses a better formula for g^-1 using Mittelwertsatz
void InvertMotorBigModel::learnController(){

  Matrix C_update(C.getM(), C.getN());
  Matrix H_update(H.getM(), H.getN());
  calcCandHUpdates(C_update, H_update, 0);

  if(adaptRate>0){ // adapt learning rate
    double norm = matrixNorm1(C); // + matrixNorm1(H);
    double update_norm = matrixNorm1(C_update) + matrixNorm1(H_update)*0.05;

    epsC = adapt(epsC, update_norm, norm*nomUpdate, adaptRate, adaptRate*10);
    epsC = min( 5.0, epsC);
  }
  updateCandH(C_update, H_update, squashSize);
};

/// calculates the Update for C and H
// @param y_delay timesteps to delay the y-values.  (usually 0)
//  Please note that the delayed values are NOT used for the error calculation
//  (this is done in calcXsi())
void InvertMotorBigModel::calcCandHUpdates(Matrix& C_update, Matrix& H_update, int y_delay){
  assert( steps + y_delay < buffersize);
  // Matrix& eta = zero_eta;
  // Matrix v_old = (eta_buffer[t%buffersize]).map(g);
  bool teaching = (conf.modelCompliant!=0) || useTeaching;
  Matrix C_updateTeaching;
  Matrix H_updateTeaching;
  if(teaching){
    C_updateTeaching.set(C.getM(), C.getN());
    H_updateTeaching.set(H.getM(), H.getN());
  }
  Matrix v = zero_eta;
  for(unsigned int s = 1; s <= steps; s++){
    const Matrix& eta = eta_buffer[(t-s)%buffersize].map(g);

    //const Matrix& x      = A * z.map(g) + xsi;
    //    z                    = R * z.map(g) + H + C*xsi; // z is a dynamic itself (not successful)
    const Matrix& x          = x_buffer[(t-s-y_delay)%buffersize];
    const Matrix& z          = C * x + H;
    const Matrix shift       = (eta + v).map(g); // limit shift with g (arbitrary choice)
    const Matrix g_prime     = Matrix::map2(g_s, z, shift);
    const Matrix g_prime_inv = g_prime.map(one_over);
    const Matrix g_2p_div_p  = Matrix::map2(g_ss_div_s, z, shift);

    const Matrix zeta = shift.multrowwise(g_prime_inv); // G'(Z)^-1 * (eta+v)
    R                 = C * A;
    const Matrix chi  = ((R.multMT()+SmallID)^-1) * zeta;
    v                 = (R^T) * chi;
    const Matrix rho  = g_2p_div_p.multrowwise(chi.multrowwise(zeta)) * -1;

    // calculate updates of H,C
    // delta C += eps * (zeta * v^T * A^T + beta * x^T)
    //  q_prime_inv.elementProduct is the correction of negled terms
    //   (LL^T)^-1 from learning rule, which drive the system out of saturation. (important)
    // double q_prime_invers = q_prime_inv.elementProduct();
    C_update += ( chi*(v^T)*(A^T) - rho*(x^T) ) * epsC;
    H_update += rho * -epsC;

    if(conf.modelCompliant!=0 && s==1){  // learning of the forward task
      // eta is difference between last y and reconstructed one -> used as forward error signal
      // The question is wether to use eta (linearised), zeta (neuron inverse) or eta*g' (Backprop) !
      const Matrix& g_eta = eta.multrowwise(g_prime);
      C_updateTeaching += ( g_eta*(x^T) ) * conf.modelCompliant * epsC;
      H_updateTeaching += g_eta * conf.modelCompliant * epsC;
    }
    if(useTeaching && s==1){
      const Matrix& y = y_buffer[(t)% buffersize]; // eventuell t-1
      const Matrix& xsi = y_teaching - y;
      const Matrix& delta = xsi.multrowwise(g_prime);
      C_updateTeaching += ( delta*(x^T) ) * teacher * epsC;
      H_updateTeaching += delta * teacher * epsC;
      useTeaching=false; // after we applied teaching signal it is switched off until new signal is given
    }
  }
  // we are just using the last shift here! Is this of any problem.
  double error_factor = calcErrorFactor(v, (logaE & 1) !=0, (rootE & 1) !=0);
  C_update *= error_factor;
  H_update *= error_factor;
  if(teaching){
    C_update+=C_updateTeaching;
    H_update+=H_updateTeaching;
  }
};


void InvertMotorBigModel::updateCandH(const Matrix& C_update, const Matrix& H_update, double _squashSize){
  C += C_update.mapP(&_squashSize, squash);
  double H_squashSize = _squashSize*10;
  H += H_update.mapP(&H_squashSize, squash);
}


// learns model and calculates Xsi and A and learns the model
void InvertMotorBigModel::learnModel(int delay){
  const Matrix& x = x_buffer[t % buffersize];
  const Matrix& y = y_buffer[(t - 1 - delay) % buffersize];
  xsi = x -  conf.model->process(y);
  xsi_norm = matrixNorm1(xsi);

  if(xsi_norm > 5*xsi_norm_avg){
    pain= 1; //xsi_norm/ xsi_norm_avg/5;
  } else {
    pain = 0; //pain > 1 ? pain*0.9: 0;
    double error_factor = calcErrorFactor(xsi, (logaE & 2) != 0, (rootE & 2) != 0);
    conf.model->learn(y, x, error_factor);
    if(conf.useS){
      const Matrix& x_primes = calcDerivatives(x_buffer, 1);
      const Matrix& S_update=(( xsi*(x_primes^T) ) * (epsA * error_factor));
      S += S_update.mapP(&squashSize, squash);
    }
  }
  A = conf.model->response(y);

};

/// calculate controller outputs
/// @param x_smooth smoothed sensors Matrix(number_channels,1)
Matrix InvertMotorBigModel::calculateControllerValues(const Matrix& x_smooth){
  return (C*x_smooth+H).map(g);
};


void InvertMotorBigModel::getLastMotors(motor* motors, int len){
  const Matrix& y = y_buffer[t%buffersize];
  y.convertToBuffer(motors, len);
}

Matrix InvertMotorBigModel::calcDerivatives(const matrix::Matrix* buffer,int delay){
  const Matrix& xt    = buffer[(t-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

void InvertMotorBigModel::management(){
  if(dampA){
    conf.model->damp(dampA * managementInterval * epsA);
  }
  if(dampS){
      S -= S*(dampS * managementInterval * epsA);
  }
  if(inhibition){
    kwtaInhibition(C,max((int)kwta,1),inhibition*managementInterval*epsC);
  }else if(limitRF){
    limitC(C,max(1,(int)limitRF));
  }
}

void InvertMotorBigModel::kwtaInhibition(matrix::Matrix& wm, unsigned int k, double damping){
  unsigned int n = wm.getN();
  unsigned int k1 = std::min(n,k); // avoid overfloats
  double inhfactor = 1-damping;
  //  double exfactor  = 1+(damping/k1);
  for(unsigned int i=0; i < wm.getM(); i++){
    Matrix r = wm.row(i).map(fabs);
    double x = getKthLargestElement(r,k1);
    for(unsigned int j=0;j< n; j++){
      if( fabs(wm.val(i,j)) < x){
        wm.val(i,j)*= inhfactor;
      } //else {
        // double d = m - abs_of_elem;  // allways possitive
        //        wm.val(i,j)*= 1+(damping*d); // scale exhitation by distance to max
        //      }
    }
  }
}

void InvertMotorBigModel::limitC(matrix::Matrix& wm, unsigned int rfSize){
  int n = wm.getN();
  int m = wm.getM();
  for(int i=0; i < m; i++){
    for(int j=0; j < n; j++){
      if(std::abs(i-j) > (int)rfSize-1) wm.val(i,j)= 0;
    }
  }
}


/** stores the controller values to a given file. */
bool InvertMotorBigModel::store(FILE* f) const {
  // save matrix values
  C.store(f);
  H.store(f);
  conf.model->store(f);
  if(conf.useS) S.store(f);
  Configurable::print(f,0);
  return true;
}

/** loads the controller values from a given file. */
bool InvertMotorBigModel::restore(FILE* f){
  // save matrix values
  C.restore(f);
  H.restore(f);
  conf.model->restore(f);
  if(conf.useS) S.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}



list<Inspectable::iparamkey> InvertMotorBigModel::getInternalParamNames() const {
  list<iparamkey> keylist;
  if(conf.someInternalParams){
    keylist += store4x4AndDiagonalFieldNames(A, "A");
    if(conf.useS) keylist += store4x4AndDiagonalFieldNames(S, "S");
    keylist += store4x4AndDiagonalFieldNames(C, "C");
    keylist += store4x4AndDiagonalFieldNames(R, "R");
  }else{
    keylist += storeMatrixFieldNames(A, "A");
    if(conf.useS) keylist += storeMatrixFieldNames(S, "S");
    keylist += storeMatrixFieldNames(C, "C");
    keylist += storeMatrixFieldNames(R, "R");
    keylist += storeMatrixFieldNames(y_teaching, "yteach");
  }
  keylist += storeVectorFieldNames(H, "H");
  keylist += string("epsA");
  keylist += string("epsC");
  keylist += string("xsi");
  keylist += string("xsi_norm");
  keylist += string("xsi_norm_avg");
  keylist += string("pain");
  return keylist;
}

list<Inspectable::iparamval> InvertMotorBigModel::getInternalParams() const {
  list<iparamval> l;
  if(conf.someInternalParams){
    l += store4x4AndDiagonal(A);
    if(conf.useS) l += store4x4AndDiagonal(S);
    l += store4x4AndDiagonal(C);
    l += store4x4AndDiagonal(R);
  }else{
    l += A.convertToList();
    if(conf.useS) l += S.convertToList();
    l += C.convertToList();
    l += R.convertToList();
    l += y_teaching.convertToList();
  }
  l += H.convertToList();
  l += epsA;
  l += epsC;
  l += xsi.elementSum();
  l += xsi_norm;
  l += xsi_norm_avg;
  l += pain;
  return l;
}

list<Inspectable::ILayer> InvertMotorBigModel::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  l+=ILayer("x","", number_sensors, 0, "Sensors");
  l+=ILayer("y","H", number_motors, 1, "Motors");
  l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> InvertMotorBigModel::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  return l;
}

double clip095_bigmodel(double x){
  return clip(x,-0.95,0.95);
}

void InvertMotorBigModel::setMotorTeachingSignal(const motor* teaching, int len){
  assert(len == number_motors);
  y_teaching.set(len, 1, teaching);
  y_teaching.toMap(clip095_bigmodel);
  useTeaching=true;
}

void InvertMotorBigModel::setSensorTeachingSignal(const sensor* teaching, int len){
  assert(len == number_sensors);
  Matrix x_teaching(len,1,teaching);
  // calculate the y_teaching, that belongs to the distal teaching value by the inverse model.
  y_teaching = (A.multTM()^(-1)) *  ((A^T) * x_teaching) ;
  y_teaching.toMap(clip095_bigmodel);
  useTeaching=true;
}
