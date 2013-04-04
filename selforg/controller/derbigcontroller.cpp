/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
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

#include "derbigcontroller.h"
//#include "dercontroller.h"
#include "regularisation.h"
#include "invertmotornstep.h"
#include "invertmotorcontroller.h"

using namespace matrix;
using namespace std;

DerBigController::DerBigController( const DerBigControllerConf& conf)
  : InvertMotorController(conf.buffersize, "DerBigController", "$Id$"), conf(conf) {

  assert(conf.model != NULL);

  fantControl = 50;
  fantControlLen = 0;
  fantReset = 5;

  YNoiseGen = 0;
  BNoiseGen = 0;
  noiseB = 0;
  zetaupdate=0.0;
  managementInterval=10;
  useTeaching=false;

  addParameterDef("inhibition",&inhibition,0);
  addParameterDef("kwta",&kwta,2);
  addParameterDef("limitrf",&limitRF,0);
  addParameterDef("dampS",&dampS,0);
  addParameterDef("dampC",&dampC,0);
  addParameterDef("modelcompl",&(this->conf.modelCompliant),0);

  addParameterDef("weighting",&weighting,0.5);
};


DerBigController::~DerBigController(){
  if(x_buffer && y_buffer && eta_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] eta_buffer;
  }

  if(BNoiseGen) delete BNoiseGen;
  if(YNoiseGen) delete YNoiseGen;
}


void DerBigController::init(int sensornumber, int motornumber, RandGen* randGen){
  number_motors  = motornumber;
  number_sensors = sensornumber;
  assert(number_motors && number_sensors);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak

  ID.set(number_motors, number_motors);
  ID.toId();
  ID_Sensor.set(number_sensors, number_sensors);
  ID_Sensor.toId();

  conf.model->init(number_motors, number_sensors, conf.modelInit);
  A = conf.model->response(Matrix(number_motors,1));
//   A_Hat = conf.model->response(Matrix(number_motors,1));
//   ATA_inv = (A.multTM()+ID*0.01)^-1;

  if (conf.useS) S.set(number_sensors, number_sensors*2); // S gets frist and second derivative

  C.set(number_motors,  number_sensors);
  GSC.set(number_motors,  number_sensors);

  // initialise the C matrix with identity + noise (-conf.cNonDiag, conf.cNonDiag) scaled to cInit value
  //C = ((C^0) + C.mapP(randGen, random_minusone_to_one) * conf.cNonDiag) * conf.cInit;
  C = (C^0)  * conf.cInit * 1.0;

  //  DD.set(number_sensors, number_sensors);
  // DD.toId(); DD *= 0.1; // noise strength estimate
  // Dinverse.set(number_sensors, number_sensors);
  // Dinverse = DD^-1;
  //  eta_smooth.set(number_motors,1);

  H.set(number_motors,  1);
  R.set(number_motors, number_motors);
  R=C*A;
  RRT_inv = (R +  ID * 0.2)^-1;
   squashSize = .05;

  xsi.set(number_sensors,1);
  xsi_norm=0;
  xsi_norm_avg=0.2;
  pain=0;

  YNoiseGen = new WhiteUniformNoise();
  YNoiseGen->init(number_motors);
  BNoiseGen = new WhiteUniformNoise();
  BNoiseGen->init(number_sensors);



  x_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  eta_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
    eta_buffer[k].set(number_motors,1);
  }
  y_teaching.set(number_motors, 1);
  x_intern.set(number_sensors,1);
  x_smooth_long.set(number_sensors,1);

  zero_eta.set(number_motors, 1);
  eta.set(number_motors, 1);
  v_smooth.set(number_motors, 1);
  y_smooth.set(number_motors, 1);

  t_rand = int(randGen->rand()*managementInterval);
  initialised = true;
}
//*************** End init *******************

/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void DerBigController::step(const sensor* x_, int number_sensors,
                            motor* y_, int number_motors){
  fillBuffersAndControl(x_, number_sensors, y_, number_motors);
  if(t>buffersize){
    int delay = max(int(s4delay)-1,0);
    // learn Model with actual sensors and with effective motors,
    // calc xsi and A;
    learnModel(delay);
    // learn controller with effective input/output
    learnController(delay);

  }
  // update step counter
  t++;
   if (epsC>0) epsC += .00002;// Anwachsen kann dadurch von Hand gesstoppt werden.
  // epsA *= 1.001;

};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void DerBigController::stepNoLearning(const sensor* x, int number_sensors,
                                            motor*  y, int number_motors ){
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
};

void DerBigController::fillBuffersAndControl(const sensor* x_, int number_sensors,
                                              motor* y_, int number_motors){
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);

  Matrix x(number_sensors,1,x_);

  x_smooth_long += ( x - x_smooth_long ) * 0.005;

  // put new input vector in ring buffer x_buffer
   putInBuffer(x_buffer, x);
   // putInBuffer(x_buffer, x - x_smooth_long);

  // averaging over the last s4avg values of x_buffer
  x_smooth = calculateSmoothValues(x_buffer, t < s4avg ? 1 : int(max(1.0,s4avg)));

  // calculate controller values based on smoothed input values
  Matrix y = calculateControllerValues(x_smooth);
  y += noiseMatrix(y.getM(),y.getN(), *YNoiseGen, -noiseY, noiseY);

  // from time to time call management function. For example damping and inhibition is done.
  if((t+t_rand)%managementInterval==0) management();

  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);

  // convert y to motor*
  y.convertToBuffer(y_, number_motors);
}


/* learn values H,C
   This implementation uses a better formula for g^-1 using Mittelwertsatz
*/
void DerBigController::learnController(int delay){
  // eta = A^-1 xsi (first shift in motor-space at current time)
  //
  //  we use pseudoinverse U=A^T A -> eta = U^-1 A^T xsi
  //   if ((t==buffersize+1) || ((t%10)==2))
  // if ( (t%managementInterval)+1==t_rand)
  //  ATA_inv = (A.multTM() + ID*0.1)^-1;
  // Matrix eta = ATA_inv * ( (A^T) * xsi );
  // noise for the null space!!!!!!!!!

  Matrix y = calculateControllerValues(x_smooth - x_smooth_long);//??????????????????????????????????????????????????????????);
  A_Hat =  conf.model->response(y);
  eta +=  (A_Hat^T) * (A_Hat*eta - xsi) * -0.1 /*(-epsA)*/ + eta * -0.001; //TEST
  // eta += noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY);

  // eta *= sin(t/100); //Test

  eta_buffer[(t-1)%buffersize] = eta; // Todo: work with the reference

  Matrix C_update(C.getM(), C.getN());
  Matrix H_update(H.getM(), H.getN());

  bool teaching = (conf.modelCompliant!=0) || useTeaching;
  Matrix C_updateTeaching;
  Matrix H_updateTeaching;

  if(teaching){
    C_updateTeaching.set(C.getM(), C.getN());
    H_updateTeaching.set(H.getM(), H.getN());
  }

  //Update noise matrix and inverse noise matrix:
  //  double D_length = 100;
  // DD += ((xsi * (xsi^T)) - DD)* ( 1/ D_length);

  // CALC UPDATES

  assert( steps + delay < buffersize);
  // Matrix& eta = zero_eta;
  // Matrix v_old = (eta_buffer[t%buffersize]).map(g);
    Matrix v = zero_eta;

  for(unsigned int s = 1; s <= steps; s++){
    //????? const Matrix& eta = eta_buffer[(t-s)%buffersize].map(g);

     //const Matrix& x      = A * z.map(g) + xsi;
    //    z                    = R * z.map(g) + H + C*xsi; // z is a dynamic itself (not successful)
    // TODO: check delay!!!!!!!!!!!
    const Matrix& x          = x_buffer[(t-s-delay)%buffersize];
    const Matrix& y          = y_buffer[(t-s-delay)%buffersize];
    const Matrix& z          = (C * x + H);// * weighting +( R * y + H) * (1 - weighting);
    //  const Matrix shift       = (eta  + v).map(g); /// maybe clip
    // const Matrix shift       = (eta  + v);//.map(g);  //Regularization  changed 07.02.07
      const Matrix g_prime = Matrix::map2(g_s, z,eta);
    // const Matrix g_prime_inv = g_prime.map(one_over);
    //const Matrix g_2p_div_p  = Matrix::map2(g_ss_div_s, z, eta);
     const Matrix g_prime_inv = Matrix::map2(g_s_inv_expand2, z,eta);
      const Matrix g_2p_div_p  = Matrix::map2(g_ss_div_s_expand2, z, eta);

     //const Matrix zeta =  y *.1;  //TEST!!!!!!!!!!!!!
      //  const Matrix zeta = eta.multrowwise(g_prime_inv); // G'(Z)^-1 * (eta+v)

    if(s==1)  R  = C * A;
//+++++++==Regularisierter Sensorspace  Ansatz fuer den TLE 4.12.07+++++++++
//       GSC = C.multcolwise(g_prime);
//       const Matrix zeta = (( GSC*(GSC^T) + ID * 0.1 )^(-1))*eta;
//        v_smooth += ((GSC^T) * zeta -v_smooth )*.05;
//        v =v_smooth;
//        // v = (GSC^T) * zeta;
//      const Matrix  chi = zeta.multrowwise(g_prime);

//       const Matrix rho = chi.multrowwise((z.map(g)).multrowwise(C*v))*-2;
//       C_update = ( (  chi * (v^T )*.5 ) +  rho*(x^T) ) * epsC;
//       H_update =   rho  * epsC;



//+++++++==Ende Regularisierter Sensorspace  Ansatz fuer den TLE 4.12.07++++++++++

//+++++++==Neuer Ansatz fuer den TLE 30.11.07+++++++++

    const Matrix& alpha = y +  eta *0.5;
    const Matrix& zz          = C * x;
    const Matrix& gg = (C*x+H).map(g);

    const Matrix& beta =     alpha.multrowwise(gg).multrowwise(zz);

    C_update =(( alpha  * (x^T) *.25 + beta  * (x^T) *-2)*epsC);
    H_update = beta  *-epsC;

//+++++++==Ende ** Neuer Ansatz fuer den TLE 30.11.07+++++++++



// // //+++++++==Neuer Ansatz fuer den TLE 24.01.07+++++++++

// //    if ( (t%managementInterval+2)==t_rand) {
//     // we have to do this every step since R is not smooth anymore (due to nonlinear model)
//       RRT_inv = (R +  ID * 0.1)^-1;
//       //  Dinverse =  (DD + ID_Sensor * 0.001)^-1;
//       //    }
//       //  v = RRT_inv * zeta;

//       //********+ TEST smooth v ***************

// //       v_smooth += ( R^T ) * ( R * v_smooth - zeta.map(g) ) * -0.05 - v_smooth * 0.01;

// //       v = v_smooth;
//       // const Matrix  v = RRT_inv * zeta;

//       //********+ End TEST smooth v ***************

// //     const Matrix chi  = (RRT_inv^T) * v; // changed 16.02.07
// //     const Matrix rho  = g_2p_div_p.multrowwise(chi.multrowwise(zeta)) * -1;

// //     C_update += (( (  chi * (v^T ) ) /*- (zeta * (chi^T))*/  )*(A^T) -  rho*(x^T) ) * epsC;
// //     // C_update += (( (  chi * (v^T ) ) /*- (zeta * (chi^T))*/  )*(A^T) - rho*(x^T) ) * Dinverse * epsC ;
// //     // C_update += /*z*/eta * (x^T) * teacher;//Wird jetzt extra gelernt, siehe useTeaching

// //     H_update +=  rho *( -epsC);// - H * .09;

// //     //++++++++++++++++++++++++++++++++++++=Neuer Ansatz fuer den TLE 24.01.07 == Ende





    if(conf.modelCompliant!=0 && s==1){  // learning of the forward task
      // eta is difference between last y and reconstructed one -> used as forward error signal
      // The question is wether to use eta (linearised), zeta (neuron inverse) or eta*g' (Backprop) !
      const Matrix g_p     = z.map(g_s);
      const Matrix& g_eta = eta.multrowwise(g_p);
      C_updateTeaching += ( g_eta*(x^T) ) * conf.modelCompliant * epsC;
      H_updateTeaching += g_eta * conf.modelCompliant * epsC;
    }


    if(useTeaching && s==1){
      const Matrix& y = y_buffer[(t)% buffersize]; // eventuell t-1
       const Matrix& kappa = y_teaching - y;
      const Matrix g_p     = z.map(g_s);
      const Matrix& delta = ( kappa ).multrowwise(g_p);
        C_updateTeaching += ( delta*(x^T) ) * teacher;// * epsC;
       H_updateTeaching += delta * teacher;// * epsC;
      // C_updateTeaching += ( (y_buffer[(t)% buffersize])*(x^T) ) * teacher;// * epsC;
      //H_updateTeaching +=  y_buffer[(t)% buffersize]* teacher;// * epsC;

        useTeaching=false; // false; after we applied teaching signal it is switched off until new signal is given
    }
  }

  //  double error_factor = calcErrorFactor(v, (logaE & 1) !=0, (rootE & 1) !=0);
 //  C_update *= error_factor;
//   H_update *= error_factor;

  if(teaching){
    C_update+=C_updateTeaching;
    H_update+=H_updateTeaching;
  }




  // Controlling the learning parameters:
  double u = calcMatrixNorm(C_update);  //TEST
  double q = calcMatrixNorm( C_update.mapP(&squashSize, squash) ); //TEST
  //    double Au = calcMatrixNorm(A_update);  //TEST
  //double Aq = calcMatrixNorm( A_update.mapP(&squashSize, squash) ); //TEST
   if ( u*u > 1.25 *  q*q)      epsC *=0.95999999999999;
   else {
     C += C_update.mapP(&squashSize, squash) - C* dampC;
     double H_squashSize = squashSize*2.0;//TEST: H soll sich schneller bewegen können.
     H += H_update.mapP(&H_squashSize, squash) - H * dampC; //Test; //Test - H*.001;
     // std::cout <<  "c_up=" << H_update.mapP(&H_squashSize, squash) << std::endl;


   }

//   //  //  A += A_update.mapP(&_squashSize, squash);  //A immer updaten

// //     if ( Au*Au > .5 *  Aq*Aq)      epsA *=0.97;
// //     else {

// //       A += A_update.mapP(&_squashSize, squash);
// //     }

//   // End  Controlling the learning parameters:
};

// learns model and calculates Xsi and A  and learns the model
//New: calculates also eta
void DerBigController::learnModel(int delay){
  const Matrix& x = x_buffer[t % buffersize];
  const Matrix& y = y_buffer[(t - 1 - delay) % buffersize];
    y_smooth += ( y - y_smooth)* .005;
    //  xsi = x -  conf.model->process(y);
    xsi = x- x_smooth_long -  conf.model->process(y-y_smooth);
    // xsi = x -  conf.model->process(y);
  xsi_norm = matrixNorm1(xsi);


  //  if(xsi_norm > 10 /* 5*xsi_norm_avg)*/{ //??????????????????????????????????????????
  //    pain= 1; //xsi_norm/ xsi_norm_avg/5;
  //  } else {
  //    pain = 0; //pain > 1 ? pain*0.9: 0;
    double error_factor = calcErrorFactor(xsi, (logaE & 2) != 0, (rootE & 2) != 0);
      conf.model->learn(y-y_smooth, x- x_smooth_long, error_factor);
      //conf.model->learn(y, x, error_factor);
    if(conf.useS){
      const Matrix& x_primes = calcDerivatives(x_buffer, 1);
      const Matrix& S_update=(( xsi*(x_primes^T) ) * (epsA * error_factor));
      S += S_update.mapP(&squashSize, squash);
      //    }
  }
  A = conf.model->response(y - y_smooth);

//   A_Hat =  conf.model->response(y + eta);
//   eta +=  (A_Hat^T) * (A_Hat*eta - xsi) *-0.1/* (-epsA)*/ - eta * 0.01; //TEST
//   eta += noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY);
//   eta_buffer[(t-1)%buffersize] = eta; // Todo: work with the reference

};

/// calculate controller outputs
/// @param x_smooth smoothed sensors Matrix(number_channels,1)
Matrix DerBigController::calculateControllerValues(const Matrix& x_smooth){
   return (C* x_smooth+H).map(g);
   //return (C* (x_smooth - x_smooth_long) +H).map(g); //TEST

};


void DerBigController::getLastMotors(motor* motors, int len){
  const Matrix& y = y_buffer[t%buffersize];
  y.convertToBuffer(motors, len);
}

Matrix DerBigController::calcDerivatives(const matrix::Matrix* buffer,int delay){
  const Matrix& xt    = buffer[(t-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

void DerBigController::management(){
  if(dampA){
    conf.model->damp(dampA * managementInterval);
  }
  if(dampS){
      S -= S*(dampS * managementInterval);
  }
  if(dampC){
      C -= C*(dampC * managementInterval);
      H -= H*(dampC * managementInterval);
  }
  if(inhibition){
    kwtaInhibition(C,max((int)kwta,1),inhibition*managementInterval*epsC);
  }else if(limitRF){
    limitC(C,max(1,(int)limitRF));
  }
}

void DerBigController::kwtaInhibition(matrix::Matrix& wm, unsigned int k, double damping){
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

void DerBigController::limitC(matrix::Matrix& wm, unsigned int rfSize){
  int n = wm.getN();
  int m = wm.getM();
  for(int i=0; i < m; i++){
    for(int j=0; j < n; j++){
      if(std::abs(i-j) > (int)rfSize-1) wm.val(i,j)= 0;
    }
  }
}


/** stores the controller values to a given file. */
bool DerBigController::store(FILE* f) const {
  // save matrix values
  C.store(f);
  H.store(f);
  if(conf.useS)  S.store(f);
  conf.model->store(f);
  Configurable::print(f,0);
  return true;
}

/** loads the controller values from a given file. */
bool DerBigController::restore(FILE* f){
  // save matrix values
  C.restore(f);
  H.restore(f);
  if(conf.useS)  S.restore(f);
  conf.model->restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}


void DerBigController::notifyOnChange(const paramkey& key){
  if(key == "epsA") {
    cout << "MODEL old value of eps: " << conf.model->getParam("eps") << endl;
    conf.model->setParam("eps",epsA);
  }
}


list<Inspectable::iparamkey> DerBigController::getInternalParamNames() const {
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
    keylist += storeVectorFieldNames(eta, "eta");
    // keylist += storeVectorFieldNames(xsi, "xsi");
    keylist += storeVectorFieldNames(x_smooth_long, "v_smooth");
  keylist += string("weighting");
  keylist += string("epsA");
  keylist += string("epsC");
  // keylist += string("xsi");
  keylist += string("xsi_norm");
  keylist += string("xsi_norm_avg");
  keylist += string("pain");
  return keylist;
}

list<Inspectable::iparamval> DerBigController::getInternalParams() const {
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
  l += eta.convertToList();
  //l += xsi.convertToList();
  l += x_smooth_long.convertToList();//TEST
  l += weighting;
  l += epsA;
  l += epsC;
   l += xsi.elementSum();
    l += xsi_norm;
  l += xsi_norm_avg;
  l += pain;
  return l;
}

list<Inspectable::ILayer> DerBigController::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  l+=ILayer("x","", number_sensors, 0, "Sensors");
  l+=ILayer("y","H", number_motors, 1, "Motors");
  l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> DerBigController::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  return l;
}

//double clip095(double x){
// return clip(x,-0.95,0.95);
//}

void DerBigController::setMotorTeachingSignal(const motor* teaching, int len){
  assert(len == number_motors);
  y_teaching.set(len, 1, teaching);
  //  y_teaching.toMap(clip095); //TODO where is clip
  useTeaching=true;
}

void DerBigController::setSensorTeachingSignal(const sensor* teaching, int len){
  assert(len == number_sensors);
  Matrix x_teaching(len,1,teaching);
  // calculate the y_teaching, that belongs to the distal teaching value by the inverse model.
  // y_teaching = (A.multTM()^(-1)) *  ((A^T) * x_teaching) *0.000000000; //TEST
  //y_teaching.toMap(clip095); //TODO
  useTeaching=true;
}

double DerBigController::calcMatrixNorm(const Matrix& m){
  return m.map(fabs).elementSum() / (m.getM() * m.getN());
}
