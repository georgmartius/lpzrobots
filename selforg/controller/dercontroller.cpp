/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/

#include "dercontroller.h"
#include "regularisation.h"

using namespace matrix;
using namespace std;


DerController::DerController( const DerControllerConf& conf)
  : InvertMotorController(conf.buffersize,"DerController", "$Id$"), conf(conf) {
  // entspricht:  this->conf = conf;
  //uuu
  addParameterDef("fantcontrol", &fantControl,50,0,1000,"interval length for fantasising");
  addParameterDef("fantcontrollen", &fantControlLen,0,0,100,"length of fantasy control");
  addParameterDef("fantreset", &fantReset,5,0,1000,"number of fantasy control events before reseting internal state");

  YNoiseGen = 0;
  BNoiseGen = 0;
  noiseB = 0;

  zetaupdate=0.0;
};


DerController::~DerController(){
  if(x_buffer && y_buffer && eta_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] eta_buffer;
  }
  if(BNoiseGen) delete BNoiseGen;
  if(YNoiseGen) delete YNoiseGen;
}


void DerController::init(int sensornumber, int motornumber, RandGen* randGen){
  number_motors  = motornumber;
  number_sensors = sensornumber;
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak


  //  z.set(number_motors,1);
  A.set(number_sensors, number_motors);
  if (conf.useS) S.set(number_sensors, number_sensors);
  C.set(number_motors,  number_sensors);
  DD.set(number_sensors, number_sensors);
  Dinverse.set(number_sensors, number_sensors);
  //  v_smooth.set(number_motors,1);
  eta_smooth.set(number_motors,1);

  DD.toId(); DD *= 0.1; //TODO hier noise strength

  H.set(number_motors,  1);
  B.set(number_sensors, 1);
  R.set(number_motors, number_motors);
  SmallID.set(number_motors, number_motors);
  SmallID.toId();
  SmallID *= .1;//0.001; New  //Test this seems to be sensitive!!
  xsi.set(number_sensors,1);
  xsi_norm=0;
  xsi_norm_avg=0.2;
  pain=0;

  YNoiseGen = new WhiteUniformNoise();
  YNoiseGen->init(number_motors);
  BNoiseGen = new WhiteUniformNoise();
  BNoiseGen->init(number_sensors);

  A.toId(); // set A to identity matrix;
  if (conf.useS) S.toId(); // set S to identity matrix;

  // initialise the C matrix with identity + noise (-conf.cNonDiag, conf.cNonDiag) scaled to cInit value
  //C = ((C^0) + C.mapP(randGen, random_minusone_to_one) * conf.cNonDiag) *  conf.cInit;
  C = ((C^0) *  conf.cInit + C.mapP(randGen, random_minusone_to_one) * conf.cNonDiag)*0.0;//  conf.cInit;

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

  zero_eta.set(number_motors, 1);
  initialised = true;
}


/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void DerController::step(const sensor* x_, int number_sensors,
                         motor* y_, int number_motors){
  fillBuffersAndControl(x_, number_sensors, y_, number_motors);
  if(t>buffersize){
    // learn controller with effective input/output
    learnController();

    // learn Model with actual sensors and with effective motors;
    // learnModel(delay);
  }
  // update step counter
  epsC += .001;
  // epsA *= 1.001;

  t++;
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void DerController::stepNoLearning(const sensor* x, int number_sensors,
                                            motor*  y, int number_motors ){
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
};//TODO: Das kann wohl raus?

void DerController::fillBuffersAndControl(const sensor* x_, int number_sensors,
                                          motor* y_, int number_motors){
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);

  Matrix x(number_sensors,1,x_);

  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);

  // averaging over the last s4avg values of x_buffer
  x_smooth = calculateSmoothValues(x_buffer, t < s4avg ? 1 : int(std::max(1,s4avg)));

  // calculate controller values based on smoothed input values
  Matrix y = calculateControllerValues(x_smooth);
  if(noiseY!=0){// add motor noise
    // calculate a noise matrix for motor values
    Matrix y_noise  = noiseMatrix(y.getM(),y.getN(), *YNoiseGen, -noiseY, noiseY); // noise for bias
    y += y_noise;
  }

  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);


  /////// FANTASY
  if (conf.useFantasy) {
    Matrix y_fant = calculateControllerValues(x_intern);
    x_intern = model(x_intern, y_fant);
    if((t % fantControl) < fantControlLen){ // control is now given by fantasy
      y = y_fant; // maybe introduce same rank function here
    }
    if( ((t-fantControlLen) % (fantControl*fantReset)) == 0){ // fantasy is reset to real world
      x_intern = x_smooth;
    }
  }

  // convert y to motor*
  y.convertToBuffer(y_, number_motors);
}

double regularizedInverse2(double v){
  return 1/(fabs(v)+0.1);
}


/// calculates Eta which corrensponds to xsi_t which means eta_{t-1}
//  @param delay 0 for no delay and n>0 for n timesteps delay in the SML
void DerController::calcEtaAndBufferIt(int delay){
  // eta = A^-1 xsi (first shift in motor-space at current time)
  //
  //  we use pseudoinverse U=A^T A -> eta = U^-1 A^T xsi
  //  if ((t==buffersize+1) || ((t%10)==2))
    if (AAT.isNulltimesNull() || ((t%10)==5))
       AAT = (A.multTM()+SmallID)^-1;
     Matrix eta = AAT * ( (A^T) * xsi );
     //     eta_smooth +=  (A^T) * (A*eta_smooth - xsi) * 0.1 - eta_smooth * 0.01; //TEST
     //  Matrix eta = eta_smooth;
      //     eta += noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY); // noise for the null space!!!!!!!!!
  if(relativeE != 0) { // divide eta by |y|  == relative error
    // const Matrix& y = y_buffer[t % buffersize];
    //   eta = eta.multrowwise(y.map(regularizedInverse2)); //TODO Prüfen was das macht.
  }
  eta_buffer[(t-1)%buffersize] = eta; // Todo: work with the reference
}

/// calculates xsi for the current time step using the delayed y values
//  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
void DerController::calcXsi(int delay){
  const Matrix& x = x_buffer[t % buffersize];
  const Matrix& y = y_buffer[(t - 1 - delay) % buffersize];
  const Matrix& x_tm1 = x_buffer[(t-1) % buffersize];
  xsi = x -  model(x_tm1,y);
  xsi_norm = calcMatrixNorm(xsi); //?????????????????
}

Matrix DerController::model(const Matrix& x, const Matrix& y){
  if(conf.useS){
    return ((A * y + S * x) + B);
  } else {
    return ((A * y ) + B);        //TEST B-Lernen
  }
}

/// learn values H,C
// This is the implementation uses a better formula for g^-1 using Mittelwertsatz
void DerController::learnController(){

  Matrix C_update(C.getM(), C.getN());
  Matrix H_update(H.getM(), H.getN());

  Matrix A_update(A.getM(), A.getN());
  //TODO: B lernen wieder einführen!

  //
  int delay = std::max(int(s4delay)-1,0);
  calcXsi(delay);            // calculate the error (use delayed y values)
  calcEtaAndBufferIt(delay);

  //Update noise matrix and inverse noise matrix:
  double D_length = 100;
    DD += ((xsi * (xsi^T)) - DD)* ( 1/ D_length);
  calcCandHandAUpdates(C_update, H_update, A_update, 0);
  // calcCandHUpdates(C_update, H_update, 0);
  // Test if(conf.useTeaching){
  //Test calcCandHUpdatesTeaching(C_update, H_update, 0);
  //}

  if(adaptRate>0){ // adapt learning rate
    // double E;
    //  double norm = calcMatrixNorm(C); // + calcMatrixNorm(H);
    // double update_norm = calcMatrixNorm(C_update) + calcMatrixNorm(H_update)*0.05;

    // epsC = adapt(epsC, update_norm, norm*nomUpdate, adaptRate, adaptRate*10);
    //Test epsC = min( 5.0, epsC);
  }
   updateCandHandA(C_update, H_update, A_update, squashSize);
};


/// calculates the Update for C and H
// @param y_delay timesteps to delay the y-values.  (usually 0)
//  Please note that the delayed values are NOT used for the error calculation
//  (this is done in calcXsi())


//Central part of the algorithm:

void DerController::calcCandHandAUpdates(Matrix& C_update, Matrix& H_update, Matrix& A_update, int y_delay){
  assert( steps + y_delay < buffersize);
  // Matrix& eta = zero_eta;
  // Matrix v_old = (eta_buffer[t%buffersize]).map(g);
   Matrix v = zero_eta;





  for(unsigned int s = 1; s <= steps; s++){
      const Matrix& eta = eta_buffer[(t-s)%buffersize].map(g);

     //const Matrix& x      = A * z.map(g) + xsi;
    //    z                    = R * z.map(g) + H + C*xsi; // z is a dynamic itself (not successful)
     const Matrix& x          = x_buffer[(t-s-y_delay)%buffersize];
    const Matrix& y          = y_buffer[(t-s-y_delay)%buffersize];
    const Matrix& z          = (C * x + H) *.5 +  ( R * y  + H) * .5; //TEST: gleiche Stärke innen und aussen
    const Matrix shift       = (eta  + v).map(g);
    // const Matrix shift       = (eta  + v);//.map(g);  //Regularization  changed 07.02.07
    const Matrix g_prime     = Matrix::map2(g_s, z, shift);
    const Matrix g_prime_inv = g_prime.map(one_over);
    const Matrix g_2p_div_p  = Matrix::map2(g_ss_div_s, z, shift);

    const Matrix zeta = shift.multrowwise(g_prime_inv); // G'(Z)^-1 * (eta+v)

    if(s==1) {  R  = C * A;
//  //+++++++==Neuer Ansatz fuer den TLE 29.10.07+++++++++

//     const Matrix xHat = x + v;
//     const Matrix zHat = C * x + H;
//     const Matrix g_prim     = z.map(g);
//      const Matrix yHat = z.multrowwise(g_prime);
//      x = A * z;


// //      const Matrix chi   = (RRT^T) * zeta;
// //      const Matrix
// //      const Matrix XsiV = x_buffer[(t)] - A * ;



// //+++++++==Ende Neuer Ansatz fuer den TLE 29.10.07+++++++++



    }
//+++++++==Neuer Ansatz fuer den TLE 24.01.07+++++++++

 if (RRT.isNulltimesNull() || ((t%10)==0))
          RRT = (R +  SmallID * 2)^-1;
    v = RRT * zeta;
    // const Matrix chi  = (RRT^T) * zeta;
    // const Matrix rho  = g_2p_div_p.multrowwise(chi.multrowwise(zeta)) * -1;
    const Matrix chi  = (RRT^T) * v; // changed 16.02.07
    const Matrix rho  = g_2p_div_p.multrowwise(chi.multrowwise(v)) * -1;
    // Dinverse =  (DD + SmallID *.05)^-1;
    // C_update += Dinverse * (( (  chi * (v^T ) ) /*- (zeta * (chi^T))*/  )*(A^T) - rho*(x^T) ) * epsC ;
    C_update += (( (  chi * (v^T ) ) /*- (zeta * (chi^T))*/  )*(A^T) - rho*(x^T) ) * epsC;// * Dinverse ;
    C_update +=  C  * -dampA;  // Mit Dämpfung
     C_update -= /*z*/eta * (x^T) * teacher;// Neu mit Lernen der Vorwärtsaufgabe !!!!!!!!!!???????

     H_update += /*Dinverse* */ rho * -epsC ; // TEST langsameres H
     //  H_update +=  H  * -dampA;


    //++++++++++++++++++++++++++++++++++++=Neuer Ansatz fuer den TLE 24.01.07 == Ende

 // Learning of the model parameters:
     if ( s==1 ) {
     if (AAT.isNulltimesNull() || ((t%10)==5))
     const Matrix AAT = (A.multTM()+ SmallID)^-1;
       A_update +=  xsi * (y^T) * epsA;//  +  (C^T)*( Chi*(V^T)  ) * epsC * zetaupdate - A * dampA;
       B +=  xsi  * epsA * .05; // TEST B-Lernen

     }

  }


  double error_factor = calcErrorFactor(v, (logaE & 1) !=0, (rootE & 1) !=0);
  C_update *= error_factor;
  H_update *= error_factor;
  // error_factor = calcErrorFactor(v, (logaE & 2) !=0, (rootE & 2) !=0);
  error_factor = calcErrorFactor(xsi, (logaE & 2) !=0, (rootE & 2) !=0);
  A_update *= error_factor;
  // cout << endl;
};

/// calculates the Update for C and H using the teaching signal
//  @param y_delay timesteps to delay the y-values.  (usually 0)
//  Please note that the delayed values are NOT used for the error calculation
//  (this is done in calcXsi())


// calculates the error_factor for either logarithmic (E=ln(e^T*e)) or square (E=sqrt(e^t*e)) error
double DerController::calcErrorFactor(const Matrix& e, bool loga, bool root) {
  double error_factor = 1;

    if (loga){   // using logarithmic error E=ln(v^T*v)
    error_factor= 1/(e.multTM().val(0,0)+0.000001)*0.01; // factor 1/100 for normalising (empirically)
  }
  if (root){  // using root error E=(v^T*v)^(1/2)
    error_factor= 1/sqrt(e.multTM().val(0,0)+0.000001)*0.1; // factor 1/10 for normalising (empirically)
  }
  return error_factor;
}


void DerController::updateCandHandA(const Matrix& C_update, const Matrix& H_update,const Matrix& A_update, double _squashSize){




  // Controlling the learning parameters:
   double u = calcMatrixNorm(C_update);  //TEST
   double q = calcMatrixNorm( C_update.mapP(&_squashSize, squash) ); //TEST
//   //    double Au = calcMatrixNorm(A_update);  //TEST
//   // double Aq = calcMatrixNorm( A_update.mapP(&_squashSize, squash) ); //TEST
    if ( u*u > 1.05 *  q*q)      epsC *=0.95;
//    else {
      C += C_update.mapP(&_squashSize, squash);
      double H_squashSize = _squashSize*5.0;//TEST: H soll sich schnelleer bewegen können.
      H += H_update.mapP(&H_squashSize, squash); //Test - H*.001;


        A += A_update.mapP(&_squashSize, squash);
//    }

  //  //  A += A_update.mapP(&_squashSize, squash);  //A immer updaten

//     if ( Au*Au > .5 *  Aq*Aq)      epsA *=0.97;
//     else {

//       A += A_update.mapP(&_squashSize, squash);
//     }

  // End  Controlling the learning parameters:
}

/// calculate controller outputs
/// @param x_smooth smoothed sensors Matrix(number_channels,1)
Matrix DerController::calculateControllerValues(const Matrix& x_smooth){
  return (C*x_smooth+H).map(g);
};

void DerController::getLastMotors(motor* motors, int len){
  const Matrix& y = y_buffer[t%buffersize];
  y.convertToBuffer(motors, len);
}


bool DerController::store(FILE* f) const{
  // save matrix values
  C.store(f);
  H.store(f);
  A.store(f);
  B.store(f);
  S.store(f);
  return true;
}

bool DerController::restore(FILE* f){
  C.restore(f);
  H.restore(f);
  A.restore(f);
  B.restore(f);
  S.restore(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}


list<Inspectable::iparamkey> DerController::getInternalParamNames() const {
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
  }
  keylist += storeVectorFieldNames(H, "H");
  keylist += storeVectorFieldNames(B, "B");
  if(conf.useFantasy)
      keylist += storeVectorFieldNames(x_intern, "xFant");
  keylist += string("epsA");
  keylist += string("epsC");
  keylist += string("xsi");
  keylist += string("xsi_norm");
  keylist += string("xsi_norm_avg");
  keylist += string("pain");
  return keylist;
}

list<Inspectable::iparamval> DerController::getInternalParams() const {
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
  }
  l += H.convertToList();
  l += B.convertToList();
  if(conf.useFantasy)
      l += x_intern.convertToList();
  l += epsA;
  l += epsC;
  l += xsi.elementSum();
  l += xsi_norm;
  l += xsi_norm_avg;
  l += pain;
  return l;
}

list<Inspectable::ILayer> DerController::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  l+=ILayer("x","", number_sensors, 0, "Sensors");
  l+=ILayer("y","H", number_motors, 1, "Motors");
  l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> DerController::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  if(conf.useS) l+=IConnection("S", "x", "xP");
  return l;
}

void DerController::setTeachingMode(bool onOff){
  conf.useTeaching=onOff;
}

bool DerController::getTeachingMode(){
  return conf.useTeaching;
}

void DerController::setMotorTeachingSignal(const motor* teaching, int len){
  assert(len == number_motors);
  y_teaching.set(len, 1, teaching);
}

double DerController::calcMatrixNorm(const Matrix& m){
  return m.map(fabs).elementSum() / (m.getM() * m.getN());
}

