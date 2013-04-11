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

#include "derpseudosensor.h"
//#include "dercontroller.h"
#include "regularisation.h"
#include "invertmotornstep.h"
#include "invertmotorcontroller.h"
//#include "statistictools.h"
using namespace matrix;
using namespace std;

DerPseudoSensor::DerPseudoSensor( const DerPseudoSensorConf& conf)
  : InvertMotorController(conf.buffersize, "DerPseudoSensor", "$Id$"), conf(conf) {

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

  addParameterDef("dampS",&dampS,0);
  addParameterDef("dampC",&dampC,0);
  addParameterDef("modelcompl",&(this->conf.modelCompliant),0);
  addParameterDef("epsSat",&epsSat,0.1);
  addParameterDef("satT",&satelliteTeaching,0.01);

  addParameterDef("weighting",&weighting,1.0);
};


DerPseudoSensor::~DerPseudoSensor(){
  if(x_buffer && y_buffer && eta_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] eta_buffer;
  }

  if(BNoiseGen) delete BNoiseGen;
  if(YNoiseGen) delete YNoiseGen;
}


void DerPseudoSensor::init(int sensornumber, int motornumber, RandGen* randGen){
  number_motors  = motornumber;
  number_sensors = sensornumber;
  assert(number_motors && number_sensors);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak


  ID.set(number_motors, number_motors);
  ID.toId();
  ID_Sensor.set(number_sensors, number_sensors);
  ID_Sensor.toId();

    conf.model->init(number_motors, number_sensors, conf.modelInit);
     A.set(number_sensors,  number_motors); //TEST
     A=A^0;  //TEST


     //A = conf.model->response(Matrix(number_motors,1));
  //   A_Hat = conf.model->response(Matrix(number_motors,1));
  ATA_inv = (A.multTM()+ID*0.0)^-1;

  if (conf.useS) S.set(number_sensors, number_sensors*2); // S gets frist and second derivative

  if(conf.sat) conf.sat->init(number_sensors,number_motors, conf.cInit);

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
  //  R.set(number_motors, number_motors);
  R.set(number_sensors, number_sensors);
  RG.set(number_sensors, number_sensors);
  Q.set(number_sensors, number_motors);
  Q1.set(number_sensors, number_motors);
  CCT_inv.set(number_motors, number_motors);
  CST.set(number_motors, number_sensors);
  //  R=C*A;
  //   RRT_inv = (R +  ID * 0.2)^-1;
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
  x_smooth.set(number_sensors,1);
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
void DerPseudoSensor::step(const sensor* x_, int number_sensors,
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

};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void DerPseudoSensor::stepNoLearning(const sensor* x, int number_sensors,
                                     motor*  y, int number_motors ){
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
  cout << y << endl;
};

void DerPseudoSensor::fillBuffersAndControl(const sensor* x_, int number_sensors,
                                            motor* y_, int number_motors){
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);
  Matrix x(number_sensors,1,x_);
   x = ((x * .9).map(g) )*(1/.9);//TEST
  // averaging over the last s4avg values of x_buffer
  x_smooth += (x - x_smooth)*(1.0/s4avg);//calculateSmoothValues(x_buffer, t < s4avg ? 1 : int(max(1.0,s4avg)));
  x_smooth_long += ( x - x_smooth_long ) * 0.0001;

  //x_smooth +=  ( x - x_smooth_long - x_smooth )* s4avg;

  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);
  //putInBuffer(x_buffer, x - x_smooth_long);//TEST

  // calculate controller values based on smoothed input values//*.001;//*(tanh(exp(40*grang2)/5)*5);//TEST Granger
  Matrix y = calculateControllerValues(x_smooth);
 // Matrix y = (conf.sat->process(x_smooth));//TEST
  //  y += noiseMatrix(y.getM(),y.getN(), *YNoiseGen, -noiseY, noiseY);

  // from time to time call management function. For example damping and inhibition is done.
  // if((t+t_rand)%managementInterval==0) management();

  // put new output vector in ring buffer y_buffer
   putInBuffer(y_buffer, y);
   //putInBuffer(y_buffer, y +  x_smooth_long);//TEST

  // convert y to motor*; y is sent to the motors.
  y.convertToBuffer(y_, number_motors);
}


/* learn values H,C
   This implementation uses a better formula for g^-1 using Mittelwertsatz
*/
void DerPseudoSensor::learnController(int delay){

  const Matrix& x = x_buffer[(t-delay)%buffersize];
  const Matrix& y = calculateControllerValues( x_smooth);// y_buffer[(t-delay)%buffersize];
  const Matrix y_sat = (conf.sat->process(x_smooth));


  // learning of satellite network
//   if(conf.sat){
//     const Matrix& ySat = conf.sat->process(x_smooth);
//     conf.sat->learn(x_smooth,y,epsSat);
//     cout << "satlearning" << endl;
//     if(satelliteTeaching){// set teaching signal for controller
//       if(useTeaching) // if external teaching signal then combine
//         y_teaching+=ySat;
//       else
//         y_teaching=ySat;
//       double c=0.95;
//       y_teaching.toMapP(&c, clip);
//       useTeaching=true;
//     }
//   }
  //NEW APPROACHES Calculate Psi (Matrix R)

  const Matrix deltaR = ( x_buffer[(t)%buffersize] - x_buffer[(t-delay)%buffersize] - R * x_buffer[(t-delay)%buffersize]).map(tanh);
  R += deltaR * (x_buffer[(t-delay)%buffersize]^T ) * .01 - R * .00001;
  grang1 = ((deltaR^T)*deltaR).val(0,0);

  // Granger causality

  const Matrix deltaQ = ( x_buffer[(t)%buffersize] - x_buffer[(t-delay)%buffersize]
                          - ( RG * x_buffer[(t-delay - 3)%buffersize] )
                          - ( Q * x_buffer[(t-delay)%buffersize])
                          - ( Q1 * x_buffer[(t - 5 -delay)%buffersize])).map(tanh);
  RG += deltaQ *( x_buffer[(t-delay - 3)%buffersize]^T)* .01 - RG * .00001;
  Q += deltaQ * (  x_buffer[(t-delay)%buffersize]^T ) * .01 - Q * .00001;
  Q1 += deltaQ * (  x_buffer[(t-delay - 5)%buffersize]^T ) * .01 - Q1 * .00001;
   grang2 = ((deltaQ^T)*deltaQ).val(0,0);
  causal += (1 - grang2/grang1 - causal)*.2;
  causalfactor = (tanh(causal*4)+1);
  grang2 = grang1 - grang2;


 //NEW APPROACHES End


 // eta = A^-1 xsi (first shift in motor-space at current time)
  //
  //  we use pseudoinverse U=A^T A -> eta = U^-1 A^T xsi

  //    A_Hat =  conf.model->response(y);
  //   eta +=  (A_Hat^T) * (A_Hat*eta - xsi) * -0.1 /*(-epsA)*/ + eta * -0.001; //TEST
 // eta = A^-1 xsi (first shift in motor-space at current time)
  if ((t%50)==2)
    ATA_inv = (A.multTM() + ID*0.1)^-1;
  xsi = x - A*( y * weighting + y_sat * (1 - weighting ));
  eta =  ATA_inv * (A^T) * xsi;
  //  noise for the null space:
  eta += noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY);
  //  eta = noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY); // TEST
  eta_buffer[(t-1)%buffersize] = eta;

  Matrix C_update(C.getM(), C.getN());
  Matrix H_update(H.getM(), H.getN());

  bool teaching = (conf.modelCompliant!=0) || useTeaching;
  Matrix C_updateTeaching;
  Matrix H_updateTeaching;

  if(teaching){
    C_updateTeaching.set(C.getM(), C.getN());
    H_updateTeaching.set(H.getM(), H.getN());
  }


    assert( steps + delay < buffersize);


  //+++++++==Regularisierter Sensorspace  Ansatz fuer den TLE 14.01.08+++++++++



  //The learning step
  if (steps > 2) {
    cout << "steps must be 1 or 2" << endl;
    steps =2 ;
  }

//    const Matrix& y          = y_buffer[(t-s-delay)%buffersize];
  const Matrix& z          = (C * x_smooth + H);
  const Matrix zpluseta = z;// + eta * .5;
  const Matrix g_prime = zpluseta.map(g_derivative);
  CST = C.multrowwise(g_prime);

  if(steps == 1) {

  CCT_inv = (( CST * (CST^T)).pluslambdaI(0.1))^(-1);
  const Matrix mue = CCT_inv*eta;
  const Matrix chi = mue.multrowwise(g_prime);
  const Matrix rho = mue.multrowwise((z.map(g)).multrowwise(CST*(CST^T)*mue))*-2;
  C_update = ( (  chi * (chi^T )* C) +  rho*((x)^T)) * epsC*causalfactor;
  //  C_update = ( (  chi * (chi^T )* C) +  rho*((A*y)^T) /*!!!!!!!!!!!!*/ ) * epsC;
  H_update =   rho  * epsC*causalfactor;
    }
   if (steps==2) {

     CCT_inv = ( ( CST * A * CST ) * (( CST * A * CST )^T) + ( CST * (CST^T)*.4).pluslambdaI(0.1))^(-1);
  const Matrix mue = CCT_inv*eta;
  const Matrix chi = mue.multrowwise(g_prime);
  const Matrix rho = mue.multrowwise((z.map(g)).multrowwise(CST*(CST^T)*mue))*-2;
  C_update = ( (  chi * (chi^T )* C) +  rho*(x^T)) * epsC*causalfactor;// * .001;//*(tanh(exp(40*grang2)/5)*5);//TEST Granger
  //    C_update = ( (  chi * (chi^T )* C) +  rho*((A*y)^T) *3 /*!!!!!!!!!!!!*/ ) * epsC;
  H_update =   rho  * epsC*causalfactor;

  const Matrix kappa = ((CST * A * CST)^T) * mue;
  const Matrix kappa1 =  CST * kappa;
  const Matrix kappa2 = CST * A * kappa1;
    C_update +=  (chi * (( A * kappa1 )^T) + ((kappa2.multrowwise(mue*(-2))).multrowwise(z.map(g))*(x^T))
                  + ( (A^T)*(C^T) * chi ).multrowwise(g_prime) * (kappa^T) + ((( (A^T)*(C^T) * chi ).multrowwise(z.map(g))).multrowwise(kappa1*(-2)))*(x^T))*epsC*causalfactor;

//     C_update +=  (chi * (( A * kappa1 )^T) + ((kappa2.multrowwise(mue*(-2))).multrowwise(z.map(g))*((A*y)^T))
//     + ( (A^T)*(C^T) * chi ).multrowwise(g_prime) * (kappa^T) + ((( (A^T)*(C^T) * chi ).multrowwise(z.map(g))).multrowwise(kappa1*(-2)))*((A*y)^T))*epsC;
    H_update +=   ((kappa2.multrowwise(mue*(-2))).multrowwise(z.map(g)) + ((( (A^T)*(C^T) * chi ).multrowwise(z.map(g))).multrowwise(kappa1*(-2))))*epsC*causalfactor;


   }

//Weighting the time loop error
  double X2 = sqrt(((eta^T)*eta).val(0,0));
  C_update *= .3/(X2 +0.001);
  H_update *= .3/(X2 +0.001);

  //Hier Teaching
  //conf.sat->learn(x_smooth,y*.98,epsSat);
 conf.sat->process(x_buffer[(t-delay-10)%buffersize]);
 conf.sat->learn(x_buffer[(t-delay-10)%buffersize],y_buffer[(t - 10)%buffersize]*.99,epsSat*causalfactor);//learning with the current causalfactor
   const Matrix delta = ( y_sat*.99 - y ).multrowwise(g_prime);
  // const Matrix delta = ( x - y ).multrowwise(g_prime); //TEST
  C_update += delta * (x^T)*teacher*causalfactor;// *epsC;
  H_update += delta * teacher*causalfactor;// *epsC;
 // const  Matrix yy = conf.sat->process(x_smooth);//TEST ?????????????????????????????
//   conf.sat->learn(x_smooth,yy - mue,epsSat);//TEST


  //+++++++==Ende Regularisierter Sensorspace  Ansatz fuer den TLE  14.01.08++++++++++


  //***********Experiments 11.03.08*****************

//   //Calculate the time loop error and ...

//   const Matrix v =  (CST^T)* mue;
//   TLE = ((v^T) * v).val(0,0);
//   pain = ((eta^T) * eta).val(0,0);//pain nur als Hilfsvariable benutzt!
//   // double X2 = ((x_smooth^T)*x_smooth).val(0,0);
// //   C_update *= .1/(X2 +0.01);
// //   H_update *= .1/(X2 +0.01);

//   // End Calculate the time loop error


  //**********End * Experiments 11.03.08*****************


  if(conf.modelCompliant!=0){  // learning of the forward task
    // eta is difference between last y and reconstructed one -> used as forward error signal
    // The question is wether to use eta (linearised), zeta (neuron inverse) or eta*g' (Backprop) !
    const Matrix g_p     = z.map(g_derivative);
    const Matrix& g_eta = eta.multrowwise(g_p);
    C_updateTeaching += ( g_eta*(x^T) ) * conf.modelCompliant * epsC;
    H_updateTeaching += g_eta * conf.modelCompliant * epsC;
  }

  if(useTeaching){
    const Matrix& y = y_buffer[(t)% buffersize]; // eventuell t-1
    const Matrix& kappa = y_teaching - y;
    const Matrix g_p     = z.map(g_derivative);
    const Matrix& delta = ( kappa ).multrowwise(g_p);
    C_updateTeaching += ( delta*(x^T) ) * teacher;// * epsC;
    H_updateTeaching += delta * teacher;// * epsC;
    // C_updateTeaching += ( (y_buffer[(t)% buffersize])*(x^T) ) * teacher;// * epsC;
    //H_updateTeaching +=  y_buffer[(t)% buffersize]* teacher;// * epsC;

    useTeaching=false; // false; after we applied teaching signal it is switched off until new signal is given
  }
  //}

  //  double error_factor = calcErrorFactor(v, (logaE & 1) !=0, (rootE & 1) !=0);
  //   C_update *= error_factor;
  //   H_update *= error_factor;

//     if(teaching){
//       C_update+=C_updateTeaching;
//       H_update+=H_updateTeaching;
//     }

  // Controlling the learning parameters:
  double Test_squashSize = squashSize/5.0;
  double u = calcMatrixNorm(C_update);  //TEST
  double q = calcMatrixNorm( C_update.mapP(&Test_squashSize, squash) ); //TEST
  //    double Au = calcMatrixNorm(A_update);  //TEST
  //double Aq = calcMatrixNorm( A_update.mapP(&squashSize, squash) ); //TEST
  if (epsC>0)  epsC *= 1.005;
  if ( u*u > 1.01 *  q*q)      epsC *=0.8;
  // else {
  C += C_update.mapP(&squashSize, squash) - C* dampC ;
  H += H_update.mapP(&squashSize, squash) - H * dampC;

  //}

  //   // End  Controlling the learning parameters:


};

// learns model and calculates Xsi and A  and learns the model
//New: calculates also eta
void DerPseudoSensor::learnModel(int delay){
  const Matrix& x = x_buffer[t % buffersize];
  const Matrix& y = y_buffer[(t - 1 - delay) % buffersize];//TEST
  y_smooth += ( y - y_smooth)* s4avg;
  //  xsi = x -  conf.model->process(y);
  //  xsi = x- x_smooth_long -  conf.model->process(y-y_smooth);
   xsi = x -  conf.model->process(y);
   //  xsi = x - A * y; //TEST
  //xsi_norm = matrixNorm1(xsi);


  double error_factor = calcErrorFactor(xsi, (logaE & 2) != 0, (rootE & 2) != 0);
  //   conf.model->learn(y-y_smooth, x- x_smooth_long, error_factor);
   conf.model->learn(y, x,error_factor);

  if(conf.useS){
    const Matrix& x_primes = calcDerivatives(x_buffer, 1);
    const Matrix& S_update=(( xsi*(x_primes^T) ) * (epsA * error_factor));
    S += S_update.mapP(&squashSize, squash);
    //    }
  }
  // A = conf.model->response(y - y_smooth);
  A = conf.model->response(y);

  //   A_Hat =  conf.model->response(y + eta);
  //   eta +=  (A_Hat^T) * (A_Hat*eta - xsi) *-0.1/* (-epsA)*/ - eta * 0.01; //TEST
  //   eta += noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY);
  //   eta_buffer[(t-1)%buffersize] = eta; // Todo: work with the reference

};

/// calculate controller outputs
/// @param x_smooth smoothed sensors Matrix(number_channels,1)
Matrix DerPseudoSensor::calculateControllerValues(const Matrix& x_smooth){

 //  if(1 || satelliteTeaching){ ///TEST
//     // cout << "satteachiing" << endl;
//     const Matrix& ySat = conf.sat->process(x_smooth);
//     return ySat; }
//   else {
    return ((C* x_smooth+H).map(g)) * weighting + ( conf.sat->process(x_smooth) * ( 1.0 - weighting));
    // }
  // return (C* (x_smooth - x_smooth_long) +H).map(g); //TEST
  // return (C* x_smooth+H).map(g);
};


void DerPseudoSensor::getLastMotors(motor* motors, int len){
  const Matrix& y = y_buffer[t%buffersize];
  y.convertToBuffer(motors, len);
}

Matrix DerPseudoSensor::calcDerivatives(const matrix::Matrix* buffer,int delay){
  const Matrix& xt    = buffer[(t-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

void DerPseudoSensor::management(){
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
}



/** stores the controller values to a given file. */
bool DerPseudoSensor::store(FILE* f) const {
  // save matrix values
  C.store(f);
  H.store(f);
  if(conf.useS)  S.store(f);
  conf.model->store(f);
  Configurable::print(f,0);
  return true;
}

/** loads the controller values from a given file. */
bool DerPseudoSensor::restore(FILE* f){
  // save matrix values
  C.restore(f);
  H.restore(f);
  if(conf.useS)  S.restore(f);
  conf.model->restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}


void DerPseudoSensor::notifyOnChange(const paramkey& key){
  if(key == "epsA") {
    cout << "MODEL: " << conf.model->getParam("eps") << endl;
    conf.model->setParam("eps",epsA);
  }
}


list<Inspectable::iparamkey> DerPseudoSensor::getInternalParamNames() const {
  list<iparamkey> keylist;
  if(conf.someInternalParams){
    keylist += store4x4AndDiagonalFieldNames(A, "A");
    //  if(conf.useS) keylist += store4x4AndDiagonalFieldNames(S, "S");
    keylist += store4x4AndDiagonalFieldNames(C, "C");
         keylist += store4x4AndDiagonalFieldNames(R, "R");
//         keylist += store4x4AndDiagonalFieldNames(Q, "Q");

  }else{
    keylist += storeMatrixFieldNames(A, "A");
    //    if(conf.useS) keylist += storeMatrixFieldNames(S, "S");
    keylist += storeMatrixFieldNames(C, "C");
       keylist += storeMatrixFieldNames(R, "R");
//       keylist += storeMatrixFieldNames(Q, "Q");
  }
  //    keylist += storeMatrixFieldNames(y_teaching, "yteach");
  keylist += storeVectorFieldNames(H, "H");
  if(conf.sat)
          keylist += conf.sat->getInternalParamNames();
  //  keylist += storeVectorFieldNames(eta, "eta");
  // keylist += storeVectorFieldNames(xsi, "xsi");
  //  keylist += storeVectorFieldNames(x_smooth_long, "v_smooth");
  keylist += string("weighting");
  keylist += string("epsA");
  keylist += string("epsC");
  // keylist += string("xsi");
  //  keylist += string("xsi_norm");
  //  keylist += string("xsi_norm_avg");
  //   keylist += string("pain");
  // keylist += string("TimeLoopError");
  keylist += string("GrangerError1");
  keylist += string("GrangerError2");
  keylist += string("causalFactor");
  keylist += string("headPos_z");
  keylist += string("trunkPos_z");
  return keylist;
}

list<Inspectable::iparamval> DerPseudoSensor::getInternalParams() const {
  list<iparamval> l;
  if(conf.someInternalParams){
    l += store4x4AndDiagonal(A);
    //  if(conf.useS) l += store4x4AndDiagonal(S);
      l += store4x4AndDiagonal(C);
        l += store4x4AndDiagonal(R);
//        l += store4x4AndDiagonal(Q);

  }else{
    l += A.convertToList();
    //   if(conf.useS) l += S.convertToList();
    l += C.convertToList();
      l += R.convertToList();
//      l += Q.convertToList();
  }

  //l += y_teaching.convertToList();
   l += H.convertToList();
  if(conf.sat)
          l += conf.sat->getInternalParams();
  // l += eta.convertToList();
  //l += xsi.convertToList();
  //  l += x_smooth_long.convertToList();//TEST
  l += weighting;
  l += epsA;
  l += epsC;
  // l += xsi.elementSum();
//   l += xsi_norm;
//   l += xsi_norm_avg;
  // l += pain;
  // l += TLE;
  l += grang1;
  l += grang2;
  l += causalfactor;
  l += (double)headPosition.z;
  l += (double)trunkPosition.z;
  return l;
}

list<Inspectable::ILayer> DerPseudoSensor::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  l+=ILayer("x","", number_sensors, 0, "Sensors");
  l+=ILayer("y","H", number_motors, 1, "Motors");
  l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> DerPseudoSensor::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  return l;
}

//double clip095(double x){
// return clip(x,-0.95,0.95);
//}

void DerPseudoSensor::setMotorTeachingSignal(const motor* teaching, int len){
  assert(len == number_motors);
  y_teaching.set(len, 1, teaching);
  //  y_teaching.toMap(clip095); //TODO where is clip
  useTeaching=true;
}

void DerPseudoSensor::setSensorTeachingSignal(const sensor* teaching, int len){
  assert(len == number_sensors);
  Matrix x_teaching(len,1,teaching);
  // calculate the y_teaching, that belongs to the distal teaching value by the inverse model.
  // y_teaching = (A.multTM()^(-1)) *  ((A^T) * x_teaching) *0.000000000; //TEST
  //y_teaching.toMap(clip095); //TODO
  useTeaching=true;
}

double DerPseudoSensor::calcMatrixNorm(const Matrix& m){
  return m.map(fabs).elementSum() / (m.getM() * m.getN());
}
