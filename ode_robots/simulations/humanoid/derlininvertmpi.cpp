/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.2  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.1  2009/08/10 14:59:19  der
 *   copy of local controller version.
 *   new skeleton from bambi
 *
 *   Revision 1.6  2009/04/23 14:36:59  der
 *   bug fixing and simplifications
 *
 *   Revision 1.5  2009/04/22 13:30:54  der
 *   changing the averaging for the controller
 *
 *   Revision 1.4  2008/11/15 09:27:58  der
 *   small changes
 *
 *   Revision 1.3  2008/05/27 13:23:41  guettler
 *   multilayerffnn: inserted function for setting activation function of all layers
 *   from outside and back
 *
 *   Revision 1.2  2008/05/26 19:43:55  der
 *   Bugfixing Modelllernen mit A
 *
 *   Revision 1.1  2008/05/26 16:17:08  der
 *   Controller mit Inversion nur  der linearen Dynamik
 *
 *   Revision 1.9  2008/04/10 06:40:08  der
 *   intermediary changes
 *
 *   Revision 1.8  2008/03/26 09:24:49  der
 *   Mit neuer regularisierter Zweischritt Lernregel
 *
 *   Revision 1.7  2008/03/17 06:54:05  der
 *   Including analytical tools for Granger causality
 *
 *   Revision 1.6  2008/03/14 08:06:29  der
 *   With satellite controller network learning from C update
 *
 *   Revision 1.5  2008/02/15 07:09:11  der
 *   Several changes in the algorithm
 *
 *   Revision 1.4  2008/02/08 13:35:55  der
 *   *** empty log message ***
 *
 *   Revision 1.3  2008/02/08 11:34:45  der
 *   *** empty log message ***
 *
 *   Revision 1.2  2008/02/05 07:59:13  der
 *   Komprimierte Fassung
 *
 *   Revision 1.1  2007/12/06 16:50:49  der
 *   new versions
 *
 *
 *
 ***************************************************************************/

#include "derlininvertmpi.h"
//#include "dercontroller.h"
#include <selforg/regularisation.h>
#include <selforg/invertmotornstep.h>
//#include "invertmotorcontroller.h"
//#include "statistictools.h"
using namespace matrix;
using namespace std;

DerLinInvertMPI::DerLinInvertMPI( const DerLinInvertMPIConf& conf)
  : InvertMotorController(conf.buffersize, "DerLinInvertMPI", "$Id$"), conf(conf) {

  assert(conf.model != NULL);
  addConfigurable(conf.model);
//   fantControl = 50;
//   fantControlLen = 0;
//   fantReset = 5;

  YNoiseGen = 0;
  BNoiseGen = 0;
  noiseB = 0;
  //    zetaupdate=1.0;
  managementInterval=10;
  useTeaching=false;

  addParameterDef("dampS",&dampS,0);
  addParameterDef("dampC",&dampC,0);
  addParameterDef("modelcompl",&(this->conf.modelCompliant),0);
  addParameterDef("epsSat",&epsSat,0.1);
  addParameterDef("satT",&satelliteTeaching,1);

  addParameterDef("weighting",&weighting,1);
//   addParameterDef("PIDint",&PIDint,.03);
//   addParameterDef("PIDdrv",&PIDdrv,.2);
  //  addParameterDef("intstate",&intstate,1.0);
  //  addParameterDef("zetaupdate",&zetaupdate,.0);
};


DerLinInvertMPI::~DerLinInvertMPI(){
  if(x_buffer && y_buffer && eta_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] eta_buffer;
  }

  if(BNoiseGen) delete BNoiseGen;
  if(YNoiseGen) delete YNoiseGen;
}


void DerLinInvertMPI::init(int sensornumber, int motornumber, RandGen* randg){
  if (!randg)
    randg = new RandGen();
  number_motors  = motornumber;
  number_sensors = sensornumber;
  assert(number_motors && number_sensors);

  ID.set(number_motors, number_motors);
  ID.toId();
  ID_Sensor.set(number_sensors, number_sensors);
  ID_Sensor.toId();

    conf.model->init(number_motors, number_sensors, conf.modelInit);
     A.set(number_sensors,  number_motors); //TEST
     //     A0.set(number_sensors,  number_motors); //TEST
     A0.set(number_motors,  number_sensors); //TEST
      A=A^0;  //TEST
      A0=A0^0;


     //A = conf.model->response(Matrix(number_motors,1));
  //   A_Hat = conf.model->response(Matrix(number_motors,1));
   ATA_inv = (A.multTM()+ID*0.03)^-1;

  if (conf.useS) S.set(number_sensors, number_sensors*2); // S gets frist and second derivative

  if(conf.sat) conf.sat->init(number_sensors,number_motors, conf.cInit);

  C.set(number_motors,  number_sensors);
  GSC.set(number_motors,  number_sensors);

  // initialise the C matrix with identity + noise (-conf.cNonDiag, conf.cNonDiag) scaled to cInit value
  //C = ((C^0) + C.map(random_minusone_to_one) * conf.cNonDiag) * conf.cInit;
  C = (C^0)  * conf.cInit * -1.0;

  //  DD.set(number_sensors, number_sensors);
  // DD.toId(); DD *= 0.1; // noise strength estimate
  // Dinverse.set(number_sensors, number_sensors);
  // Dinverse = DD^-1;
  //  eta_smooth.set(number_motors,1);

  B.set(number_sensors,  1);
  H.set(number_sensors,  1);//TEST H
  R.set(number_sensors, number_sensors);
  RG.set(number_sensors, number_sensors);
  //  Q.set(number_sensors, number_motors);
  //  Q1.set(number_sensors, number_motors);
  Q.set(number_motors, number_sensors);
  Q = (Q^0)*.1;
  Q1.set(number_motors, number_sensors);
  Q1 = (Q1^0)*.1;
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
  y_sat.set(number_motors, 1);
  y_smooth_long.set(number_motors, 1);
  y_forecast.set(number_motors, 1);
  y_integration.set(number_motors, 1);
  PID_deriv.set(number_motors, 1);

  t_rand = rand()%(RAND_MAX/managementInterval);
  initialised = true;
}
//*************** End init *******************

/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void DerLinInvertMPI::step(const sensor* x_, int number_sensors,
                           motor* y_, int number_motors){
  fillBuffersAndControl(x_, number_sensors, y_, number_motors);
  if(t>buffersize){
    int delay = max(int(s4delay)-1,0);
    // learn Model with actual sensors and with effective motors,
    // calc xsi and A;
    //  learnModel(delay);
    // learn controller with effective input/output
    learnController(delay);

  }
  // update step counter
  t++;

};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void DerLinInvertMPI::stepNoLearning(const sensor* x, int number_sensors,
                                     motor*  y, int number_motors ){
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
  cout << y << endl;
};

void DerLinInvertMPI::fillBuffersAndControl(const sensor* x_, int number_sensors,
                                            motor* y_, int number_motors){
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);
  Matrix x(number_sensors,1,x_);
  //********Fehlerkorrektur ************

  // for ( int i=0; i<number_sensors && i<number_motors; i++)

//      if ( fabs(x.val(i,0) -  y_buffer[(t + buffersize)%buffersize].val(i,0)) > 0.05 ) x.val(i,0) = y_buffer[(t + buffersize)%buffersize].val(i,0)*.98 ;


  //********Fehlerkorrektur Ende ************

    x_smooth_long += ( x - x_smooth_long ) * ((0.03 * 1.0)/(double)s4avg);

  //  x -=   x_smooth_long;


  // averaging over the last s4avg values of x_buffer
    x_smooth += (x - x_smooth)*(1.0/(double)s4avg);//calculateSmoothValues;

    //    x -= x_smooth_long; ////////////////////***********///////////////////////
    putInBuffer(x_buffer, x);

  // calculate controller values
    //  Matrix y = y_buffer[(t -1 + buffersize)%buffersize];
//         y += (calculateControllerValues(x) +  noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY) - y)*(1.0/(double)s4avg);
   Matrix y = calculateControllerValues(x_smooth);
    // Add noise
    //  y += noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY);

  y_smooth += (y - y_smooth)* (1.0/(double)s4avg);

  //  y_smooth.convertToBuffer(y_, number_motors);
    y.convertToBuffer(y_, number_motors);


    // putInBuffer(y_buffer, y_smooth);
  putInBuffer(y_buffer, y);

//   ///////Global PID controller

//   y_integration += (A^T) * ( x_smooth - A * y_smooth) *(-.1) - y_integration * PIDint;
//   PID_deriv += ((A0^T) * ( (  x_buffer[(t + buffersize)%buffersize] - x_buffer[(t -1 + buffersize)%buffersize] )

//  - A0 * (  y_buffer[(t + buffersize)%buffersize] - y_buffer[(t -1 + buffersize)%buffersize] ))


//                 -PID_deriv)* 0.03;

//     ((y_smooth + (y_integration) - PID_deriv*PIDdrv ).map(g)).convertToBuffer(y_, number_motors);
//   // convert y to motor*; y is sent to the motors.



//   ///////Global PID controller End


  // convert y to motor*; y is sent to the motors.



}
/* learn values H,C

*/
void DerLinInvertMPI::learnController(int delay){

  Matrix C_update(C.getM(), C.getN());
  Matrix H_update(H.getM(), H.getN());

  bool teaching = (conf.modelCompliant!=0) || useTeaching;
  Matrix C_updateTeaching;
  Matrix H_updateTeaching;

  if(teaching){
    C_updateTeaching.set(C.getM(), C.getN());
    H_updateTeaching.set(H.getM(), H.getN());
  }



  const Matrix& x = x_buffer[(t -1/*????????????????*/ + buffersize)%buffersize];//
  const Matrix& y = y_buffer[(t-delay)%buffersize];
  y_sat = conf.sat->process(x_smooth);
           // y_smooth_long += ( y - y_smooth_long)*.0003;// s4avg;


  causalfactor = 1; //TEST

  //    cout << A<<endl;
  // xsi = x.map(g) - A* y - B;
  // xsi = x - A* y - B;//( y * weighting + y_sat * (1 - weighting ));// - v_smooth;
  xsi = x - B -  A* y;//( calculateControllerValues(x) * weighting + y_sat * (1 - weighting ));// - v_smooth;

   //  xsi = x_smooth  -  x_smooth_long -  A* ( y * weighting + y_sat * (1 - weighting ));// - v_smooth;
  A += xsi * (y^T) * epsA;

  //<<<<<<< derlininvert.cpp
  // B += xsi * epsA * factorB - B * epsA*.02* factorB;
  // =======
  B += xsi * epsA * factorB - B * factorB * epsA*.02;
  // >>>>>>> 1.6

// (( y * weighting + y_sat * (1 - weighting ))^T)*epsA;
  // A += (x_smooth - A * ( y_smooth + y_smooth_long *-1))  * (( ( y_smooth + y_smooth_long*-1) * weighting + y_sat * (1 - weighting ))^T)*epsA; //TEST

// <<<<<<< derlininvert.cpp
//   A += ((A^0)-A) * epsA*dampA;
// //   B += ( xsi - B ) * .001*factorB;
//     xsi -= B;

//   if ((t%50)==2)
//     ATA_inv = (A.multTM() + ID*0.1)^-1;
//   //    cout <<A.val(0,0) << " " << xsi.val(0,0) << endl;
//     eta = ATA_inv * (A^T) * xsi;
// =======
  A -= (A - (A^0)) * epsA*dampA;
    B += ( xsi - B ) * .001*factorB;

 //   eta = y_buffer[(t + buffersize)%buffersize] - A0 * x - Q * x_buffer[(t -20 + buffersize)%buffersize] ;

  eta =  Q1 * x_buffer[(t -40 + buffersize)%buffersize] + Q * x_buffer[(t -20 + buffersize)%buffersize] + A0 * x -  y_buffer[(t + buffersize)%buffersize];
  A0 -= eta * (x^T) * epsA;
  Q -= eta * ( x_buffer[(t -20 + buffersize)%buffersize] ^T) * epsA ;
  Q -= Q*.0003;
  Q1 -= eta * ( x_buffer[(t -40 + buffersize)%buffersize] ^T) * epsA ;
  Q1 -= Q1*.0003;

  A0 -= (A0 - (A0^0)) * epsA*dampA;


  eta +=  noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY);

//   if ((t%50)==2)
//     ATA_inv = (A.multTM() + ID*0.1)^-1;
//   //    cout <<A.val(0,0) << " " << xsi.val(0,0) << endl;
 //     eta = ATA_inv * (A^T) * xsi;
//      eta = eta.map(g);

     //    const Matrix& eta0 = eta.map(g);
//>>>>>>> 1.6
  //  noise for the null space:
  // eta += noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY);
  eta_buffer[(t-1)%buffersize] = eta;

  //Mit Differenzen:

  // eta -= eta_buffer[(t-2)%buffersize];

    assert( steps + delay < buffersize);


   //+++++++==Lernen mit Inversion nur der linearen Dynamik+++++++++


  //The learning step
  if (steps > 2) {
    cout << "steps must be 1 or 2" << endl;
    steps =2 ;
  }
// <<<<<<< derlininvert.cpp
//   GSC=GSC^0;
//   double nm;
//  nm =  calcMatrixNorm(x_buffer[(t + buffersize)%buffersize] - x_buffer[(t -1 + buffersize)%buffersize]);
// =======
  double nm;
 nm =  calcMatrixNorm(x_buffer[(t + buffersize)%buffersize] - x_buffer[(t -1 + buffersize)%buffersize]);
// >>>>>>> 1.6

//    const Matrix& y          = y_buffer[(t-s-delay)%buffersize];
//  const Matrix& z          = (C * x_smooth + H);
// <<<<<<< derlininvert.cpp
//  const Matrix& z   = (C * x/*_smooth*/ + C * H);
// =======

 const Matrix& z          = (C * x_smooth + C * H);
//  const Matrix& z          = (C * x/*_smooth*/ + C * H)* intstate + y_smooth*(1-intstate);//TEST innerer Zustand
// >>>>>>> 1.6
  const Matrix g_prime = z.map(g_derivative);

// <<<<<<< derlininvert.cpp
//   CST = C.multrowwise(g_prime);
//   if ( nm < .25 ) { //TEST may cause problems under noise!!!!!!!!!
// =======
  //  CST = C.multrowwise(g_prime);
  if ( nm < .15 )
    { //TEST may cause problems under noise!!!!!!!!!
      // >>>>>>> 1.6
  if(steps == 1) {
    double alpha = 1.0;//1.0/((double)s4avg);//<- sonst divergenz!!!!!!!!!!!

    eta +=  z.map(g) + z * (-1);

    // CCT_inv = (( C * (C^T)).pluslambdaI(0.01))^(-1)3
    CCT_inv = (( (C*alpha + (C^0)*(1-alpha)) * ( (C*alpha + (C^0)*(1-alpha))^T)).pluslambdaI(0.01))^(-1);
  const Matrix mue = CCT_inv*eta;
  const Matrix vau = (C^T) * mue;
  // const Matrix vau = ( (C*alpha + (C^0)*(1-alpha))^T) * mue;
  double EE;
  EE =  /*sqrt*/(((vau^T)*vau).val(0,0)) + .0001;
  //  EE = sqrt( 1/EE);// Mist
  TLE = EE;


  // Mit EE besseres Lernen, wirkt praktisch wie eine Lernratenkorrektur.
  C_update = ( mue * (mue^T )*(C*alpha + (C^0)*(1-alpha)) *1 /*TEST*/ +( mue.multrowwise(g_prime * -1) +  mue ) *  ((x/*_smooth*/ + H)^T) /*** desens !!!!**/)*epsC *(1/EE); //TEST EE,

  H_update = (C^T) *  (   mue.multrowwise(g_prime *-1)  + mue ) * epsC *(1/EE);

  ////////////Feed-forward learning step Begin

//   C_update += (eta0.multrowwise(g_prime)) * ((H+x)^T) * zetaupdate;
//   H_update += (C^T) * (eta0.multrowwise(g_prime)) *zetaupdate;//eta0;
//  C_update +=((((A^0)^T)* xsi).multrowwise(g_prime)) * (x^T) * zetaupdate;
//   H_update += (C^T) * ((((A^0)^T)* xsi).multrowwise(g_prime) ) * xsi.multrowwise(g_prime) *zetaupdate;//eta0;


  ////////////////Feed-forward learning step End

    }
  if (steps == 2) {
    cout << "sorry, steps is put equal to 1" << endl;
    steps =1;
  }

 }

  //+++++++==Lernen mit Inversion nur der linearen Dynamik Ende ++++++++++

  //Hier Teaching
  //conf.sat->learn(x_smooth,y*.98,epsSat);
 conf.sat->process(x_buffer[(t-delay-10)%buffersize]);
// <<<<<<< derlininvert.cpp
//  conf.sat->learn(x_buffer[(t-delay-30)%buffersize],y_buffer[(t - 30)%buffersize]*.99,epsSat);//*causalfactor);//learning with the current causalfactor
//    const Matrix delta = ( y_sat*.95 - y ).multrowwise(g_prime);
// =======
 conf.sat->learn(x_buffer[(t-delay-10)%buffersize],y_buffer[(t - 10)%buffersize]*.99,epsSat);//*causalfactor);//learning with the current causalfactor
   const Matrix delta = ( y_sat*.95 - y ).multrowwise(g_prime);
   //>>>>>>> 1.6
  // const Matrix delta = ( x - y ).multrowwise(g_prime); //TEST
   C_update += delta * (x^T)*teacher;//*causalfactor ;// *epsC;
  //  H_update += delta * teacher*causalfactor;// *epsC;
   H_update += (C^T) * delta  * teacher;//*causalfactor;// *epsC; TEST H ??????????
 // const  Matrix yy = conf.sat->process(x_smooth);//TEST ?????????????????????????????
//   conf.sat->learn(x_smooth,yy - mue,epsSat);//TEST



  // Controlling the learning parameters:
// <<<<<<< derlininvert.cpp
//   double Test_squashSize = squashSize/5.0;
//  //  double u = calcMatrixNorm(C_update);  //TEST
// //   double q = calcMatrixNorm( C_update.mapP(&Test_squashSize, squash) ); //TEST
// =======
//   double Test_squashSize = squashSize/5.0;
//   double u = calcMatrixNorm(C_update);  //TEST
//   double q = calcMatrixNorm( C_update.mapP(&Test_squashSize, squash) ); //TEST
//>>>>>>> 1.6
  //    double Au = calcMatrixNorm(A_update);  //TEST
//   double Aq = calcMatrixNorm( A_update.mapP(&squashSize, squash) ); //TEST
//   if (epsC>0)  epsC *= 1.001;
//   if ( fabs( u) > 1.01 * fabs( q) )      epsC *= 0.9;

  //  else {
   C += C_update.mapP(&squashSize, squash);
   H += H_update.mapP(&squashSize, squash);
  if (epsC > 0) {
// <<<<<<< derlininvert.cpp
//     C -= (C - (C^0)) * dampC;
//     H -= H * dampC;

// =======

    C -= (C - (C^0)) * dampC;
    H -= H * dampC;

//>>>>>>> 1.6
}


}
 //   // End  Controlling the learning parameters:


  //  }




// learns model and calculates Xsi and A  and learns the model
//New: calculates also eta
void DerLinInvertMPI::learnModel(int delay){
  const Matrix& x = x_buffer[(t-1)  % buffersize];
  const Matrix& y = y_buffer[(t - 2 - delay) % buffersize];//TEST
  // y_smooth += ( y - y_smooth)*.001;// s4avg;
  //  xsi = x -  conf.model->process(y);
  //  xsi = x- x_smooth_long -  conf.model->process(y-y_smooth);
  // xsi = x -  conf.model->process(y);
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
  //  A = conf.model->response(y);

  //   A_Hat =  conf.model->response(y + eta);
  //   eta +=  (A_Hat^T) * (A_Hat*eta - xsi) *-0.1/* (-epsA)*/ - eta * 0.01; //TEST
  //   eta += noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY);
  //   eta_buffer[(t-1)%buffersize] = eta; // Todo: work with the reference

};

/// calculate controller outputs
/// @param x_smooth smoothed sensors Matrix(number_channels,1)
Matrix DerLinInvertMPI::calculateControllerValues(const Matrix& x){

 //  if(1 || satelliteTeaching){ ///TEST
//     // cout << "satteachiing" << endl;
//     const Matrix& ySat = conf.sat->process(x_smooth);
//     return ySat; }
//   else {
  //  return ((C* x_smooth+H).map(g)) * weighting + ( conf.sat->process(x_smooth) * ( 1.0 - weighting));
   return (C* ( x +   H )).map(g)  * weighting + ( y_sat * ( 1.0 - weighting));
   // return ((C* ( x +   H )) *intstate  + y_smooth*(1-intstate) ).map(g)  * weighting + ( y_sat * ( 1.0 - weighting));//TEST Innerer Zustand
    // }
  // return (C* (x_smooth - x_smooth_long) +H).map(g); //TEST
  // return (C* x_smooth+H).map(g);
};


void DerLinInvertMPI::getLastMotors(motor* motors, int len){
  const Matrix& y = y_buffer[(t-1)%buffersize];
  y.convertToBuffer(motors, len);
}

Matrix DerLinInvertMPI::calcDerivatives(const matrix::Matrix* buffer,int delay){
  const Matrix& xt    = buffer[(t-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

void DerLinInvertMPI::management(){
  if(dampA){
    conf.model->damp(dampA * managementInterval);
  }
  if(dampS){
    S -= S*(dampS * managementInterval);
  }
  if(dampC){
    C -= (C - (C^0))*(dampC * managementInterval);
    H -= H*(dampC * managementInterval);
  }
}



/** stores the controller values to a given file. */
bool DerLinInvertMPI::store(FILE* f) const {
  // save matrix values
  C.store(f);
  H.store(f);
  if(conf.useS)  S.store(f);
  conf.model->store(f);
  Configurable::print(f,0);
  return true;
}

/** loads the controller values from a given file. */
bool DerLinInvertMPI::restore(FILE* f){
  // save matrix values
  C.restore(f);
  H.restore(f);
  if(conf.useS)  S.restore(f);
  conf.model->restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}


list<Inspectable::iparamkey> DerLinInvertMPI::getInternalParamNames() const {
  list<iparamkey> keylist;
  if(conf.someInternalParams){
    keylist += store4x4AndDiagonalFieldNames(A, "A");
    //  if(conf.useS) keylist += store4x4AndDiagonalFieldNames(S, "S");
    keylist += store4x4AndDiagonalFieldNames(C, "C");
    //     keylist += store4x4AndDiagonalFieldNames(R, "R");
//         keylist += store4x4AndDiagonalFieldNames(Q, "Q");

  }else{
    keylist += storeMatrixFieldNames(A, "A");
    //    if(conf.useS) keylist += storeMatrixFieldNames(S, "S");
    keylist += storeMatrixFieldNames(C, "C");
    //    keylist += storeMatrixFieldNames(R, "R");
//       keylist += storeMatrixFieldNames(Q, "Q");
  }
  // keylist += storeVectorFieldNames(y_integration, "y_integration");
  //  keylist += storeVectorFieldNames(PID_deriv, "PID_deriv");
  keylist += storeVectorFieldNames(H, "H");
 //  if(conf.sat)
//             keylist += conf.sat->getInternalParamNames();
  //  keylist += storeVectorFieldNames(eta, "eta");
  // keylist += storeVectorFieldNames(xsi, "xsi");
  //  keylist += storeVectorFieldNames(x_smooth_long, "v_smooth");
  keylist += string("weighting");
  keylist += string("epsA");
  keylist += string("epsC");
  // keylist += string("xsi");
   keylist += string("xsi_norm");
  //  keylist += string("xsi_norm_avg");
  keylist += string("PIDint");
   keylist += string("TimeLoopError");
  keylist += string("GrangerError1");
  keylist += string("GrangerError2");
  keylist += string("GrangerCausality");
  return keylist;
}

list<Inspectable::iparamval> DerLinInvertMPI::getInternalParams() const {
  list<iparamval> l;
  if(conf.someInternalParams){
    l += store4x4AndDiagonal(A);
    //  if(conf.useS) l += store4x4AndDiagonal(S);
      l += store4x4AndDiagonal(C);
      //       l += store4x4AndDiagonal(R);
//        l += store4x4AndDiagonal(Q);

  }else{
    l += A.convertToList();
    //   if(conf.useS) l += S.convertToList();
    l += C.convertToList();
    //      l += R.convertToList();
//      l += Q.convertToList();
  }

 //  l += y_integration.convertToList();
//   l += PID_deriv.convertToList();
  //l += y_teaching.convertToList();
   l += H.convertToList();
   //   l += xsi.convertToList();
  // l += x_buffer[(t - 1 + buffersize )%buffersize].convertToList();
//   if(conf.sat)
//           l += conf.sat->getInternalParams();
  // l += eta.convertToList();
  //l += xsi.convertToList();
  //  l += x_smooth_long.convertToList();//TEST
  l += weighting;
  l += epsA;
  l += epsC;
  // l += xsi.elementSum();
  l += TLE;//xsi_norm;
//   l += xsi_norm_avg;
  l += PIDint;
   l += TLE;
  l += grang1;
  l += grang2;
  l += causalfactor;
  return l;
}

list<Inspectable::ILayer> DerLinInvertMPI::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  l+=ILayer("x","", number_sensors, 0, "Sensors");
  l+=ILayer("y","H", number_motors, 1, "Motors");
  l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> DerLinInvertMPI::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  return l;
}

//double clip095(double x){
// return clip(x,-0.95,0.95);
//}

void DerLinInvertMPI::setMotorTeachingSignal(const motor* teaching, int len){
  assert(len == number_motors);
  y_teaching.set(len, 1, teaching);
  //  y_teaching.toMap(clip095); //TODO where is clip
  useTeaching=true;
}

void DerLinInvertMPI::setSensorTeachingSignal(const sensor* teaching, int len){
  assert(len == number_sensors);
  Matrix x_teaching(len,1,teaching);
  // calculate the y_teaching, that belongs to the distal teaching value by the inverse model.
  // y_teaching = (A.multTM()^(-1)) *  ((A^T) * x_teaching) *0.000000000; //TEST
  //y_teaching.toMap(clip095); //TODO
  useTeaching=true;
}

double DerLinInvertMPI::calcMatrixNorm(const Matrix& m){
  return m.map(fabs).elementSum() / (m.getM() * m.getN());
}
