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

#include "derinf.h"
//#include "dercontroller.h"
#include "regularisation.h"
#include "invertmotornstep.h"
//#include "invertmotorcontroller.h"
//#include "statistictools.h"
using namespace matrix;
using namespace std;

DerInf::DerInf( const DerInfConf& conf)
  : InvertMotorController(conf.buffersize, "DerInf", "$Id$"), conf(conf) {

//   fantControl = 50;
//   fantControlLen = 0;
//   fantReset = 5;

  YNoiseGen = 0;
  BNoiseGen = 0;
  noiseB = 0;
      zetaupdate=.05;
  managementInterval=1;
  useTeaching=false;

  addParameterDef("dampS",&dampS,0.01);
  addParameterDef("dampC",&dampC,0.0001);
 //  addParameterDef("modelcompl",&(this->conf.modelCompliant),0);
//   addParameterDef("epsSat",&epsSat,0.1);
//   //  addParameterDef("satT",&satelliteTeaching,1);

//    addParameterDef("weighting",&weighting,.01);
//   addParameterDef("gamma",&gamma,.1);
//   addParameterDef("PIDint",&PIDint,.03);
//   addParameterDef("PIDdrv",&PIDdrv,.2);
//   addParameterDef("intstate",&intstate,1.0);
  addParameterDef("zetaupdate",&zetaupdate,.03);
  addParameterDef("adaptrate",&adaptrate,1.8);
  addParameterDef("xsifactor",&xsifactor,1.1);
  addParameterDef("creat",&creat,.0);
  addParameterDef("num_iterations",&num_iterations,4);
  addParameterDef("sense",&sense,1.5);
};


DerInf::~DerInf(){
  if(x_buffer && y_buffer && eta_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] eta_buffer;
  }

  if(BNoiseGen) delete BNoiseGen;
  if(YNoiseGen) delete YNoiseGen;
}


void DerInf::init(int sensornumber, int motornumber, RandGen* randg){
  if (!randg)
    randg = new RandGen();
  number_motors  = motornumber;
  number_sensors = sensornumber;
  assert(number_motors && number_sensors);
  ID.set(number_motors, number_motors);
  ID.toId();
  ID_Sensor.set(number_sensors, number_sensors);
  ID_Sensor.toId();

  A.set(number_sensors,  number_motors); //TEST
  //     A0.set(number_sensors,  number_motors); //TEST
  A0.set(number_motors,  number_sensors); //TEST
  A=A^0;  //TEST
  A0=A0^0;

    ATA_inv = (A.multTM()+ID*0.1)^-1;//?????????????????????????????

    if (conf.useS){
      S.set(number_sensors, number_sensors); // S gets previous sensor values
      S = (S^0)*.3;
    }
  C.set(number_motors,  number_sensors);
  GSC.set(number_motors,  number_sensors);

  // initialise the C matrix with identity + noise (-conf.cNonDiag, conf.cNonDiag) scaled to cInit value
  //C = ((C^0) + C.map(random_minusone_to_one) * conf.cNonDiag) * conf.cInit;
  C = (C^0)  * conf.cInit;

   DD.set(number_motors, number_motors);
  DD.toId(); DD *= 0.001; // noise strength estimate
  G.set(number_motors, number_motors);
     G *= 0;
  L.set(number_sensors, number_sensors);
//   Dinverse.set(number_sensors, number_sensors);
//   Dinverse = DD^-1;
  //  eta_smooth.set(number_motors,1);

  B.set(number_sensors,  1);
  H.set(number_sensors,  1);//TEST H
  HY.set(number_motors,  1);
  R.set(number_sensors, number_sensors);
  RG.set(number_sensors, number_sensors);
  Q.set(number_motors, number_motors);
  // Q = (Q^0)*.1;
  Q1.set(number_motors, number_motors);
  Q1 = (Q1^0)*.1;
  CCT_inv.set(number_motors, number_motors);
  CST.set(number_motors, number_sensors);
  //  R=C*A;
  //   RRT_inv = (R +  ID * 0.2)^-1;

  C_updateOld.set(number_motors,  number_sensors);
  H_updateOld.set(number_sensors,1);
  HY_updateOld.set(number_motors, 1);
  A_updateOld.set(number_sensors,  number_motors);
  B_updateOld.set(number_sensors, 1);

  squashSize = .05;

  xsi.set(number_sensors,1);
  xx.set(number_sensors,1);
  yy.set(number_motors,1);
  zz.set(number_motors,1);
  x_intern.set(number_sensors,1);
  y_intern.set(number_motors,1);
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
  xsi_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    xsi_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
    eta_buffer[k].set(number_motors,1);
  }
  y_teaching.set(number_motors, 1);
  x_smooth.set(number_sensors,1);
  x_smooth_long.set(number_sensors,1);

  zero_eta.set(number_motors, 1);
  eta.set(number_motors, 1);
  ups.set(number_motors, 1);
  v_smooth.set(number_motors, 1);
  y_smooth.set(number_motors, 1);
  y_sat.set(number_motors, 1);
  y_smooth_long.set(number_motors, 1);
  y_forecast.set(number_motors, 1);
  y_integration.set(number_motors, 1);
  y_error.set(number_motors, 1);
  PID_deriv.set(number_motors, 1);
  vau_avg.set(number_sensors, 1);

  t_rand = rand()%(RAND_MAX/managementInterval);
  initialised = true;
}
//*************** End init *******************

/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void DerInf::step(const sensor* x_, int number_sensors,
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
void DerInf::stepNoLearning(const sensor* x, int number_sensors,
                                     motor*  y, int number_motors ){
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
  //  cout << y << endl;
};

void DerInf::fillBuffersAndControl(const sensor* x_, int number_sensors,
                                            motor* y_, int number_motors){
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);
  Matrix x(number_sensors,1,x_);

  //   x = (((x*1.3).map(g))*(1/1.3))*(1/sqrt(xsifactor));
       x = (((x*1.3).map(g))*(1/1.3));

  //  x += H; //TEST H
//  if ( t > 60) x = x_intern;//TEST
    x_smooth_long += ( x - x_smooth_long ) * ((0.03 * 3.0)/(double)s4avg);
 //  x -=   x_smooth_long;

  // averaging over the last s4avg values of x_buffer
    x_smooth += (x - x_smooth)*(1.0/(double)s4avg);//calculateSmoothValues;

    putInBuffer(x_buffer, x);
    //  x_buffer[(t + buffersize)%buffersize] liefert das x zurueck.


    /////////Normalverfahren
    //    calculate controller values
  //    Matrix y = y_buffer[(t -1 + buffersize)%buffersize];
//         y += (calculateControllerValues(x) +  noiseMatrix(eta.getM(),eta.getN(), *YNoiseGen, -noiseY, noiseY) - y)*(1.0/(double)s4avg);

    Matrix y = calculateControllerValues(x/*TEST*/);// +  vau_avg.map(g)*creat);//_buffer[(t - t_delay  + buffersize)%buffersize]);
  //  x_intern=A*y;

   y_smooth += (y - y_smooth)* (1.0/(double)s4avg);

   //  y_smooth.convertToBuffer(y_, number_motors);

    putInBuffer(y_buffer, y);

    //   y = ((y + y_integration)*1.2).map(g)*(1/1.2);

   y.convertToBuffer(y_, number_motors);
  // convert y to motor*; y is sent to the motors.

}
void DerInf::learnController(int delay){

  Matrix C_update(C.getM(), C.getN());
  Matrix H_update(H.getM(), H.getN());
  Matrix HY_update(H.getM(), H.getN());

  int sensornumber = C.getN();

  if (!conf.useS) S *= 0;
    const Matrix& x = x_buffer[(t + buffersize)%buffersize];//

    //    t_delay = 1;

   const Matrix& x_delay = x_buffer[(t - delay + buffersize)%buffersize];

   //            cout << x-x_delay <<endl;

   // causalfactor = 1; //TEST

//Iteration Beginn
    xx =  (  x_buffer[(t - num_iterations + buffersize)%buffersize]);
    for (int i=0; i < num_iterations; i++){
      if(!conf.useS){
        xx = A * ((C*xx + C*H + HY).map(g))+B;
      }
      else {
        xx = A * ((C*xx + C*H + HY).map(g))+B + S*xx;
      }
    }
//Iteration Ende

    zz = C*xx + C*H + HY;
    yy = zz.map(g);
    //    if(t%100==1) cout<< 20 <<endl ;
    const Matrix& z   = C * x_delay  + C * H + HY;
  const Matrix& y = z.map(g);
  const Matrix g_prime = zz.map(g_derivative);
  const Matrix g_prime_inv = g_prime.map(one_over);

    xsi = x  - B -  A* y;
    if ( conf.useS ) xsi = x  - B -  A * y - S*x_delay;


     A += xsi * ((y + eta*.000000001)^T) * epsA - A*dampA*epsA;
     B += xsi * epsA * factorB - B * factorB * epsA*.02;
     if ( conf.useS )  S += xsi*(x_delay^T)*epsA - S * epsA*dampS;

     for ( int i = 0; i<number_motors; i++) {
       G.val(i,i) = g_prime.val(i,0);
     }

     putInBuffer(xsi_buffer,xsi);

     // C += (G * (A^T )*xsi*(x^T))*teacher;
     // HY += (G * (A^T )*xsi)*teacher;

//      L = A * G * C;
//      if ( conf.useS ) L += S;
    //  eta =  ((A^T) *(
//                       L*L* xsi_buffer[(t-2+buffersize)%buffersize]
//                      + L* xsi_buffer[(t-1+buffersize)%buffersize]
//                      + xsi
//                      )).multrowwise(g_prime);
     eta = G*(A^T)*( x - xx );
     const Matrix xsi1=  G*(A^T)*xsi;
     DD += ( eta * (eta^T) - xsi1*(xsi1^T)- DD)*zetaupdate;

//      RG = RG.map(random_minusone_to_one)*.03;
//      RG += RG^T;
//      //     DD =      (A^T)*(((G^0)*xsifactor - L * (L^T) )^(-1))*A;
//      DD = G*(A^T)*
//        ((RG + ((G^0)*xsifactor -  L * (L^T) )*((G^0)*xsifactor -  L * (L^T) ))
//         ^(-1))*A*G;
//     DD = G*(A^T)*((RG + ((ID_Sensor^0) -  L * (L^T) ))^(-1))* A * G; ;
     //    DD = DD  - xsi1*(xsi1^T);
  xsistrength += ((( (xsi^T)*xsi).val(0,0))/sensornumber - xsistrength )*.03;
  //  double Dfactor = 1/(xsistrength + .00001);
  //  Dinverse = (( ((DD*Dfactor)^-1)*.03 ).map(tanh))*3;
  ////   cout << DD.val(0,0)*Dfactor << "Dfactor"<<Dfactor<< endl;



     //  xx = x; yy = y;
    for (int i=0; i<number_motors; i++)  {
     ups.val(i,0) = (DD * C * (C^T)).val(i,i);
}

    EE = .3/(((eta^T)*eta).val(0,0)+.0000000001);
    //  EE = 1.0/(((ups^T)*ups).val(0,0)+.0000000001);//TEST
    //  EE = sqrt(EE);
    //    EE *= 1+sin(t/80);


    ups = ups.multrowwise(yy)*-2*sense;

    //Lernen:

    C_update = ((DD * C + ups * (xx^T)))*epsC*EE;

    HY_update =  (ups/*.map(g)*/) *(epsC)*EE - yy * creat;

    //   C_update = ((DD * C + (DD * C * (C^T) ) * y * (xx^T)*(-2 * xsifactor)).map(g))*epsC*EE;

 //    for (int k=0; k<number_motors;k++) for (int l=0; l<number_sensors;l++) {

//          if(k-l>1 || l-k>1) C.val(k,l)=0;
//     }

    //  HY_update += x*-.003;//TEST
 if (epsC > 0) {
    C -= (C - (C^0))/* *(C - (C^0)) *(C - (C^0)) */ * dampC;
    H -= (H & H & H) * dampC;
    HY -= (HY & HY & HY) * .00000001;


//     C_update = C_update * gamma + C_updateOld * ( 1 - gamma );
//     H_update = H_update * gamma + H_updateOld * ( 1 - gamma );
//     HY_update = HY_update * gamma + HY_updateOld * ( 1 - gamma );

//     C_updateOld = C_update;
//     H_updateOld = H_update;
//     HY_updateOld = HY_update;


    C += (C_update).map(g)*.01;//weighting;
    //  H += H_update;
    HY += (HY_update).map(g)*.01;//weighting*2;
//     //    epsC=epsC_old;

//      H*=0;
//       HY*=0;



}


}
 //   // End  Controlling the learning parameters:


  //  }


/// calculate controller outputs
/// @param x_smooth smoothed sensors Matrix(number_channels,1)

Matrix DerInf::calculateControllerValues(const Matrix& x){
  return (C*x + HY).map(g);
};


void DerInf::getLastMotors(motor* motors, int len){
  const Matrix& y = y_buffer[(t-1)%buffersize];
  y.convertToBuffer(motors, len);
}

Matrix DerInf::calcDerivatives(const matrix::Matrix* buffer,int delay){
  const Matrix& xt    = buffer[(t-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

void DerInf::management(){
}



/** stores the controller values to a given file. */
bool DerInf::store(FILE* f) const {
  // save matrix values
  C.store(f);
  H.store(f);
  HY.store(f);
  A.store(f);
  if(conf.useS)  S.store(f);
  Configurable::print(f,0);
  return true;
}

/** loads the controller values from a given file. */
bool DerInf::restore(FILE* f){
  // save matrix values
  C.restore(f);
  H.restore(f);
  HY.restore(f);
  A.restore(f);
  if(conf.useS)  S.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}



list<Inspectable::iparamkey> DerInf::getInternalParamNames() const {
  list<iparamkey> keylist;
  if(conf.someInternalParams){
    keylist += store4x4AndDiagonalFieldNames(A, "A");
    if(conf.useS) keylist += store4x4AndDiagonalFieldNames(S, "S");
    keylist += store4x4AndDiagonalFieldNames(C, "C");
   //            keylist += store4x4AndDiagonalFieldNames(S, "S");
//         keylist += store4x4AndDiagonalFieldNames(Q, "Q");

  }else{
    keylist += storeMatrixFieldNames(A, "A");
      if(conf.useS) keylist += storeMatrixFieldNames(S, "S");
    keylist += storeMatrixFieldNames(C, "C");
    //    keylist += storeMatrixFieldNames(R, "R");
//       keylist += storeMatrixFieldNames(Q, "Q");
  }
  //   keylist += storeVectorFieldNames(y_integration, "y_integration");
  //  keylist += storeVectorFieldNames(PID_deriv, "PID_deriv");
  keylist += storeVectorFieldNames(HY, "HY");
  //  keylist += storeVectorFieldNames(vau_avg, "vau_avg");
  keylist += storeVectorFieldNames(xx, "xx");
 //  if(conf.sat)
//             keylist += conf.sat->getInternalParamNames();
  //  keylist += storeVectorFieldNames(eta, "eta");
  // keylist += storeVectorFieldNames(xsi, "xsi");
  //  keylist += storeVectorFieldNames(x_smooth_long, "v_smooth");
//   keylist += string("weighting");
//   keylist += string("epsA");
//   keylist += string("epsC");
//   // keylist += string("xsi");
//    //  keylist += string("xsi_norm");
//   //  keylist += string("xsi_norm_avg");
//   keylist += string("PIDint");
//    keylist += string("TimeLoopError");
//   keylist += string("EE");
//   keylist += string("GrangerError2");
//   keylist += string("GrangerCausality");
  return keylist;
}

list<Inspectable::iparamval> DerInf::getInternalParams() const {
  list<iparamval> l;
  if(conf.someInternalParams){
    l += store4x4AndDiagonal(A);
   if(conf.useS) l += store4x4AndDiagonal(S);
      l += store4x4AndDiagonal(C);
  //                 l += store4x4AndDiagonal(S);
//        l += store4x4AndDiagonal(Q);

  }else{
    l += A.convertToList();
    //   if(conf.useS) l += S.convertToList();
    l += C.convertToList();
    //      l += R.convertToList();
//      l += Q.convertToList();
  }

  // l += y_integration.convertToList();
//   l += PID_deriv.convertToList();
  //l += y_teaching.convertToList();
   l += HY.convertToList();
   l += xx.convertToList();
//    l += vau_avg.convertToList();
   //   l += xsi.convertToList();
  // l += x_buffer[(t - 1 + buffersize )%buffersize].convertToList();
//   if(conf.sat)
//           l += conf.sat->getInternalParams();
  // l += eta.convertToList();
  //l += xsi.convertToList();
  //  l += x_smooth_long.convertToList();//TEST
//   l += weighting;
//   l += epsA;
//   l += epsC;
//   // l += xsi.elementSum();
//  //  l += xsi_norm;
// //   l += xsi_norm_avg;
//   l += PIDint;
//    l += TLE;
//   l += grang1;
//   l += grang2;
//   l += causalfactor;
  return l;
}

list<Inspectable::ILayer> DerInf::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  l+=ILayer("x","", number_sensors, 0, "Sensors");
  l+=ILayer("y","H", number_motors, 1, "Motors");
  l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> DerInf::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  return l;
}

//double clip095(double x){
// return clip(x,-0.95,0.95);
//}

double DerInf::calcMatrixNorm(const Matrix& m){
  return m.map(fabs).elementSum() / (m.getM() * m.getN());
}
