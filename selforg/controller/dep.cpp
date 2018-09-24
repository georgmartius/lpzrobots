/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
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
*                                                                         *
***************************************************************************/

#include <algorithm>
#include <numeric>

#include "dep.h"
#include <selforg/matrixutils.h>

#include <numeric>

using namespace matrix;
using namespace std;

DEP::DEP(const DEPConf& conf)
  : AbstractController("DEP", "1.0"),
    conf(conf)
{
  t=0;

  addParameterDef("epsC", &epsC, 0.1,     0,5, "learning rate of the controller");
  addParameterDef("epsh", &epsh, 0.1,     0,5, "learning rate of the controller bias");
  addParameterDef("epsA", &epsA, 0.0,     0,5, "learning rate of the model");
  addParameterDef("s4avg", &s4avg, 1,     1, buffersize-1, "smoothing (number of steps)");
  addParameterDef("s4delay", &s4delay, 1, 1, buffersize-1, "delay  (number of steps)");
  addParameterDef("damping",   &damping,     0.0001, 0,0.01, "forgetting term for model");

  addParameterDef("learningrule",  (int*)(&this->conf.learningRule),
                  false,              std::string("which learning rule to use: ") +
                  std::accumulate(conf.LearningRuleNames.begin(),conf.LearningRuleNames.end(),
                                  std::string(),[](std::string a, std::pair<DEPConf::LearningRule,std::string> lr){return a + itos((int)lr.first) + ": " + lr.second + ", ";}));
  addParameterDef("timedist", &timedist, 1,     0,10, "time distance of product terms in learning rule");
  addParameterDef("synboost", &synboost, 5,     0,1,  "booster for synapses during motor signal creation");
  addParameterDef("urate", &urate, .1,          0,5,  "update rate ");

  //  addParameterDef("maxspeed", &maxSpeed, 0.5,   0,2, "maximal speed for motors");
  addParameterDef("indnorm", &indnorm,     1,   0,2, "individual normalization for each motor");
  addParameterDef("regularization", &regularization, 12, 0, 15, "exponent of regularization 10^{-regularization}");

  addInspectableMatrix("A", &A, false, "model matrix");
  //  addInspectableMatrix("S", &S, true, "model matrix");

  if(conf.useExtendedModel) addInspectableMatrix("S", &S, false, "model matrix (sensor branch)");
  addInspectableMatrix("h", &h, false,   "acting controller bias");
  addInspectableMatrix("C", &C, false, "acting controller matrix");

  if(conf.calcEigenvalues){
    addInspectableMatrix("EvRe", &eigenvaluesLRe, false, "Eigenvalues of L (Re)");
    addInspectableMatrix("EvIm", &eigenvaluesLIm, false, "Eigenvalues of L (Im)");
    addInspectableMatrix("EVs",  &eigenvectors,   false, "Eigenvectors of L (Re)");
    addInspectableValue("proj1",  &proj_ev1,  "projection of x on first Eigenvector (Re)");
    addInspectableValue("proj2",  &proj_ev2,  "projection of x on second Eigenvector (Re)");
    addParameterDef("evinterval", &calcEVInterval, 1,          0,1000,  "interval to update eigenvalues etc (0: never) ");
  }

  addInspectableValue("norming", &norming, "Normalization");
  addInspectableMatrix("normmor", &normmot, false, "individual motor normalization factor");

  _internWithLearning=false; // used in step to enable learning in stepNoLearning and have only one function
};

DEP::~DEP(){
}


void DEP::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak


  number_sensors= sensornumber;
  number_motors = motornumber;
  A.set(number_sensors, number_motors);
  S.set(number_sensors, number_sensors);
  C.set(number_motors, number_sensors);
  C_update.set(number_motors, number_sensors);
  b.set(number_sensors, 1);
  h.set(number_motors, 1);
  L.set(number_sensors, number_sensors);
  eigenvectors.set(number_sensors,number_sensors);
  eigenvaluesLRe.set(number_sensors,1);
  eigenvaluesLIm.set(number_sensors,1);
  normmot.set(number_motors, 1);

  R.set(number_sensors, number_sensors);

  // special Model initialization for delay sensor or direct perception of others
  if(conf.initModel){
    if(number_sensors>=2*number_motors){
      Matrix A1(number_sensors/2,number_motors);
      A1.toId();
      A=A1.above(A1);
    }else
      A.toId(); // set a to identity matrix;
  }

  C_update.toId();
  C_update*=conf.initFeedbackStrength;

  x_smooth.set(number_sensors,1);

  x_buffer.init(buffersize, Matrix(number_sensors,1));
  y_buffer.init(buffersize, Matrix(number_motors,1));
 }


// performs one step (includes learning). Calculates motor commands from sensor inputs.
void DEP::step(const sensor* x_, int number_sensors,
               motor* y_, int number_motors){
  _internWithLearning=true;
  stepNoLearning(x_, number_sensors, y_, number_motors);
  _internWithLearning=false;
};


void DEP::stepNoLearning(const sensor* x_, int number_sensors_robot,
                         motor* y_, int number_motors_){
  assert((unsigned)number_sensors_robot <= this->number_sensors
         && (unsigned)number_motors_ <= this->number_motors);

  Matrix xrobot(number_sensors_robot,1,x_); // store sensor values

  // averaging over the last s4avg values of x_buffer
  // if ( damping ==0) for(int i = 18; i<36; i++) xrobot.val(i,0) = xrobot.val(i-18,0);
  if(s4avg > 1)
    x_smooth += (xrobot - x_smooth)*(1.0/s4avg);
  else
    x_smooth = xrobot;

  x_buffer[t] = x_smooth;

  // if(damping==0) for (int i=18; i<36;i++) x_buffer[t].val(i,0) = x_buffer[t-20].val(i,0);


  if(_internWithLearning)
    learnController();

  // Controller function
  Matrix y =   (C*x_smooth  + h  ).map(g);

  // y_buffer[t] = y;
  // the motors have their own maximal speed, so there should be no need
  // //protecting the robot motors:
  // double maxDy = maxSpeed;
  // Matrix dy = y - y_buffer[t-1];
  // y = y_buffer[t-1] + dy.mapP(maxDy,clip);

  y_buffer[t] = y;

  if(_internWithLearning && epsA!=0)
    learnModel(epsA);


  y.convertToBuffer(y_, number_motors);
  // update step counter
  t++;
};


void DEP::learnController(){
  ///////////////// START of Controller Learning / Update  ////////////////

  int offset = 0; //Shifting v forward in time by offset steps.

  const Matrix& x   = x_buffer[t];
  const Matrix& xx  = x_buffer[t - 1 + offset];
  const Matrix& xxx = x_buffer[t - 2 + offset];
  const Matrix& yy  = y_buffer[t - 1];
  const Matrix& yyy = y_buffer[t - 2];
  // cout <<"senasordiff="<< x.val(0,0) - x.val(18,0)<<endl;

  Matrix mu;
  Matrix v;
  switch(conf.learningRule){
  case DEPConf::DEPNormRule:  ////////////////////////////
  case DEPConf::DEPRule: { ////////////////////////////
    Matrix chi  = x - xx - S*(xx -xxx);
    if (conf.learningRule==DEPConf::DEPNormRule){
      // normalize chi
      chi*=(1.0/(sqrt(chi.norm_sqr())+0.001));
    }
    //v  =   xx-xxx; (except that we introduce a time distance between chi and v
    v = x_buffer[t - timedist + offset] - x_buffer[t - timedist - 1 + offset];
    const Matrix& B = A^T;
    mu = (B * chi);
    break;
  }
  case DEPConf::DHLRule:  ////////////////////////////
    mu  = yy - yyy;
    //v   = xx - xxx;
    v = x_buffer[t - timedist + offset] - x_buffer[t - timedist -1 + offset];
    break;
  case DEPConf::HLwFB: { ////////////////////////////
    const Matrix& chi  = x - S*(xx);
    v    = xx;
    const Matrix& B = A^T;
    mu = (B * chi);
    break;
  }
  case DEPConf::HLPlain: ////////////////////////////
    mu = yy;
    v  = xx;
    break;
  default:
    cerr << "unkown learning rule!" << endl;
  }
  const Matrix& updateC =   ( mu ) * (v^T);

  C_update += ((updateC   - C_update)*urate);      // fast dynamics

  C = C_update;       // matrix for controller before normalization

  double reg = pow(10,-regularization);
  switch(indnorm){
  case 1: {
    //***** individual normalization for each motor neuron***************
    // Matrix normmot(h);//just for Initialisierung
    const Matrix& CA=C*A;
    for (int i=0; i<number_motors; i++) {
      double normi = sqrt(CA.row(i).norm_sqr()); // norm of one row
      normmot.val(i,0) = .3*synboost/( normi + reg);
    }
    C = C.multrowwise(normmot);
    break;
  }
  case 0: { // global
    double norm1 = sqrt((C*A).norm_sqr());
    norming +=  (norm1 - norming)*.3; // just for logging
    C *= synboost/(norm1 + reg);    // C stays relatively constant is size
    C.toMapP(5.0,clip); // nevertheless clip C to some reasonable range
    break;
  }
  default:  { // no normalization
    C *= synboost;
    break;
  }}
  if(conf.calcEigenvalues){
    if(calcEVInterval!=0 && (t%calcEVInterval==0)){
      Matrix EVImag;
      const Matrix& L=A*C;
      eigenValuesVectors(L, eigenvaluesLRe, eigenvaluesLIm, eigenvectors, EVImag);
      toPositiveSignEigenVectors(eigenvectors, EVImag);
      scaleEigenVectorsWithValue(eigenvaluesLRe, eigenvaluesLIm, eigenvectors, EVImag);
    }
    // calc overlap of sensor state with first 2 eigenvectors (this we do every step
    proj_ev1=((eigenvectors.column(0)^T) * x).val(0,0);
    proj_ev2=((eigenvectors.column(1)^T) * x).val(0,0);
  }

  const Matrix& y_last = y_buffer[t-1];
  if(epsh>=0)
    h -= ( y_last *  epsh).mapP(.05, clip) + h*.001;
  else
    h*=0;

  ///////////////// End of Controller Learning ////////
}

void DEP::learnModel(double eps){
  // learn inverse model y = F(x') = M x' where M=A^T
  // the effective x/y is (actual-steps4delay) element of buffer
  s4delay = ::clip(s4delay,1,buffersize-1);
  int  t_delay =  max(s4delay,1)-1;
  /// we learn here with the velocities.
  if(eps!=0){
    const Matrix& ydot = y_buffer[t - t_delay - timedist] - y_buffer[t - timedist - 1 - t_delay];
    // future sensor (with respect to x,y)
    const Matrix& x_fut   = x_buffer[t] - x_buffer[t - 1];
    // learn model
    const Matrix& xi = ydot - ((A^T) * x_fut);
    //M += eps xi x_fut^T -> A = eps x_fut xi^T
    A += (x_fut *(xi^T) * (eps)).mapP(0.05, clip);
  }
};


void DEP::motorBabblingStep(const sensor* x_, int number_sensors_robot,
                            const motor* y_, int number_motors){
  assert((unsigned)number_motors <= this->number_motors);
  Matrix x(number_sensors_robot,1,x_); // convert to matrix
  Matrix y(number_motors,1,y_); // convert to matrix
  x_buffer[t] = x;
  y_buffer[t] = y;

  // model learning
  learnModel(1.0/(sqrt(t+1)));

  t++;
}


/* stores the controller values to a given file. */
bool DEP::store(FILE* f) const{
  // save matrix values
  C.store(f);
  C_update.store(f);
  h.store(f);
  A.store(f);
  b.store(f);
  S.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool DEP::restore(FILE* f){
  // save matrix values
  C.restore(f);
  C_update.restore(f);
  h.restore(f);
  A.restore(f);
  b.restore(f);
  S.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}
