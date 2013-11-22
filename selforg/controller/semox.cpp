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

#include "semox.h"
#include <selforg/regularisation.h>
using namespace matrix;
using namespace std;

SeMoX::SeMoX( const SeMoXConf& conf)
  : HomeokinBase(conf.buffersize, "SeMoX", "$Id$"),
    conf(conf) {


  addParameterDef("dampModel",&dampModel,0);
  addParameterDef("dampController",&dampController,0);
  addParameterDef("modelNoise",&modelNoise,0);
  addParameterDef("gamma_cont",&gamma_cont,0);
  addParameterDef("gamma_teach",&gamma_teach,0);
  addParameterDef("discountS",&discountS,0.001);
  _modelExt_copy = conf.modelExt;
  addParameter("extendedModel",&_modelExt_copy);

  addInspectableMatrix("A", &A, conf.someInternalParams, "model matrix");
  addInspectableMatrix("S", &S, conf.someInternalParams, "extended Model matrix");
  addInspectableMatrix("C", &C, conf.someInternalParams, "controller matrix");
  addInspectableMatrix("R", &R, conf.someInternalParams, "linear Response matrix");
  if(!conf.someInternalParams){
    addInspectableMatrix("yteach", &y_teaching, false, "motor teaching signal");
  }
  addInspectableMatrix("H", &H, false, "controller bias");
  addInspectableMatrix("B", &B, false, "model bias");
  if(!conf.someInternalParams){
    addInspectableMatrix("v", &v, false, "postdiction error");
  }
  addInspectableValue("xsi_norm", &xsi_norm, "1-norm of prediction error");
  addInspectableValue("xsi_norm_avg", &xsi_norm_avg, "averaged 1-norm of prediction error");

  managementInterval=10;
  intern_useTeaching=false;
  BNoiseGen = 0;
};


SeMoX::~SeMoX(){
  if(x_buffer && y_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
  }
  if(BNoiseGen) delete BNoiseGen;
}


void SeMoX::init(int sensornumber, int motornumber,
                              RandGen* randGen){

  number_motors  = motornumber;
  number_sensors = sensornumber - conf.numContext;

  A.set(number_sensors, number_motors);
  A = (A^0) * conf.aInit; // set A to identity matrix;
  B.set(number_sensors, 1);
  if (conf.modelExt) {
    S.set(number_sensors, number_sensors  + conf.numContext );
    S = (S^0) * conf.sInit;
  }

  if(conf.initialC.getM() != number_motors || conf.initialC.getN() != number_sensors ){
    if(!conf.initialC.isNulltimesNull())
      cerr << "dimension of initialC are not correct, default is used! \n";
    C.set(number_motors,  number_sensors);
    // initialise the C matrix with identity + noise scaled to cInit value
    C = ((C^0) + C.mapP(randGen, random_minusone_to_one) * conf.cNonDiag) * conf.cInit;
  }else{
    C=conf.initialC; // use given matrix
  }

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
  BNoiseGen->init(number_sensors, randGen);

  v.set(number_motors,1); // (shift is in motor space)

  x_buffer = new Matrix[buffersize];
  x_c_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    x_c_buffer[k].set(number_sensors + conf.numContext,1);
    y_buffer[k].set(number_motors,1);
  }
  y_teaching.set(number_motors, 1);

  t_rand = rand()%managementInterval;
  initialised = true;
}


/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void SeMoX::step(const sensor* x_, int number_sensors,
                            motor* y_, int number_motors){
  fillBuffersAndControl(x_, number_sensors, y_, number_motors);
  if(t>buffersize){
    int delay = max(int(s4delay)-1,0);
    calcXsi(delay);            // calculate the error (use delayed y values)
    // learn controller with effective input/output
    learnController();

    // learn Model with actual sensors and with effective motors;
    learnModel(delay);
  }
  // update step counter
  t++;
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void SeMoX::stepNoLearning(const sensor* x, int number_sensors,
                                            motor*  y, int number_motors ){
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
};


void SeMoX::fillBuffersAndControl(const sensor* x_, int number_sensors,
                                              motor* y_, int number_motors){
  assert((unsigned)number_sensors == (this->number_sensors + (unsigned)conf.numContext)
         && (unsigned)number_motors == this->number_motors);

  Matrix x(this->number_sensors,1,x_);
  Matrix x_c(this->number_sensors + conf.numContext,1,x_);

  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);
  putInBuffer(x_c_buffer, x_c);

  // averaging over the last s4avg values of x_buffer
  const Matrix& x_smooth = calculateSmoothValues(x_buffer, t < s4avg ? 1 : int(max(1.0,s4avg)));

  // calculate controller values based on smoothed input values
  const Matrix& y = calculateControllerValues(x_smooth);

  // from time to time call management function. For example damping is done here.
  if((t+t_rand)%managementInterval==0) management();

  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);

  // convert y to motor*
  y.convertToBuffer(y_, number_motors);
}

/// calculates xsi for the current time step using the delayed y values
//  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
void SeMoX::calcXsi(int delay){
  const Matrix& x     = x_buffer[t% buffersize];
  const Matrix& y     = y_buffer[(t - 1 - delay) % buffersize];
  xsi = x -  model(x_buffer, 1 , y);
  xsi_norm = matrixNorm1(xsi);
  xsi_norm_avg = xsi_norm_avg*0.999 + xsi_norm*0.001; // calc longterm average

  pain= xsi_norm > 5*xsi_norm_avg; // pain if large raise in error (arbitrary choice)
}


/// calculates the predicted sensor values
Matrix SeMoX::model(const Matrix* x_buffer, int delay, const matrix::Matrix& y){
  if(conf.modelExt){
    const Matrix& x_c_tm1 = x_c_buffer[(t - 1) % buffersize];
    return A * y + S * x_c_tm1 + B;
  } else {
    return A * y + B;
  }
}

/// learn controller paramters C and H
void SeMoX::learnController(){


  // prepare update matrices
  Matrix C_update(C.getM(), C.getN());
  Matrix H_update(H.getM(), H.getN());
  Matrix C_updateTeaching;
  Matrix H_updateTeaching;
  bool teaching = intern_useTeaching || (gamma_cont!=0);
  if(teaching){
    C_updateTeaching.set(C.getM(), C.getN());
    H_updateTeaching.set(H.getM(), H.getN());
  }


  // eta = A^-1 xsi (first shift in motor-space at current time)
  //  we use pseudoinverse eta = U^-1 A^T xsi, with U=A^T A + \lambda I
  //  it is additionally clipped to -1 to 1 via g (arbitrary choice)
  const Matrix& eta = (A.pseudoInverse(0.001) * xsi).map(g);

  const Matrix& x          = x_buffer[(t-1)%buffersize];
  const Matrix& z          = C * x + H;
  const Matrix g_prime     = Matrix::map2(g_s, z, eta);
  const Matrix g_prime_inv = g_prime.map(one_over);
  const Matrix g_2p_div_p  = Matrix::map2(g_ss_div_s, z, eta);

  const Matrix zeta = eta.multrowwise(g_prime_inv); // G'(Z)^-1 * (eta+v)
  R                 = C * A;
  const Matrix chi  = ((R.multMT()+SmallID)^-1) * zeta;
  v                 = (R^T) * chi;
  // squash v to -3,3 (arbitrary choice)
  double size = 3;
  v = v.mapP(&size,squash);
  const Matrix rho  = g_2p_div_p.multrowwise(chi.multrowwise(zeta)) * -1;

  C_update += ( chi*(v^T)*(A^T) - rho*(x^T) ) * epsC;
  H_update += rho * -epsC;

  // scale of the additional terms (natural gradient with metric LL^T)
  if(teaching){
    // scale of the additional terms
    const Matrix& LLT_I = ((R & g_prime).multMT()+SmallID)^-1;

    if(gamma_cont!=0) {  // learning to keep motorcommands smooth
      // the teaching signal is the previous motor command
      const Matrix& y = y_buffer[(t)% buffersize];
      const Matrix& y_tm1 = y_buffer[(t-1)% buffersize];
      const Matrix& delta = (y_tm1 - y) & (g_prime);
      C_updateTeaching += ( LLT_I * delta *(x^T) ) * (gamma_cont * epsC);
      H_updateTeaching += LLT_I * delta * (gamma_cont * epsC);
    }
    if(intern_useTeaching && gamma_teach!=0){
      const Matrix& y = y_buffer[(t-1)% buffersize];
      const Matrix& xsi = y_teaching - y;
      const Matrix& delta = xsi.multrowwise(g_prime);
      C_updateTeaching += (LLT_I * delta*(x^T) ) * (gamma_teach * epsC);
      H_updateTeaching += (LLT_I * delta) * (gamma_teach * epsC);
      intern_useTeaching=false; // after we applied teaching signal it is switched off until new signal is given
    }
  }

  double error_factor = calcErrorFactor(v, (logaE & 1) !=0, (rootE & 1) !=0);
  C_update *= error_factor;
  H_update *= error_factor;
  if(teaching){
    C_update+=C_updateTeaching;
    H_update+=H_updateTeaching;
  }

  C += C_update.mapP(&squashSize, squash);
  double H_squashSize = squashSize*10;
  H += H_update.mapP(&H_squashSize, squash);
};


// normal delta rule (xsi is assumed to be already up to date via calcXsi())
void SeMoX::learnModel(int delay){
  const Matrix& y_tm1 = y_buffer[(t - 1 - delay) % buffersize];

  if(!pain) {
    double error_factor = calcErrorFactor(xsi, (logaE & 2) != 0, (rootE & 2) != 0);
    Matrix A_update;
    Matrix S_update;
    Matrix B_update;

    // new hierarchical learning
    if(conf.modelExt){
      const Matrix& x = x_buffer[t % buffersize];
      const Matrix& x_c_tm1 = x_c_buffer[(t - 1) % buffersize];
      // first learn A on error with discounted S
      const Matrix& zeta = x -  (A*y_tm1 + B + (S*x_c_tm1)*(1-discountS));
      A_update=(( zeta*(y_tm1^T) ) * (epsA * error_factor));
      S_update=(( xsi*(x_c_tm1^T) ) * (epsA * error_factor));// as usual with full error
    }else{
      A_update=(( xsi*(y_tm1^T) ) * (epsA * error_factor));
    }

    B_update=( xsi * (epsA * factorB * error_factor));
    if(modelNoise>0){
      B_update  = noiseMatrix(B.getM(),B.getN(), *BNoiseGen, -modelNoise, modelNoise); // noise for bias
    }

    A += A_update.mapP(&squashSize, squash);
    B += B_update.mapP(&squashSize, squash);
    if(conf.modelExt) S += S_update.mapP(&squashSize, squash);
  }
};

/// calculate controller outputs
/// @param x_smooth smoothed sensors Matrix(number_channels,1)
Matrix SeMoX::calculateControllerValues(const Matrix& x_smooth){
  return (C*x_smooth+H).map(g);
};


void SeMoX::management(){
  if(dampModel){
    A *= 1 - dampModel * managementInterval;
    B *= 1 - dampModel * managementInterval;
    S *= 1 - dampModel * managementInterval;
  }
  if(dampController){
      C *= 1 - dampController * managementInterval;
  }
}

list<Matrix> SeMoX::getParameters() const {
  return {C,H};
}

int SeMoX::setParameters(const list<Matrix>& params){
  if(params.size() == 2){
    list<Matrix>::const_iterator i = params.begin();
    const Matrix& CN = *i;
    if(C.hasSameSizeAs(CN)) C=CN;
    else return false;
    const Matrix& HN = *(++i);
    if(H.hasSameSizeAs(HN)) H=HN;
    else return false;
  } else {
    fprintf(stderr,"setParameters wrong len %i!=2\n", (int)params.size());
    return false;
  }
  return true;
}


bool SeMoX::store(FILE* f) const {
  // save matrix values
  C.store(f);
  H.store(f);
  A.store(f);
  B.store(f);
  if(conf.modelExt) S.store(f);
  Configurable::print(f,0);
  return true;
}

bool SeMoX::restore(FILE* f){
  // save matrix values
  C.restore(f);
  H.restore(f);
  A.restore(f);
  B.restore(f);
  if(conf.modelExt) S.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

list<Inspectable::ILayer> SeMoX::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  l+=ILayer("x","", number_sensors, 0, "Sensors");
  l+=ILayer("y","H", number_motors, 1, "Motors");
  l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> SeMoX::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  if(conf.modelExt) l+=IConnection("S", "x", "xP");
  return l;
}


///////// TEACHABLE ////////////////

void SeMoX::setMotorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_motors && teaching.getN() == 1);
  // Note: through the clipping the otherwise effectless
  //  teaching with old motor value has now an effect,
  //  namely to drive out of the saturation region.
  y_teaching= teaching.mapP(0.95,clip);
  intern_useTeaching=true;
}

void SeMoX::setSensorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_sensors && teaching.getN() == 1);
  // calculate the y_teaching, that belongs to the distal teaching value by the inverse model.
  y_teaching = (A.pseudoInverse(0.001) * (teaching-B)).mapP(0.95, clip);
  intern_useTeaching=true;
}

matrix::Matrix SeMoX::getLastMotorValues(){
  return y_buffer[(t-1+buffersize)%buffersize];
}

matrix::Matrix SeMoX::getLastSensorValues(){
  return x_buffer[(t-1+buffersize)%buffersize];
}
