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

#include "invertmotornstep.h"
#include "regularisation.h"
using namespace matrix;
using namespace std;

InvertMotorNStep::InvertMotorNStep( const InvertMotorNStepConf& conf)
    : InvertMotorController(conf.buffersize, "InvertMotorNStep", "$Id$"), conf(conf)
{

  addParameterDef("inhibition",&inhibition,0);
  addParameterDef("kwta",&kwta,2);
  addParameterDef("limitrf",&limitRF,0);
  addParameterDef("dampS",&dampS,0);
  addParameterDef("dampC",&dampC,0);
  addParameterDef("noiseY",&noiseY,0);
  addParameterDef("modelcompl",&modelCompliant,0);
  addParameterDef("continuity",&continuity,0);
  addParameterDef("activeexpl",&activeExplore,0);
  // Georg: remove because special case: make a copy to the precise simulation!
  addParameterDef("cfactor",&cfactor,1);
  addParameterDef("cnondiagabs",&cnondiagabs,0);
  addParameterDef("cdiagabs",&cdiagabs,0);

  managementInterval=10;
  useTeaching=false;
  reinforcement=0;
  E_val=0;
  reinforcefactor=0;
  BNoiseGen = 0;
  YNoiseGen = 0;

  x_buffer = 0;
  y_buffer = 0;
  eta_buffer = 0;

  addInspectableMatrix("A", &A, conf.someInternalParams, "model matrix");
  if(conf.useS)
    addInspectableMatrix("S", &S, conf.someInternalParams, "extended Model matrix");
  if(conf.useSD)
    addInspectableMatrix("SD", &SD, conf.someInternalParams, "extended Model matrix (deriv)");

  addInspectableMatrix("C", &C, conf.someInternalParams, "controller matrix");
  addInspectableMatrix("R", &R, conf.someInternalParams, "linear Response matrix");
  addInspectableMatrix("H", &H, false, "controller bias");
  addInspectableMatrix("B", &B, false, "model bias");
  if(!conf.someInternalParams){
    addInspectableMatrix("yteach", &y_teaching, false, "motor teaching signal");
    addInspectableMatrix("xsi", &xsi, false, "prediction error");
    addInspectableMatrix("v", &v, false, "postdiction error");
  }
  addInspectableValue("epsA", &epsA, "learning rate of the Model");
  addInspectableValue("epsC", &epsC, "learning rate of the Controller");
  addInspectableValue("xsi_norm", &xsi_norm, "1-norm of prediction error");
  addInspectableValue("xsi_norm_avg", &xsi_norm_avg, "averaged 1-norm of prediction error");
  addInspectableValue("E", &E_val, "value of error function");
  addInspectableValue("pain", &pain, "internal pain signal");
  addInspectableValue("reinforce", &reinforcement, "reward value");
};


InvertMotorNStep::~InvertMotorNStep()
{
  if(x_buffer && y_buffer && eta_buffer)
  {
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] eta_buffer;
  }
  if(BNoiseGen) delete BNoiseGen;
  if(YNoiseGen) delete YNoiseGen;
}


void InvertMotorNStep::init(int sensornumber, int motornumber, RandGen* randGen)
{
  assert(sensornumber>=motornumber);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak

  number_motors  = motornumber;
  number_sensors = sensornumber;
  if(conf.numberContext == 0) // we just declare all sensors as context
    conf.numberContext = number_sensors;

  //  z.set(number_motors,1);
  A.set(number_sensors, number_motors);
  // S gets context sensors
  if (conf.useS) S.set(number_sensors, conf.numberContext);
  // S gets first and second derivative of context sensors
  if (conf.useSD) SD.set(number_sensors, number_sensors*2);
  C.set(number_motors,  number_sensors);
  H.set(number_motors,  1);
  B.set(number_sensors, 1);
  R.set(number_motors, number_motors);
  SmallID.set(number_motors, number_motors);
  SmallID.toId();
  SmallID *= 0.001;

  xsi.set(number_sensors,1);
  xsi_norm=0;
  xsi_norm_avg=0.2;
  pain=0;
  sensorweights.set(number_sensors,1);
  sensorweights.toMapP(1, constant); // fill it with ones

  BNoiseGen = new WhiteUniformNoise();
  BNoiseGen->init(number_sensors);
  YNoiseGen = new ColorUniformNoise(0.1);
  //  YNoiseGen = new WhiteUniformNoise();
  YNoiseGen->init(number_motors);

  A.toId(); // set A to identity matrix;
  //  if (conf.useS) S.mapP(randGen, random_minusone_to_one)*0.01; // set S to small random matrix;
  // if (conf.useSD) S.mapP(randGen, random_minusone_to_one)*0.01; // set SD to small random matrix;

  if(conf.initialC.getM() != number_motors || conf.initialC.getN() != number_sensors ){
    if(!conf.initialC.isNulltimesNull()) cerr << "dimension of Init C are not correct, default is used! \n";
    // initialise the C matrix with identity + noise scaled to cInit value
    C = ((C^0) + C.mapP(randGen, random_minusone_to_one) * conf.cNonDiag) * conf.cInit;
  }else{
    C=conf.initialC; // use given matrix
  }
  cnondiagabs = conf.cNonDiagAbs;

  x_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  eta_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++)
  {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
    eta_buffer[k].set(number_motors,1);
  }
  y_teaching.set(number_motors, 1);

  zero_eta.set(number_motors, 1);
  v=zero_eta;

  t_rand = int(randGen->rand()*10);
  initialised = true;
}

/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void InvertMotorNStep::step(const sensor* x_, int number_sensors,
                            motor* y_, int number_motors)
{
  fillBuffersAndControl(x_, number_sensors, y_, number_motors);
  if(t>buffersize)
  {
    int delay = max(int(s4delay)-1,0);
    calcXsi(delay);            // calculate the error (use delayed y values)
    calcEtaAndBufferIt(delay);
    // learn controller with effective input/output
    learnController(delay);

    // learn Model with actual sensors and with effective motors;
    learnModel(delay);
  }
  // update step counter
  t++;

//   // Georg: This a very special case. Should be removed! Make a copy to the precise simulation
  if (cfactor!=1)
  {
    C = (C^0) *cfactor;
  }
  if (cnondiagabs!=0)
  {
    C.val(0,1)=cnondiagabs;
    C.val(1,0)=cnondiagabs;
  }
  if (cdiagabs!=0)
  {
    C.val(0,0)=cdiagabs;
    C.val(1,1)=cdiagabs;
  }

};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void InvertMotorNStep::stepNoLearning(const sensor* x, int number_sensors,
                                      motor*  y, int number_motors )
{
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
};

void InvertMotorNStep::fillBuffersAndControl(const sensor* x_, int number_sensors,
    motor* y_, int number_motors)
{
  assert((unsigned)number_sensors == this->number_sensors
         && (unsigned)number_motors == this->number_motors);

  Matrix x(number_sensors,1,x_);

  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);

  // averaging over the last s4avg values of x_buffer
  x_smooth = calculateSmoothValues(x_buffer, t < s4avg ? 1 : int(max(1.0,s4avg)));


  // calculate controller values based on smoothed input values
  Matrix y = calculateControllerValues(x_smooth);
  if(noiseY!=0)
  {// add motor noise
    // calculate a noise matrix for motor values
    Matrix y_noise  = noiseMatrix(y.getM(),y.getN(), *YNoiseGen, -noiseY, noiseY); // noise for bias
    y += y_noise;
  }
  // Question: do this before or after buffering y
  if(activeExplore!=0)
  { // through model backpropagated error used as control signal for robot
    y += eta_buffer[(t-1)%buffersize]*activeExplore;
  }

  // from time to time call management function. For example damping and inhibition is done.
  if((t+t_rand)%managementInterval==0) management();

  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);

  // convert y to motor*
  y.convertToBuffer(y_, number_motors);
}


double InvertMotorNStep::regularizedInverse(double v)
{
  return 1/(fabs(v)+0.1);
}

/// calculates Eta which corrensponds to xsi_t which means eta_{t-1}
//  @param delay 0 for no delay and n>0 for n timesteps delay in the SML
void InvertMotorNStep::calcEtaAndBufferIt(int delay)
{
  // eta = A^-1 xsi (first shift in motor-space at current time)
  //
  // Georg: a comment to the pseudoinverse: This does not quite give
  //  the right result in case of rectangular matrix. Consider:
  //  A=(1,0.1), xsi=(0,1), We would expect eta to be very large,
  //  because the response of A along the second dimension is very low.
  //  However the result is eta \approx 0.1, which is correct in the
  //  mean squares sense
  //
  // 09.10.2008
  //  For rectangular matrices it is better to use A^{-1} = (A + \lambda I)^{-1}

  // We use pseudoinverse U=A^T A -> eta = U^-1 A^T xsi // TODO add 0.01*I
  Matrix eta = (A.multTM()^-1) * ( (A^T) * xsi );

  // new 18.10.2007 by Georg
  // squash eta
  double squashsize = 0.5;
  eta = eta.mapP(&squashsize, squash);

  if(relativeE != 0)
  { // divide eta by |y|  == relative error
    const Matrix& y = y_buffer[t % buffersize];
    eta = eta.multrowwise(y.map(regularizedInverse));
  }
  eta_buffer[(t-1)%buffersize] = eta; // Todo: work with the reference
}

/// calculates xsi for the current time step using the delayed y values
//  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
void InvertMotorNStep::calcXsi(int delay)
{
  const Matrix& x     = x_buffer[t% buffersize];
  const Matrix& y     = y_buffer[(t - 1 - delay) % buffersize];
  //  xsi = (x -  model(x_buffer, 1 , y));
  xsi = (x -  model(x_buffer, 1 , y)).multrowwise(sensorweights); // new Georg 18.10.2007
  //  xsi_norm = matrixNorm1(xsi);
  xsi_norm = xsi.multTM().val(0,0);
}


/// calculates the predicted sensor values
Matrix InvertMotorNStep::model(const Matrix* x_buffer, int delay, const matrix::Matrix& y)
{
  Matrix xp = A * y + B;
  if(conf.useS)
  {
    const Matrix& x_c = x_buffer[(t-delay)% buffersize].rows(number_sensors - conf.numberContext,number_sensors - 1);
    xp += S * x_c ;
  }
  if(conf.useSD)
  {
    const Matrix& x_primes = calcDerivatives(x_buffer, delay);
    xp += SD * x_primes ;
  }
  return xp;
}


/// learn values H,C
// This is the implementation uses a better formula for g^-1 using Mittelwertsatz
void InvertMotorNStep::learnController(int delay)
{

  Matrix C_update(C.getM(), C.getN());
  Matrix H_update(H.getM(), H.getN());
  calcCandHUpdates(C_update, H_update, delay);

  if(adaptRate>0)
  { // adapt learning rate
    double norm = matrixNorm1(C); // + calcMatrixNorm(H);
    double update_norm = matrixNorm1(C_update) + matrixNorm1(H_update)*0.05;

    epsC = adapt(epsC, update_norm, norm*nomUpdate, adaptRate, adaptRate*10);
    epsC = min( 5.0, epsC);
  }
  updateCandH(C_update, H_update, squashSize);
  E_val = getE();
};

/// calculates the Update for C and H
// @param y_delay timesteps to delay the y-values.  (usually 0)
//  Please note that the delayed values are NOT used for the error calculation
//  (this is done in calcXsi())
void InvertMotorNStep::calcCandHUpdates(Matrix& C_update, Matrix& H_update, int delay)
{
  assert( steps + delay < buffersize);
  // Matrix& eta = zero_eta;
  // Matrix v_old = (eta_buffer[t%buffersize]).map(g);
  bool teaching = (modelCompliant!=0) || useTeaching || (continuity!=0);
  Matrix C_updateTeaching;
  Matrix H_updateTeaching;
  if(teaching)
  {
    C_updateTeaching.set(C.getM(), C.getN());
    H_updateTeaching.set(H.getM(), H.getN());
  }
  v = zero_eta;
  double shiftlimit = 1; //maximal size of vector components in shifts
  //and other intermediate vectors calculated with the inverse or pseudoinverse
  for(unsigned int s = 1; s <= steps; s++)
  {
    const Matrix& eta = eta_buffer[(t-s)%buffersize].map(g);

    //const Matrix& x      = A * z.map(g) + xsi;
    //    z                    = R * z.map(g) + H + C*xsi; // z is a dynamic itself (not successful)
    const Matrix& x          = x_buffer[(t-s)%buffersize];
    const Matrix& z          = C * x + H;
    const Matrix shift       = (eta + v).mapP(&shiftlimit,squash); // limit shift by shiftlimit (arbitrary choice)
    const Matrix g_prime     = Matrix::map2(g_s, z, shift);
    const Matrix g_prime_inv = g_prime.map(one_over);
    const Matrix g_2p_div_p  = Matrix::map2(g_ss_div_s, z, shift);

    const Matrix zeta = shift.multrowwise(g_prime_inv); // G'(Z)^-1 * (eta+v)
    R                 = C * A;
    // Georg 09.10.2008: consider to use different pseudoinverse: (R+\lambda I)^-1
    const Matrix chi  = (((R.multMT()+SmallID)^-1) * zeta);
    v                 = ((R^T) * chi).mapP(&shiftlimit,squash);// limit by shiftlimit // New Georg 10.2007;
    const Matrix rho  = g_2p_div_p.multrowwise(chi.multrowwise(zeta)) * -1;

    // calculate updates of H,C
    // delta C += eps * (zeta * v^T * A^T + beta * x^T)
    //  q_prime_inv.elementProduct is the correction of negled terms
    //   (LL^T)^-1 from learning rule, which drive the system out of saturation. (important)
    // double q_prime_invers = q_prime_inv.elementProduct();
    C_update += ( chi*(v^T)*(A^T) - rho*(x^T) ) * epsC;
    H_update += rho * -epsC;

    Matrix LLT_I;
    if(teaching){
      // scale of the additional terms
      LLT_I = ((R & g_prime).multMT()+SmallID)^-1;
    }

    if(modelCompliant!=0 && s==1)
    {  // learning of the forward task:
      // eta is difference between last y and reconstructed one
      //    -> used as forward error signal
      // The question is which delta to use: eta (linearised), zeta (neuron
      // inverse) or eta*g' (Backprop) !
      // Also: we need to make sure that z+delta is not in the saturation region
      Matrix delta = eta.multrowwise(g_prime);// eta; // zeta; //eta.multrowwise(g_prime);
      double cutat = 3;
      delta = (delta + z).mapP(&cutat, squash)-delta; // this not quite clean
      C_updateTeaching += ( delta*(x^T) ) * (modelCompliant * epsC);
      H_updateTeaching += delta * (modelCompliant * epsC);

    }
    if(continuity!=0 && s==1)
    {  // learning to keep motorcommands smooth
      // the teaching signal is the previous motor command
      const Matrix& y_tm1 = y_buffer[(t-1-delay)% buffersize];
      const Matrix& y_tm2 = y_buffer[(t-2-delay)% buffersize];
      const Matrix& xsi = y_tm2 - y_tm1;
      const Matrix& delta = xsi.multrowwise(g_prime);
      // we scale the update with the size of the TLE
      //   in order to keep the influence of this update low
//       double v_size = v.multTM().val(0,0);
//       C_updateTeaching += ( delta*(x^T) ) * (continuity * epsC * v_size);
//       H_updateTeaching += delta * (continuity * epsC * v_size);
      C_updateTeaching += (LLT_I*( delta*(x^T) )) * (continuity * epsC);
      H_updateTeaching += (LLT_I*delta) * (continuity * epsC);
      // cerr << v.multTM().val(0,0) << "\t" << xsi.multTM().val(0,0) << "\n";
    }
    if(useTeaching && s==1)
    {
      const Matrix& y = y_buffer[(t-1-delay)% buffersize]; // eventuell t-1
      //      printf("Learn: %i\n", t-1-delay);
      const Matrix& xsi = y_teaching - y;
      const Matrix& delta = xsi.multrowwise(g_prime);
//       double v_size = v.multTM().val(0,0);
//       C_updateTeaching += ( delta*(x^T) ) * (teacher * epsC * v_size);
//       H_updateTeaching += delta * (teacher * epsC * v_size);
      C_updateTeaching += (LLT_I * delta*(x^T) ) * (teacher * epsC);
      H_updateTeaching += (LLT_I * delta) * (teacher * epsC);
      useTeaching=false; // after we applied teaching signal it is switched off until new signal is given
    }
  }
  // we are just using the last shift here! Is this of any problem.
  double error_factor = calcErrorFactor(v, (logaE & 1) !=0, (rootE & 1) !=0);
  // apply reinforcement factor
  error_factor *= reinforcefactor;
  reinforcefactor=1; // only use it in one timestep (next has to be given by setReinforcement())

  C_update *= error_factor;
  H_update *= error_factor;
  if(teaching)
  {
    C_update+=C_updateTeaching;
    H_update+=H_updateTeaching;
  }
};

/// calculates the Update for C and H using the teaching signal
//  @param y_delay timesteps to delay the y-values.  (usually 0)
//  Please note that the delayed values are NOT used for the error calculation
//  (this is done in calcXsi())
// UNUSED! This is an old implementation: We have to figure out why we did it this way!
void InvertMotorNStep::calcCandHUpdatesTeaching(Matrix& C_update, Matrix& H_update, int y_delay)
{
  assert( steps + y_delay < buffersize);
  const Matrix& y = y_buffer[(t)% buffersize]; // eventuell t-1
  const Matrix eta = y_teaching - y;

  const Matrix& x = x_buffer[(t)%buffersize];
  const Matrix z  = C * x + H;
  const Matrix g_          = z.map(g);
  const Matrix g_prime     = Matrix::map2(g_s, z, eta);
  const Matrix g_prime_inv = g_prime.map(one_over);
  const Matrix g_2p_div_p  = Matrix::map2(g_ss_div_s, z, eta);
  R = C * A;
  const Matrix zeta = eta.multrowwise(g_prime_inv);
  const Matrix chi = (R.multMT()^-1) * zeta;
  const Matrix v   = ((R + SmallID)^-1) * zeta;

  const Matrix rho = chi.multrowwise(zeta).multrowwise(g_2p_div_p) * -1;
  C_update = (chi * (v^T) * (A^T) + (chi*teacher - rho)*(x^T)) * epsC * 0.1;
  H_update = (chi*teacher - rho) * epsC * 0.1;

  //  we use error_factor of 1 for teaching
  //  C_update *= error_factor;
  //  H_update *= error_factor;
};

void InvertMotorNStep::updateCandH(const Matrix& C_update, const Matrix& H_update, double _squashSize)
{
  C += C_update.mapP(&_squashSize, squash);
  double H_squashSize = _squashSize*10;
  H += H_update.mapP(&H_squashSize, squash);
}


// normal delta rule (xsi is assumed to be already up to date)
void InvertMotorNStep::learnModel(int delay)
{
  const Matrix& y = y_buffer[(t - 1 - delay) % buffersize];
  xsi_norm_avg = xsi_norm_avg*0.999 + xsi_norm*0.001; // calc longterm average

  if(xsi_norm > 5*xsi_norm_avg)
  {
    pain= 1; //xsi_norm/ xsi_norm_avg/5;
  }
  else
  {
    pain = 0; //pain > 1 ? pain*0.9: 0;
    //    double error_factor = calcErrorFactor(xsi, true, false); // logarithmic error
    double error_factor = calcErrorFactor(xsi, (logaE & 2) != 0, (rootE & 2) != 0); //
    Matrix A_update;
    Matrix B_update;
    A_update=(( xsi*(y^T) ) * (epsA * error_factor));

    if(conf.useS)
    {
      // select the last conf.numberContext sensors.
      const Matrix& x_c      =
        x_buffer[(t-1) % buffersize].rows(number_sensors - conf.numberContext,
                                          number_sensors - 1);
      const Matrix& S_update =(( xsi*(x_c^T) ) * (epsA * error_factor));
      S += S_update.mapP(&squashSize, squash);
    }
    if(conf.useSD)
    {
      Matrix x_primes = calcDerivatives(x_buffer, 1);
      const Matrix& SD_update=(( xsi*(x_primes^T) ) * (epsA * error_factor));
      SD += SD_update.mapP(&squashSize, squash);
    }

    B_update=( xsi * (epsA * factorB * error_factor));
    if(noiseB>0)
    {
      Matrix B_noise  = noiseMatrix(B.getM(),B.getN(), *BNoiseGen, -noiseB, noiseB); // noise for bias
      B_update += B_noise;
    }

    if(adaptRate!=0)
    {
      double normA = matrixNorm1(A);
      epsA=adaptMinMax(epsA, matrixNorm1(A_update) , normA/2500, normA/250, adaptRate, adaptRate*5);
      epsA = min( 2.0, epsA);
    }

    A += A_update.mapP(&squashSize, squash);
    B += B_update.mapP(&squashSize, squash);
  }
};

/// calculate controller outputs
/// @param x_smooth smoothed sensors Matrix(number_channels,1)
Matrix InvertMotorNStep::calculateControllerValues(const Matrix& x_smooth)
{
  return (C*x_smooth+H).map(g);
};

void InvertMotorNStep::getLastMotors(motor* motors, int len)
{
  const Matrix& y = y_buffer[(t-1)%buffersize];
  y.convertToBuffer(motors, len);
}

void InvertMotorNStep::getLastSensors(sensor* sensors, int len)
{
  const Matrix& x = x_buffer[t%buffersize];
  x.convertToBuffer(sensors, len);
}


void InvertMotorNStep::setSensorWeights(const Matrix& weights)
{
  assert(initialised);
  if(weights.getM()!=number_sensors || weights.getN()!=1)
  {
    fprintf(stderr,"InvertMotorNStep::setSensorWeights(): Matrix with wrong dimensions\n");
    return;
  }
  sensorweights=weights;
}


Matrix InvertMotorNStep::calcDerivatives(const matrix::Matrix* buffer,int delay)
{
  const Matrix& xt    = buffer[(t-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

void InvertMotorNStep::management()
{
  if(dampA)
  {
    A *= 1 - dampA * managementInterval;
    B *= 1 - dampA * managementInterval;
  }
  if(dampS)
  {
    S *= 1 - dampS * managementInterval;
    SD *= 1 - dampS * managementInterval;
  }
  if(inhibition)
  {
    kwtaInhibition(C,max((int)kwta,1),inhibition*managementInterval*epsC);
  }
  else if(limitRF)
  {
    limitC(C,max(1,(int)limitRF));
  }
}

void InvertMotorNStep::kwtaInhibition(matrix::Matrix& wm, unsigned int k, double damping)
{
  unsigned int n = wm.getN();
  unsigned int k1 = std::min(n,k); // avoid overfloats
  double inhfactor = 1-damping;
  //  double exfactor  = 1+(damping/k1);
  for(unsigned int i=0; i < wm.getM(); i++)
  {
    Matrix r = wm.row(i).map(fabs);
    double x = getKthLargestElement(r,k1);
    for(unsigned int j=0;j< n; j++)
    {
      if( fabs(wm.val(i,j)) < x)
      {
        wm.val(i,j)*= inhfactor;
      } //else {
      // double d = m - abs_of_elem;  // allways possitive
      //        wm.val(i,j)*= 1+(damping*d); // scale exhitation by distance to max
      //      }
    }
  }
}

void InvertMotorNStep::limitC(matrix::Matrix& wm, unsigned int rfSize)
{
  int n = wm.getN();
  int m = wm.getM();
  for(int i=0; i < m; i++)
  {
    for(int j=0; j < n; j++)
    {
      if(std::abs(i-j) > (int)rfSize-1) wm.val(i,j)= 0;
    }
  }
}


bool InvertMotorNStep::store(FILE* f) const
{
  // save matrix values
  C.store(f);
  H.store(f);
  A.store(f);
  B.store(f);
  if(conf.useS) S.store(f);
  if(conf.useSD) SD.store(f);
  Configurable::print(f,0);
  return true;
}

bool InvertMotorNStep::restore(FILE* f)
{
  // save matrix values
  C.restore(f);
  H.restore(f);
  A.restore(f);
  B.restore(f);
  if(conf.useS) S.restore(f);
  if(conf.useSD) SD.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

list<Inspectable::ILayer> InvertMotorNStep::getStructuralLayers() const
{
  list<Inspectable::ILayer> l;
  l+=ILayer("x","", number_sensors, 0, "Sensors");
  l+=ILayer("y","H", number_motors, 1, "Motors");
  l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> InvertMotorNStep::getStructuralConnections() const
{
  list<Inspectable::IConnection> l;
  l+=IConnection("C", "x", "y");
  l+=IConnection("A", "y", "xP");
  if(conf.useS) l+=IConnection("S", "x", "xP");
  return l;
}

double InvertMotorNStep::clip095(double x)
{
  return clip(x,-0.95,0.95);
}

void InvertMotorNStep::setMotorTeachingSignal(const motor* teaching, int len)
{
  assert(len == number_motors);
  y_teaching.set(len, 1, teaching);
  y_teaching.toMap(clip095);
  useTeaching=true;
}

void InvertMotorNStep::setSensorTeachingSignal(const sensor* teaching, int len)
{
  assert(len == number_sensors);
  Matrix x_teaching(len,1,teaching);
  // calculate the y_teaching, that belongs to the distal teaching value by the inverse model.
  y_teaching = (A.multTM()^(-1)) *  ((A^T) * (x_teaching-B)) ;
  y_teaching.toMap(clip095);
  useTeaching=true;
}


///////// New Teaching interface:


void InvertMotorNStep::setMotorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_motors && teaching.getN() == 1);
  // Note: through the clipping the otherwise effectless
  //  teaching with old motor value has now an effect,
  //  namely to drive out of the saturation region.
  y_teaching= teaching.mapP(0.95,clip);
  useTeaching=true;
}

void InvertMotorNStep::setSensorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_sensors && teaching.getN() == 1);
  // calculate the y_teaching, that belongs to the distal teaching value by the inverse model.
  y_teaching = (A.pseudoInverse(0.001) * (teaching-B)).mapP(0.95, clip);
  useTeaching=true;
}

matrix::Matrix InvertMotorNStep::getLastMotorValues(){
  return y_buffer[(t-1+buffersize)%buffersize];
}

matrix::Matrix InvertMotorNStep::getLastSensorValues(){
  return x_buffer[(t-1+buffersize)%buffersize];
}


void InvertMotorNStep::setReinforcement(double reinforcement)
{
  this->reinforcement= tanh(reinforcement);
  this->reinforcefactor = 1-0.99*this->reinforcement;
}

