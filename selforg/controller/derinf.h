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
#ifndef __DERINF_H
#define __DERINF_H

#include "invertmotorcontroller.h"

#include <assert.h>
#include <math.h>

#include "matrix.h"
#include "multilayerffnn.h"
#include "noisegenerator.h"
#include "invertablemodel.h"

typedef struct DerInfConf {
  int buffersize;  ///< buffersize size of the time-buffer for x,y,eta
  double cInit;    ///< cInit size of the C matrix to initialised with.
  double cNonDiag; ///< cNonDiag is the size of the nondiagonal elements in respect to the diagonal (cInit) ones
  bool modelInit;  ///< size of the unit-map strenght of the model
  bool useS;    ///< useS decides whether to use the S matrix in addition to the A matrix
  bool someInternalParams;  ///< someInternalParams if true only some internal parameters are exported, otherwise all
} DerInfConf;
/**
 * class for robot controller is based on InvertMotorNStep
 *
 * - direct inversion
 *
 * - motor space
 *
 * - multilayer,nonlinear model
 */
class DerInf : public InvertMotorController {

public:
  DerInf(const DerInfConf& conf = getDefaultConf());
  virtual void init(int sensornumber, int motornumber, RandGen* randg);

  virtual ~DerInf();

  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; }

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);

  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  /**************  STOREABLE **********************************/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  /************** INSPECTABLE ********************************/
  virtual iparamkeylist getInternalParamNames() const;
  virtual iparamvallist getInternalParams() const;
  virtual ilayerlist getStructuralLayers() const;
  virtual iconnectionlist getStructuralConnections() const;


  static DerInfConf getDefaultConf(){
    DerInfConf c;
    c.buffersize = 50;
    c.cInit = 1.05;
    c.cNonDiag = 0;
    c.modelInit  = 1.0;
    c.someInternalParams = true;
     //   c.someInternalParams = false;
    c.useS = false;
    return c;
  }

  void getLastMotors(motor* motors, int len);

protected:
  unsigned short number_sensors;
  unsigned short number_motors;

  matrix::Matrix A; ///< Model Matrix (motors to sensors)
  matrix::Matrix A0;
  matrix::Matrix A_Hat; ///< Model Matrix (motors to sensors) with input shift
  matrix::Matrix S; ///< additional Model Matrix (sensors to sensors)
  matrix::Matrix C; ///< Controller Matrix
  matrix::Matrix GSC; ///< G_Prime times Controller Matrix
  matrix::Matrix DD; ///< Noise  Matrix
  matrix::Matrix DD1; ///< Noise  Matrix
  matrix::Matrix Dinverse; ///< Inverse  Noise  Matrix
  matrix::Matrix H; ///< Controller Bias
  matrix::Matrix HY; ///< Controller Bias-Y
  matrix::Matrix B; ///< Model Bias
  NoiseGenerator* BNoiseGen; ///< Noisegenerator for noisy bias
  NoiseGenerator* YNoiseGen; ///< Noisegenerator for noisy motor values
  matrix::Matrix R; ///< C*A
  matrix::Matrix P; ///< A*G'*C
  matrix::Matrix RG; ///< Granger1
  matrix::Matrix Q; ///<Granger2
  matrix::Matrix Q1; //<Granger3
  matrix::Matrix RRT_inv; // (R*R^T)^-1
  matrix::Matrix ATA_inv; // ((A^T)*A)^-1
  matrix::Matrix Rm1; ///< R^-1
  matrix::Matrix ID; ///< identity matrix in the dimension of R
  matrix::Matrix ID_Sensor; ///< identity matrix in the dimension of sensor space
  matrix::Matrix CCT_inv;
  matrix::Matrix CST;
  matrix::Matrix xsi; ///< current output error
  double xsi_norm; ///< norm of matrix
  double xsi_norm_avg; ///< average norm of xsi (used to define whether Modell learns)
  double pain;         ///< if the modelling error (xsi) is too high we have a pain signal
  double TLE; // TimeLoopError
  double grang1; //GrangerCausality
  double grang2; //GrangerCausality
  double causal; //GrangerCausality
  double causalfactor; //GrangerCausality
  double EE;
  double EE_mean;
  double EE_sqr;
  double xsistrength;
  double sense;
  matrix::Matrix* x_buffer;
  matrix::Matrix* y_buffer;
  matrix::Matrix* ysat_buffer;
  matrix::Matrix* chi_buffer;
  matrix::Matrix* rho_buffer;
  matrix::Matrix* eta_buffer;
  matrix::Matrix* xsi_buffer;
  matrix::Matrix eta;
  matrix::Matrix ups;
  matrix::Matrix v_smooth;
  matrix::Matrix zero_eta; // zero initialised eta
  matrix::Matrix x_smooth;
  matrix::Matrix y_smooth;
  matrix::Matrix y_sat;
  matrix::Matrix y_smooth_long;
  matrix::Matrix eta_smooth;
  matrix::Matrix x_smooth_long;
  matrix::Matrix y_forecast;
  matrix::Matrix y_integration;
  matrix::Matrix y_error;
  matrix::Matrix y_intern;
  matrix::Matrix PID_deriv;
  matrix::Matrix C_updateOld;
  matrix::Matrix H_updateOld;
  matrix::Matrix HY_updateOld;
  matrix::Matrix A_updateOld;
  matrix::Matrix B_updateOld;
  matrix::Matrix L;
  matrix::Matrix G;
  matrix::Matrix xx;
  matrix::Matrix yy;
  matrix::Matrix zz;

  matrix::Matrix vau_avg;


  matrix::Matrix y_teaching; ///< teaching motor signal
  bool useTeaching; ///< flag whether there is an actual teachning signal or not

  matrix::Matrix x_intern;
  int num_iterations;

  int t_rand; ///< initial random time to avoid syncronous management of all controllers
  int t_delay;
  int managementInterval; ///< interval between subsequent management function calls
  paramval dampS;     ///< damping of S matrix
  paramval dampC;     ///< damping of C matrix
  paramval dampH;     ///< damping of H vector
  paramval weighting;     ///< general weighting factor between update concepts
  paramval epsSat;    ///< learning rate for satellite network
  paramval satelliteTeaching; ///< teaching rate for sat teaching
  paramval PIDint;
  paramval PIDdrv;
  paramval intstate;
  paramval gamma;
  paramval adaptrate;
  paramval xsifactor;
  paramval creat;

  DerInfConf conf;

  /// puts the sensors in the ringbuffer, generate controller values and put them in the
  //  ringbuffer as well
  virtual void fillBuffersAndControl(const sensor* x_, int number_sensors,
                             motor* y_, int number_motors);

/** learn values H,C
    This is the implementation uses a better formula for g^-1 using Mittelwertsatz
    @param delay 0 for no delay and n>0 for n timesteps delay in the SML (s4delay)
*/
  virtual void learnController(int delay);

  /// handles inhibition damping etc.
  virtual void management();

  /// returns controller output for given sensor values
  virtual matrix::Matrix calculateControllerValues(const matrix::Matrix& x_smooth);

  /** Calculates first and second derivative and returns both in on matrix (above).
      We use simple discrete approximations:
      \f[ f'(x) = (f(x) - f(x-1)) / 2 \f]
      \f[ f''(x) = f(x) - 2f(x-1) + f(x-2) \f]
      where we have to go into the past because we do not have f(x+1). The scaling can be neglegted.
  */
  matrix::Matrix calcDerivatives(const matrix::Matrix* buffer, int delay);

public:

  /// calculates the city block distance (abs) norm of the matrix. (abs sum of absolutes / size of matrix)
     virtual double calcMatrixNorm(const matrix::Matrix& m);


};

#endif
