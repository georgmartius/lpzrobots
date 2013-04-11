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
#ifndef __DERBIGCONTROLLER_H
#define __DERBIGCONTROLLER_H

#include "invertmotorcontroller.h"

#include <assert.h>
#include <cmath>

#include "matrix.h"
#include "noisegenerator.h"
#include "invertablemodel.h"

typedef struct DerBigControllerConf {
  int buffersize;  ///< buffersize size of the time-buffer for x,y,eta
  double cInit;    ///< cInit size of the C matrix to initialised with.
  double cNonDiag; ///< cNonDiag is the size of the nondiagonal elements in respect to the diagonal (cInit) ones
  bool modelInit;  ///< size of the unit-map strenght of the model
  bool useS;    ///< useS decides whether to use the S matrix in addition to the A matrix
  bool someInternalParams;  ///< someInternalParams if true only some internal parameters are exported, otherwise all

  double modelCompliant; ///< learning factor for model (or sensor) compliant learning
  bool useFantasy;           ///< if true fantasising is enabled

  InvertableModel* model;   ///< model used as world model
} DerBigControllerConf;

/**
 * class for robot controller is based on InvertMotorNStep
 *
 * - direct inversion
 *
 * - motor space
 *
 * - multilayer,nonlinear model
 */
class DerBigController : public InvertMotorController {

public:
  DerBigController(const DerBigControllerConf& conf = getDefaultConf());
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~DerBigController();

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

  /************** CONFIGURABLE ********************************/
  virtual void notifyOnChange(const paramkey& key);

  /**** TEACHING ****/
  /** The given motor teaching signal is used for this timestep.
      It is used as a feed forward teaching signal for the controller.
      Please note, that the teaching signal has to be given each timestep
       for a continuous teaching process.
   */
  virtual void setMotorTeachingSignal(const motor* teaching, int len);

  /** The given sensor teaching signal (distal learning) is used for this timestep.
      First the belonging motor teachung signal is calculated by the inverse model.
      See setMotorTeachingSignal
   */
  virtual void setSensorTeachingSignal(const sensor* teaching, int len);


  static DerBigControllerConf getDefaultConf(){
    DerBigControllerConf c;
    c.buffersize = 50;
    c.cInit = 1.05;
    c.cNonDiag = 0;
    c.modelInit  = 1.0;
     c.someInternalParams = true;
     //   c.someInternalParams = false;
    c.useS = false;
    c.modelCompliant = 0;
    c.model = 0;
    c.useFantasy = false;
    return c;
  }

  void getLastMotors(motor* motors, int len);

protected:
  unsigned short number_sensors;
  unsigned short number_motors;

  matrix::Matrix A; ///< Model Matrix (motors to sensors)
  matrix::Matrix A_Hat; ///< Model Matrix (motors to sensors) with input shift
  matrix::Matrix S; ///< additional Model Matrix (sensors to sensors)
  matrix::Matrix C; ///< Controller Matrix
  matrix::Matrix GSC; ///< G_Prime times Controller Matrix
  matrix::Matrix DD; ///< Noise  Matrix
  matrix::Matrix Dinverse; ///< Inverse  Noise  Matrix
  matrix::Matrix H; ///< Controller Bias
  matrix::Matrix B; ///< Model Bias
  NoiseGenerator* BNoiseGen; ///< Noisegenerator for noisy bias
  NoiseGenerator* YNoiseGen; ///< Noisegenerator for noisy motor values
  matrix::Matrix R; ///< C*A
  matrix::Matrix RRT_inv; // (R*R^T)^-1
  matrix::Matrix ATA_inv; // ((A^T)*A)^-1
  matrix::Matrix Rm1; ///< R^-1
  matrix::Matrix ID; ///< identity matrix in the dimension of R
  matrix::Matrix ID_Sensor; ///< identity matrix in the dimension of sensor space
  matrix::Matrix xsi; ///< current output error
  double xsi_norm; ///< norm of matrix
  double xsi_norm_avg; ///< average norm of xsi (used to define whether Modell learns)
  double pain;         ///< if the modelling error (xsi) is too high we have a pain signal
  matrix::Matrix* x_buffer;
  matrix::Matrix* y_buffer;
  matrix::Matrix* eta_buffer;
  matrix::Matrix eta;
  matrix::Matrix v_smooth;
  matrix::Matrix zero_eta; // zero initialised eta
  matrix::Matrix x_smooth;
  matrix::Matrix y_smooth;
  matrix::Matrix eta_smooth;
  matrix::Matrix x_smooth_long;

  matrix::Matrix y_teaching; ///< teaching motor signal
  bool useTeaching; ///< flag whether there is an actual teachning signal or not

  matrix::Matrix x_intern;  ///< fantasy sensor values
  int fantControl;     ///< interval length for fantasising
  int fantControlLen;  ///< length of fantasy control
  int fantReset;       ///< number of fantasy control events before reseting internal state

  int t_rand; ///< initial random time to avoid syncronous management of all controllers
  int managementInterval; ///< interval between subsequent management function calls
  paramval inhibition; ///< inhibition strength for sparce kwta strategy (is scaled with epsC)
  paramval kwta;       ///< (int) number of synapses that get strengthend
  paramval limitRF;    ///< (int) receptive field of motor neurons (number of offcenter sensors) if null then no limitation. Mutual exclusive with inhibition
  paramval dampS;     ///< damping of S matrix
  paramval dampC;     ///< damping of C matrix
  paramval dampH;     ///< damping of H vector
  paramval weighting;     ///< general weighting fator between update concepts

  DerBigControllerConf conf;

  /// puts the sensors in the ringbuffer, generate controller values and put them in the
  //  ringbuffer as well
  virtual void fillBuffersAndControl(const sensor* x_, int number_sensors,
                             motor* y_, int number_motors);

/** learn values H,C
    This is the implementation uses a better formula for g^-1 using Mittelwertsatz
    @param delay 0 for no delay and n>0 for n timesteps delay in the SML (s4delay)
*/
  virtual void learnController(int delay);

  /// learn conf.model, (and S) using motors y and corresponding sensors x
  //  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
  virtual void learnModel(int delay);

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
  /** k-winner take all inhibition for synapses. k largest synapses are strengthed and the rest are inhibited.
      strong synapes are scaled by 1+(damping/k) and weak synapses are scaled by 1-(damping/(n-k)) where n is the
      number of synapes
      @param weightmatrix reference to weight matrix. Synapses for a neuron are in one row.
             The inhibition is done for all rows independently
      @param k number of synapes to strengthen
      @param damping strength of supression and exitation (typically 0.001)
   */
  void kwtaInhibition(matrix::Matrix& weightmatrix, unsigned int k, double damping);

  /** sets all connections to zero which are further away then rfSize.
      If rfSize == 1 then only main diagonal is left.
      If rfSize = 2: main diagonal and upper and lower side diagonal are kept and so on and so forth.
   */
  void limitC(matrix::Matrix& weightmatrix, unsigned int rfSize);


  /// calculates the city block distance (abs) norm of the matrix. (abs sum of absolutes / size of matrix)
     virtual double calcMatrixNorm(const matrix::Matrix& m);

};

#endif
