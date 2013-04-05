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
#ifndef __DERCONTROLLER_H
#define __DERCONTROLLER_H

#include "invertmotorcontroller.h"

#include <assert.h>
#include <cmath>

#include "matrix.h"
#include "noisegenerator.h"

typedef struct DerControllerConf {
  int buffersize; ///< buffersize size of the time-buffer for x,y,eta
  double cInit; ///< cInit size of the C matrix to initialised with.
  double cNonDiag; ///< cNonDiag is the size of the nondiagonal elements in respect to the diagonal (cInit) ones
  bool useS;    ///< useS decides whether to use the S matrix in addition to the A matrix
  bool someInternalParams;  ///< someInternalParams if true only some internal parameters are exported, all otherwise
  bool useTeaching;         ///< if true, the controller honors the teaching signal
  bool useFantasy;           ///< if true fantasising is enabled
} DerControllerConf;

/**
 * class for robot controller that uses the georg's matrixlib for
 *  direct matrix inversion for n channels
 * (simple one layer networks)
 *
 * Implements standart parameters: eps, rho, mu, stepnumber4avg, stepnumber4delay
 */
class DerController : public InvertMotorController {

public:

  DerController(const DerControllerConf& conf = getDefaultConf());
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~DerController();

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


  /**** STOREABLE ****/
  virtual bool store(FILE* f) const;
  virtual bool restore(FILE* f);

  /**** CONFIGURABLE ****/
  virtual std::list<iparamkey> getInternalParamNames() const;
  virtual std::list<iparamval> getInternalParams() const;
  virtual std::list<ILayer> getStructuralLayers() const;
  virtual std::list<IConnection> getStructuralConnections() const;

  /**** TEACHING ****/
  virtual void setTeachingMode(bool onOff);
  virtual bool getTeachingMode();
  virtual void setMotorTeachingSignal(const motor* teaching, int len);
  //void calcCandHUpdatesTeaching(Matrix& C_update, Matrix& H_update, int y_delay);
  //void calcCandHUpdates(Matrix& C_update, Matrix& H_update,Matrix& A_update, int y_delay);//Test A

  static DerControllerConf getDefaultConf(){
    DerControllerConf c;
    c.buffersize = 50;
    c.cInit = 1.2;
    c.cNonDiag = 0;
    c.useS  = false;
    //c.someInternalParams = true;//This is for gnuplout, only the first few nodiagonal elements
    c.someInternalParams = false;//This is for gnuplout,to plot all matrix  elements
    c.useTeaching = false;
    c.useFantasy = false;
    return c;
  }

  void getLastMotors(motor* motors, int len);

protected:
  unsigned short number_sensors;
  unsigned short number_motors;

  matrix::Matrix A; ///< Model Matrix (motors to sensors)
  matrix::Matrix S; ///< additional Model Matrix (sensors to sensors)
  matrix::Matrix C; ///< Controller Matrix
  matrix::Matrix DD; ///< Noise  Matrix
  matrix::Matrix Dinverse; ///< Inverse  Noise  Matrix
  matrix::Matrix H; ///< Controller Bias
  matrix::Matrix B; ///< Model Bias
  NoiseGenerator* BNoiseGen; ///< Noisegenerator for noisy bias
  NoiseGenerator* YNoiseGen; ///< Noisegenerator for noisy motor values
  matrix::Matrix R; ///< C*A
  matrix::Matrix RRT; // R*R^T
  matrix::Matrix AAT; // (A^T)*A
  //  matrix::Matrix Rm1; ///< R^-1
  matrix::Matrix SmallID; ///< small identity matrix in the dimension of R
  matrix::Matrix xsi; ///< current output error
  double xsi_norm; ///< norm of matrix
  double xsi_norm_avg; ///< average norm of xsi (used to define whether Modell learns)
  double pain;         ///< if the modelling error (xsi) is too high we have a pain signal
  matrix::Matrix* x_buffer;
  matrix::Matrix* y_buffer;
  matrix::Matrix* eta_buffer;
  matrix::Matrix zero_eta; // zero initialised eta
  matrix::Matrix x_smooth;
  //  matrix::Matrix v_smooth;
   matrix::Matrix eta_smooth;


  matrix::Matrix y_teaching; ///< teaching motor signal

  matrix::Matrix x_intern;  ///< fantasy sensor values
  int fantControl;     ///< interval length for fantasising
  int fantControlLen;  ///< length of fantasy control
  int fantReset;       ///< number of fantasy control events before reseting internal state

  DerControllerConf conf;

  /// puts the sensors in the ringbuffer, generate controller values and put them in the
  //  ringbuffer as well
  virtual void fillBuffersAndControl(const sensor* x_, int number_sensors,
                             motor* y_, int number_motors);

  /// calculates the first shift into the motor space useing delayed motor values.
  //  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
  virtual void calcEtaAndBufferIt(int delay);
  /// calculates xsi for the current time step using the delayed y values
  //  and x delayed by one
  //  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
  virtual void calcXsi(int delay);

  /// learn H,C with motors y and corresponding sensors x
  virtual void learnController();

  /// calculates the predicted sensor values
  virtual matrix::Matrix model(const matrix::Matrix& x, const matrix::Matrix& y);

  /// calculates the Update for C, H and A
  // @param y_delay timesteps to delay the y-values.  (usually 0)
  //  Please note that the delayed values are NOT used for the error calculation
  //  (this is done in calcXsi())
  virtual void calcCandHandAUpdates(matrix::Matrix& C_update, matrix::Matrix& H_update,
                                    matrix::Matrix& A_update, int y_delay);//Test A
  /// updates the matrices C, H and A
  virtual void updateCandHandA(const matrix::Matrix& C_update, const matrix::Matrix& H_update,
                               const matrix::Matrix& A_update, double squashSize);//Test A

  virtual matrix::Matrix calculateControllerValues(const matrix::Matrix& x_smooth);

  /// calculates the city block distance (abs) norm of the matrix. (abs sum of absolutes / size of matrix)
  virtual double calcMatrixNorm(const matrix::Matrix& m);
  /// calculates the error_factor for either logarithmic (E=ln(e^T*e)) or square (E=sqrt(e^t*e)) error
  virtual double calcErrorFactor(const matrix::Matrix& e, bool loga, bool root);

};

#endif
