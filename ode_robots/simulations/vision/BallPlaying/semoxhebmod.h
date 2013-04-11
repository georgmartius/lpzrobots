/***************************************************************************
 *   Copyright (C) 2009 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
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
 *   This controller implements the extensions described in the thesis     *
 *   of Georg Martius: 2009, University Goettingen                         *
 *   Goal-Oriented Control of Self-organizing Behavior in Autonomous Robots*
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2010-09-16 15:20:35  martius
 *   simulation improved
 *
 *   Revision 1.1  2010/03/29 16:26:45  martius
 *   new simulation for Ball playing
 *
 *   Revision 1.3  2010/01/26 09:42:57  martius
 *   restore bug fixed
 *   ThisSim is a friend class (this is a hack)
 *
 *   Revision 1.2  2009/08/07 14:43:25  martius
 *   initialC and cNonDiag added
 *
 *   Revision 1.1  2009/08/05 23:08:21  martius
 *   new reduced motor space controller implementing extensions
 *    described in georgs dissertation.
 *
 *
 *
 ***************************************************************************/
#ifndef __SEMOXHEBMOD_H
#define __SEMOXHEBMOD_H

#include <selforg/homeokinbase.h>
#include <selforg/matrix.h>
#include <selforg/teachable.h>
#include <selforg/noisegenerator.h>
#include <selforg/randomgenerator.h>


typedef struct SeMoXHebModConf {
  int buffersize;   ///< buffersize size of the time-buffer for x,y,eta
  matrix::Matrix initialC; ///< initialC initial controller matrix (if null matrix then automatic, see cInit)
  double cInit;     ///< cInit initial size of the diagonals of the C matrix (if C is not given)
  double cNonDiag;  ///< cNonDiag initial size of the non-diagonal elements of the C matrix (if C is not given)
  double aInit;     ///< aInit initial size of the diagonals of the A matrix
  double sInit;     ///< sInit initial size of the diagonals of the S matrix
  bool modelExt;    ///< modelExt if true then additional matrix S is used in world model (sees sensors)
  /** number of context sensors (considered at the end of the sensor
      vector, which are only feed to the model extended model */
  int numContext;
  bool someInternalParams;  ///< someInternalParams if true only some internal parameters are exported
} SeMoXHebModConf;

/**
 * This controller follows the prinziple of homeokinesis and
 *  implements the extensions described in the thesis of Georg Martius
 *  2009, University Goettingen:
 *  Goal-Oriented Control of Self-organizing Behavior in Autonomous Robots
 *
 * This implements an additional Hebbian model which is used for the distal learning only
 *
 * Name: SElf-organizing MOtor space eXtended
 *
 * Main characteristics: Motor Space, Extended World model, Continuity, Teaching interface
 *
 */
class SeMoXHebMod : public HomeokinBase, public Teachable {
  friend class ThisSim;
public:
  SeMoXHebMod(const SeMoXHebModConf& conf = getDefaultConf());

  /// returns the default configuration
  static SeMoXHebModConf getDefaultConf(){
    SeMoXHebModConf c;
    c.buffersize = 50;
    // c.initialC // remains 0x0
    c.cInit = 1.0;
    c.cNonDiag = 0;
    c.aInit = 1.0;
    c.sInit = 0.0;
    c.modelExt  = true;
    c.someInternalParams = true;
    c.numContext = 0;
    return c;
  }

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~SeMoXHebMod();

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
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  /**** INSPECTABLE ****/
  virtual std::list<ILayer> getStructuralLayers() const;
  virtual std::list<IConnection> getStructuralConnections() const;

  /**** TEACHABLE ****/
  /** The given motor teaching signal is used for this timestep.
      It is used as a feed forward teaching signal for the controller.
      Please note, that the teaching signal has to be given each timestep
       for a continuous teaching process.
     @param teaching: matrix with dimensions (motornumber,1)
   */
  virtual void setMotorTeaching(const matrix::Matrix& teaching);

  ////// TEST of HEBBIAN Model for teaching
  /** The given sensor teaching signal (distal learning)
      te difference between current and desired values is calculated
      and then converted to motor error using the hebbian model
     @param teaching: matrix with dimensions (sensornumber,1)
   */
  virtual void setSensorTeaching(const matrix::Matrix& teaching);
  /// returns the last motor values (useful for cross motor coupling)
  virtual matrix::Matrix getLastMotorValues();
  /// returns the last sensor values (useful for cross sensor coupling)
  virtual matrix::Matrix getLastSensorValues();


protected:
  unsigned short number_sensors;
  unsigned short number_motors;

  // NEW
  matrix::Matrix M; ///< Hebbian Model Matrix (motors to sensors)

  matrix::Matrix A; ///< Model Matrix (motors to sensors)
  matrix::Matrix S; ///< additional Model Matrix (sensors derivatives to sensors)
  matrix::Matrix C; ///< Controller Matrix
  matrix::Matrix H; ///< Controller Bias
  matrix::Matrix B; ///< Model Bias
  matrix::Matrix R; ///< C*A
  matrix::Matrix SmallID; ///< small identity matrix in the dimension of R
  matrix::Matrix v; ///< shift
  matrix::Matrix xsi; ///< current output error

  NoiseGenerator* BNoiseGen; ///< Noisegenerator for noisy bias
  paramval modelNoise;       ///< strength of noisy bias

  double xsi_norm;     ///< norm of matrix
  double xsi_norm_avg; ///< average norm of xsi (used to define whether Modell learns)
  double pain;         ///< if the modelling error (xsi) is too high we have a pain signal

  matrix::Matrix* x_buffer;
  matrix::Matrix* x_c_buffer; ///< buffer for sensors with context sensors
  matrix::Matrix* y_buffer;

  matrix::Matrix y_teaching; ///< motor teaching  signal

  paramval epsM;  ///< learnning rate for hebbian model

  paramval gamma_cont;  ///< parameter to include contiuity in motor values (avoid high frequencies)
  paramval gamma_teach; ///< strength of teaching
  paramval discountS;   ///< discount strength for hierachical model

  paramval dampModel;       ///< damping of A and S matrices
  paramval dampController;  ///< damping of C matrix

  SeMoXHebModConf conf;

  // internal
  bool intern_useTeaching; ///< flag whether there is an actual teachning signal or not
  int t_rand; ///< initial random time to avoid syncronous management of all controllers
  int managementInterval; ///< interval between subsequent management function calls
  parambool _modelExt_copy; ///< copy of modelExtension variable (to achieve readonly)

  /// puts the sensors in the ringbuffer, generate controller values and put them in the
  //  ringbuffer as well
  virtual void fillBuffersAndControl(const sensor* x_, int number_sensors,
                             motor* y_, int number_motors);

  /// calculates xsi for the current time step using the delayed y values
  //  and x delayed by one
  //  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
  virtual void calcXsi(int delay);

  /// learn H,C with motors y and corresponding sensors x
  virtual void learnController();

  /// learn A, (and S) using motors y and corresponding sensors x
  //  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
  virtual void learnModel(int delay);

  /// calculates the predicted sensor values
  virtual matrix::Matrix model(const matrix::Matrix* x_buffer, int delay, const matrix::Matrix& y);

  /// handles inhibition damping etc.
  virtual void management();

  /// returns controller output for given sensor values
  virtual matrix::Matrix calculateControllerValues(const matrix::Matrix& x_smooth);

protected:
  static double regularizedInverse(double v);


};

#endif
