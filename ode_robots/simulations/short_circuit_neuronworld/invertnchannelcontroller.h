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
 *                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-12-04 18:51:59  fhesse
 *   invertnchannelcontroller has bias (changeable in constructor) now
 *   neuronworld has linear neuron now (changeable in conf)
 *
 *   Revision 1.1  2009/09/22 08:21:49  fhesse
 *   world is a schmitt trigger neuron
 *   only 1 DOF so far
 *
 *   Revision 1.21  2008/05/30 11:58:27  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.20  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.19  2006/12/11 18:14:14  martius
 *   delete for BNoise
 *   delete of buffers
 *
 *   Revision 1.18  2006/08/02 09:31:34  martius
 *   removed TODO
 *
 *   Revision 1.17  2006/07/20 17:14:35  martius
 *   removed std namespace from matrix.h
 *   storable interface
 *   abstract model and invertablemodel as superclasses for networks
 *
 *   Revision 1.16  2006/07/14 12:23:58  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.14.6.4  2006/07/10 13:05:16  martius
 *   NON-COMMERICAL LICENSE added to controllers
 *
 *   Revision 1.14.6.3  2006/07/10 11:59:24  martius
 *   Matrixlib now in selforg
 *   no namespace std in header files
 *
 *   Revision 1.14.6.2  2006/01/18 16:48:35  martius
 *   stored and restore
 *
 *   Revision 1.14.6.1  2005/11/14 17:37:29  martius
 *   moved to selforg
 *
 *   Revision 1.14  2005/10/27 15:46:38  martius
 *   inspectable interface is expanded to structural information for network visualiser
 *
 *   Revision 1.13  2005/10/27 15:02:06  fhesse
 *   commercial use added
 *                                    *
 *                                                                         *
 ***************************************************************************/
#ifndef __INVERTNCHANNELCONTROLLER_H
#define __INVERTNCHANNELCONTROLLER_H

#include "selforg/invertcontroller.h"
#include "selforg/controller_misc.h"

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>

/**
 * class for robot controller that uses the georg's matrixlib for
 *  direct matrix inversion for n channels
 * (simple one layer networks)
 *
 * Implements standart parameters: eps, rho, mu, stepnumber4avg, stepnumber4delay
 */
class InvertNChannelController : public InvertController {

public:

  enum ModelNeuronProperties{nobias, bias};

  InvertNChannelController(int _buffersize, bool _update_only_1=false, ModelNeuronProperties _model_type=nobias);
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~InvertNChannelController();

  /// returns the name of the object (with version number)
  virtual paramkey getName() const {return name; }
  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_channels; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_channels; }

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);


  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);


  /***** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  // inspectable interface
  virtual std::list<iparamkey> getInternalParamNames() const;
  virtual std::list<iparamval> getInternalParams() const;
  virtual std::list<ILayer> getStructuralLayers() const;
  virtual std::list<IConnection> getStructuralConnections() const;



protected:
  unsigned short number_channels;
  unsigned short buffersize;
  bool update_only_1;

  matrix::Matrix A; // Model Matrix
  matrix::Matrix s; // Model Bias
  matrix::Matrix C; // Controller Matrix
  matrix::Matrix h; // Controller Bias
  matrix::Matrix L; // Jacobi Matrix
  matrix::Matrix* x_buffer;
  matrix::Matrix* y_buffer;
  int t;
  paramkey name;

  ModelNeuronProperties model_type; // type of neuron used in internal model (so far: use bias or do not use bias)

/*   virtual void iteration(double *column, */
/*                          double dommy[NUMBER_CHANNELS][NUMBER_CHANNELS], */
/*                          double *improvment); */

  virtual double calculateE(const matrix::Matrix& x_delay, const matrix::Matrix& y_delay);

  /// learn values h,C
  virtual void learn(const matrix::Matrix& x_delay, const matrix::Matrix& y_delay);

  virtual void learnmodel( const matrix::Matrix& y_delay);

  /// calculate delayed values
  virtual matrix::Matrix calculateDelayedValues(const matrix::Matrix* buffer,
                                        unsigned int number_steps_of_delay_);
  virtual matrix::Matrix calculateSmoothValues(const matrix::Matrix* buffer,
                                       unsigned int number_steps_for_averaging_);

  matrix::Matrix calculateControllerValues(const matrix::Matrix& x_smooth);

  // put new value in ring buffer
  void putInBuffer(matrix::Matrix* buffer, const matrix::Matrix& vec);

  /// neuron transfer function
  static double g(double z)
  {
    return tanh(z);
  };

  ///
  static double g_s(double z)
  {
    double k=tanh(z);
    return 1.0 - k*k;
    //    return 1.0 - tanh(z)*tanh(z);
  };



  /// squashing function, to protect against to large weight updates
  static double squash(double z)
  {
    return clip(z,-0.1,0.1);
    //    return z < -0.1 ? -0.1 : ( z > 0.1 ? 0.1 : z );
    //return 0.1 * tanh(10.0 * z);
  };
};

#endif


