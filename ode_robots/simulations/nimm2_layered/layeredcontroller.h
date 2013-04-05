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
 *   Revision 1.3  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.2  2009/08/05 23:25:57  martius
 *   adapted small things to compile with changed Plotoptions
 *
 *   Revision 1.1  2009/04/22 14:39:02  guettler
 *   moved layeredcontroller, layer2_incc and layer1_incc to ode_robots/simulations/nimm2_hebb and nimm2_layered
 *
 *   Revision 1.4  2008/05/30 11:58:27  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.3  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.2  2007/10/10 08:44:27  fhesse
 *   testing
 *
 *   Revision 1.1  2007/10/08 20:15:33  fhesse
 *   initial version of layered controller
 *
 *                                    *
 *                                                                         *
 ***************************************************************************/
#ifndef __LAYEREDCONTROLLER_H
#define __LAYEREDCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <stdlib.h>
#include <string.h>

#include <selforg/controller_misc.h>
#include "layer1_incc.h"
#include "layer2_incc.h"

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
class LayeredController : public AbstractController {

public:
  LayeredController(int _buffersize, bool _update_only_1=false);
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~LayeredController();

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


  virtual paramval getParam(const paramkey& key, bool raverseChildren=true) const;
  virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren=true);
  virtual paramlist getParamList() const;

protected:

  Layer1_INCC* layer1;
  Layer2_INCC* layer2;

  unsigned short number_channels;
  unsigned short buffersize;
  bool update_only_1;

  matrix::Matrix A; // Model Matrix
  matrix::Matrix C; // Controller Matrix
  matrix::Matrix h; // Controller Bias
  matrix::Matrix L; // Jacobi Matrix
  matrix::Matrix* x_buffer;
  matrix::Matrix* y_buffer;
  paramkey name;


  matrix::Matrix lay2_sensors;
  matrix::Matrix lay2_motors;


  // step conter
  int t;



};

#endif


