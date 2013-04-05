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
#ifndef __FFNNCONTROLLER_H
#define __FFNNCONTROLLER_H

#include "abstractcontroller.h"
#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/multilayerffnn.h>

/**
 * class for robot controller with a fixed neural network
 */
class FFNNController : public AbstractController {

public:
  /** @param networkfilename file to load the network
      @param history  number of time steps the network gets input (in sense of dimension of input)
      @param input_only_x if true then the input vector is \f[ (x_{t},x_{t-1},...,x_{t-history})^T \f]
       if false then also the y values are used: \f[ (x_{t}, y_{t-1}, x_{t-1},y_{t-2},...,x_{t-history})^T \f]
      @param init_wait number of timesteps to wait before controlling
  */
  FFNNController(const std::string& networkfilename, int history, bool input_only_x, unsigned int init_wait=0);

  /** @param net pointer to network (it must have the right dimensions)
      @param history  number of time steps the network gets input (in sense of dimension of input)
      @param input_only_x if true then the input vector is \f[ (x_{t},x_{t-1},...,x_{t-history})^T \f]
       if false then also the y values are used: \f[ (x_{t}, y_{t-1}, x_{t-1},y_{t-2},...,x_{t-history})^T \f]
      @param init_wait number of timesteps to wait before controlling
  */
  FFNNController(MultiLayerFFNN* net, int history, bool input_only_x, unsigned int init_wait=0);

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~FFNNController();

  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; }

  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  /**** CONFIGURABLE ****/
  void notifyOnChange(const paramkey& key);

  /**** STOREABLE ****/
  /** stores the controller values to a given file (binary).  */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file (binary). */
  virtual bool restore(FILE* f);

  // inspectable interface
  virtual std::list<iparamkey> getInternalParamNames()const  { return std::list<iparamkey>(); }
  virtual std::list<iparamval> getInternalParams() const { return std::list<iparamval>(); }

protected:
  void putInBuffer(matrix::Matrix* buffer, const matrix::Matrix& vec, int delay = 0);

  matrix::Matrix calculateSmoothValues(const matrix::Matrix* buffer, int number_steps_for_averaging_) const;

  virtual matrix::Matrix assembleNetworkInputXY(matrix::Matrix* xbuffer, matrix::Matrix* ybuffer) const;

  virtual matrix::Matrix assembleNetworkInputX(matrix::Matrix* xbuffer, matrix::Matrix* ybuffer) const;

  virtual matrix::Matrix assembleNetworkOutput(const matrix::Matrix& output) const;


protected:
  unsigned short number_motors;
  unsigned short number_sensors;
  unsigned short history;
  unsigned short buffersize;
  bool input_only_x;
  int s4avg;
  unsigned int t;
  unsigned int init_wait;

  matrix::Matrix* x_buffer;
  matrix::Matrix* y_buffer;
  matrix::Matrix x_smooth;

  MultiLayerFFNN* net;
  bool initialised;

};

#endif
