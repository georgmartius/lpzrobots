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
#ifndef __MUTUALINFORMATIONCONTROLLER_H
#define __MUTUALINFORMATIONCONTROLLER_H

#include "abstractcontroller.h"
#include "matrix.h"

/**
 * This is a controller who is passive at the moment, that means, he will not
 * generate any motor values. This controller calculates the mutual information
 * from one to the next time step. Note that many steps are necessary for a good
 * prediction of the mutual information.
 */
class MutualInformationController : public AbstractController
{

public:

    static const int numberSensorsPreInitialized = 10;
    static const int numberMotorsPreInitialized  = 2;

  /**
   * Constructs the mutual information controller. At this time the
   * controller is not yet initialized (this does the Agent).
   * @param sensorIntervalCount is the number of the intervals used. This
   * is important for generating the internal matrices.
   * @param minSensorValue is the minimum value the sensors can become
   * @param maxSensorValue is the maximum value the sensors can become
   *
   */
  MutualInformationController(int sensorIntervalCount, double minSensorValue=-1, double maxSensorValue=1, bool showF =false, bool showP =false, bool showXsiF = false);
  virtual ~MutualInformationController() {};

  virtual double& getMI(int index) {
    if (!initialized)
      init(numberSensorsPreInitialized,numberMotorsPreInitialized);
    assert(index<sensorNumber);
    return MI[index];
  }

  virtual double& getH_x(int index) {
    if (!initialized)
      init(numberSensorsPreInitialized,numberMotorsPreInitialized);
    assert(index<sensorNumber);
    return H_x[index];
  }

  virtual double& getH_yx(int index) {
    if (!initialized)
      init(numberSensorsPreInitialized,numberMotorsPreInitialized);
    assert(index<sensorNumber);
    return H_yx[index];
  }

  virtual double& getH_Xsi(int index) {
    if (!initialized)
      init(numberSensorsPreInitialized,numberMotorsPreInitialized);
    assert(index<sensorNumber);
    return H_Xsi[index];
  }

  /**
  * we like to calculate the entropy of the xsi, therefore we need for the (self calculated) update rule
  * x_t+1=-a * tanh(c * x_t) the a and c, which should be set in the main.cpp.
  */
  virtual void setAandCandCalcH_xsi(double ainit, double cinit);


  /****************************************************************************/
  /*        BEGIN methods of AbstractController                                 */
  /****************************************************************************/

  /** initialisation of the controller with the given sensor/ motornumber
   * Must NORMALLY be called before use. For all ControllerAdapters
   * call first AbstractControllerAdapter::init(sensornumber,motornumber)
   * if you overwrite this method
   */
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  /** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
  virtual int getSensorNumber() const { return sensorNumber; }

  /** @return Number of motors the controller
      was initialised with or 0 if not initialised */
  virtual int getMotorNumber() const { return motorNumber; }

  /** performs one step (includes learning).
      Calculates motor commands from sensor inputs.
  @param sensors sensors inputs scaled to [-1,1]
  @param sensornumber length of the sensor array
  @param motors motors outputs. MUST have enough space for motor values!
  @param motornumber length of the provided motor array
  */
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber);

  /** performs one step without learning.
  @see step
  */
  virtual void stepNoLearning(const sensor* sensors , int sensornumber,
                              motor* motors, int motornumber);

  /****************************************************************************/
  /*        END methods of AbstractController                                   */
  /****************************************************************************/


  /****************************************************************************/
  /*        BEGIN methods of Inspectable                                                  */
  /****************************************************************************/

  /** The list of the names of all internal parameters given by getInternalParams().
      The naming convention is "v[i]" for vectors
       and "A[i][j]" for matrices, where i, j start at 0.
  @return: list of keys
   */
//  virtual iparamkeylist getInternalParamNames() const;

  /** @return: list of values
  */
//  virtual iparamvallist getInternalParams() const;

  /****************************************************************************/
  /*        END methods of Inspectable                                                   */
  /****************************************************************************/

  /****************************************************************************/
  /*        BEGIN methods of Storeable                                                   */
  /****************************************************************************/

  /** stores the object to the given file stream (binary).
  */
  virtual bool store(FILE* f) const { return true; }

  /** loads the object from the given file stream (binary).
  */
  virtual bool restore(FILE* f) { return true; }

  /****************************************************************************/
  /*        END methods of Storeable                                                      */
  /****************************************************************************/


  /****************************************************************************/
  /*        BEGIN methods of Configurable                                                      */
  /****************************************************************************/

//  Configurable::paramval getParam(const paramkey& key) const;

//  bool setParam(const paramkey& key, paramval val);

//  Configurable::paramlist getParamList() const;

  /****************************************************************************/
  /*        END methods of Configurable                                                      */
  /****************************************************************************/


protected:
  parambool showF;
  parambool showP;
  parambool showXsiF;
  bool initialized; // tells wether the controller is already initialized by the agent
  bool useXsiCalculation;

  double minSensorValue;
  double maxSensorValue;
  int sensorIntervalCount;
  int sensorNumber;
  int motorNumber;
  std::list<matrix::Matrix*> freqMatrixList; // stores the number of occurances of t-1 to t (frequency)
  std::list<matrix::Matrix*> probMatrixList; // stores the probability of state to state occurances of t-1 to t
  std::list<matrix::Matrix*> xsiFreqMatrixList; // stores the number of occurances of xsi(x) (frequency)

  double* oldSensorStates; // stores the sensor states for previous step (t-1)
  int t; // indicates the step (time)

  double* MI; // mutual information = MI(X_{t-1},X_t)
  double* H_x; // H(x) = H(X_{t-1})
  double* H_yx; // H(X_t|X_{t-1})
  double* H_Xsi; // H(Xsi)

  // the next two variables are only used if Xsi_x is calculated
  double ainit;
  double cinit;

  /**
   * Calculates the mutual information
   * This is made by normal formula, which
   * needs O(n^2) costs.
   */
  virtual void calculateMIs(double* MI);

    /**
   * Calculates the entropy of x
   * This is made by normal formula, which
   * needs O(n) costs.
   */
  virtual void calculateH_x(double* H);


  /**
   * Calculates the conditional entropy of y|x
   * This is made by normal formula, which
   * needs O(n²) costs.
   */
  virtual void calculateH_yx(double* H_yx);


  /**
  * Updates the xsi frequency matrix list
  */
  virtual void updateXsiFreqMatrixList(const sensor* sensors);

  /**
   * Calculates the entropy of H(Xsi)
   * This is made by normal formula, which
   * needs O(n) costs.
   */
  virtual void calculateH_Xsi(double* H_Xsi);



  /**
   * Updates the mutual information
   * This is made by a difference term, which
   * is added to the old MI(t-1).
   * This function needs the OLD MI(t-1) AND
   * the OLD F matrix, so update the MI with this
   * function first before updating the F matrix!
   * calculation costs: O(1)
   */
  virtual void updateMIs(const sensor* sensors);



  /**
   * Returns the appropiate state which belongs to the given sensorValue.
   */
  virtual int getState(double sensorValue);


  /**
   * Does the pre-initialization functionality.
   * @param sensornumber
   * @param motornumber
   * @param randGen
   */
  virtual void internalInit(int sensornumber, int motornumber, RandGen* randGen = 0);


};

#endif
