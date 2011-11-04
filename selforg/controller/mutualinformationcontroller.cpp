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
#include "mutualinformationcontroller.h"
#include "controller_misc.h"
#include <cmath>

#include <sstream>

using namespace matrix;
using namespace std;

MutualInformationController::MutualInformationController(int sensorIntervalCount, double minSensorValue,
    double maxSensorValue, bool showF, bool showP, bool showXsiF) :
  AbstractController("MutualInformation",
      "$Id$") {
  this->sensorIntervalCount = sensorIntervalCount;
  this->minSensorValue = minSensorValue;
  this->maxSensorValue = maxSensorValue;
  this->showF = showF;
  this->showP = showP;
  this->showXsiF = showXsiF;
  t = 0;
  //  if (sensorIntervalCount <= 10) {
  //    this->showF = 1;
  //    this->showP = 1;
  //  } else {
  //    this->showF = 0;
  //    this->showP = 0;
  //  }
  this->initialized = false;
  this->useXsiCalculation = false;
}

void MutualInformationController::init(int sensornumber, int motornumber, RandGen* randGen) {
  if (!initialized)
    internalInit(sensornumber, motornumber, randGen);

  // assert that sensornumber is not higher than before (this->sensorNumber), the same belongs to motornumber
  assert(sensornumber<=this->sensorNumber);
  assert(motornumber<=this->motorNumber);
  // set sensornumber and motornumber to new values
  this->sensorNumber = sensornumber;
  this->motorNumber = motornumber;
  // now register inspectable values
  for (int i = 0; i < sensorNumber; i++) {
    addInspectableValue("MI" + itos(i), &MI[i], "Mutual Information of sensor space (x[" + itos(i) + "])");
  }
  list<matrix::Matrix*>::const_iterator F = freqMatrixList.begin();
  list<matrix::Matrix*>::const_iterator P = probMatrixList.begin();
  for (int i = 0; i < sensorNumber; i++) {
    if (showF)
      addInspectableMatrix("F" + itos(i), *F, false, "frequency matrix");
    if (showP)
      addInspectableMatrix("P" + itos(i), *P, false, "probability matrix");
  }
  if (useXsiCalculation && showXsiF) {
    list<matrix::Matrix*>::const_iterator F_xsi = xsiFreqMatrixList.begin();
    for (int i = 0; i < sensorNumber; i++)
      addInspectableMatrix("F_xsi" + itos(i), *F_xsi, false, "frequency matrix of H_xsi");
  }
  return;
}

void MutualInformationController::internalInit(int sensornumber, int motornumber, RandGen* randGen) {
  this->sensorNumber = sensornumber;
  this->motorNumber = motornumber;
  // if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  // initialize lists with matrices
  for (int i = 0; i < sensorNumber; i++) {
    // matrix dimension is 1 more because we want to store
    // the sum of the rows and columns, so
    // freqMatrixList[sensorIntervalCount,sensorIntervalCount]
    // is equal to t
    matrix::Matrix* F = new matrix::Matrix(sensorIntervalCount + 1, sensorIntervalCount + 1);
    matrix::Matrix* P = new matrix::Matrix(sensorIntervalCount + 1, sensorIntervalCount + 1);
    this->freqMatrixList.push_back(F);
    this->probMatrixList.push_back(P);
  }
  // allocate memory for oldSensors
  this->oldSensorStates = (double*) malloc(sizeof(double) * sensorNumber);
  // allocate memory for MI
  this->MI = (double*) malloc(sizeof(double) * sensorNumber);
  // allocate memory for H_x
  this->H_x = (double*) malloc(sizeof(double) * sensorNumber);
  // allocate memory for H_yx
  this->H_yx = (double*) malloc(sizeof(double) * sensorNumber);

  // init them with sensorIntervalCount/2
  // init I and H with 0
  for (int i = 0; i < sensorNumber; i++) {
    oldSensorStates[i] = sensorIntervalCount / 2.0;
    MI[i] = 0.0;
    H_x[i] = 0.0;
    H_yx[i] = 0.0;
  }
  if (this->useXsiCalculation) {
    // initialize the xsifreq list with matrices
    for (int i = 0; i < sensorNumber; i++) {
      // matrix dimension is 1 more because we want to store
      // the sum of the rows and columns, so
      // xsifreqMatrixList[sensorIntervalCount]
      // is equal to t
      this->xsiFreqMatrixList.push_back(new matrix::Matrix(sensorIntervalCount + 1, 1));
    }
    // allocate memory for Xsi_x
    this->H_Xsi = (double*) malloc(sizeof(double) * sensorNumber);

    // init Xsi with 0
    for (int i = 0; i < sensorNumber; i++)
      H_Xsi[i] = 0.0;
  }
  this->initialized = true; // tells wether the controller is already initialized

}


void MutualInformationController::setAandCandCalcH_xsi(double ainit, double cinit) {
  if (this->initialized) {
    cerr
        << "ERROR: The method setAandCandCalcH_xsi(double ainit, double cinit) must be called before the initialization"
        << endl;
    cerr << "of the Agent!" << endl;
  } else {
    this->useXsiCalculation = true;
    // remember ainit and cinit
    this->cinit = cinit;
    this->ainit = ainit;
  }
}

void MutualInformationController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
  assert(initialized==true);
  assert(sensornumber==sensorNumber);
  assert(motornumber==motorNumber);
  t++;
  this->updateMIs(sensors);
  // update sensor frequency and probability matrices
  int i = 0;

  list<matrix::Matrix*>::iterator pIt = probMatrixList.begin();
  for (list<matrix::Matrix*>::iterator freqMatrix = freqMatrixList.begin(); freqMatrix != freqMatrixList.end(); freqMatrix++) {
    int state = getState(sensors[i]);
    int oldState = getState(oldSensorStates[i]);

    // set frequencies
    (*freqMatrix)->val(oldState, state)++;

    // sum of frequency of oldSensorState[i] (t-1)
    (*freqMatrix)->val(oldState, sensorIntervalCount)++;
    // sum of frequency of state (t)
    (*freqMatrix)->val(sensorIntervalCount, state)++;
    // total sum of all frequency, is equal to t
    (*freqMatrix)->val(sensorIntervalCount, sensorIntervalCount)++;

    // set probabilities
    for (int row = 0; row <= sensorIntervalCount; row++) {
      for (int col = 0; col <= sensorIntervalCount; col++) {
        (*pIt)->val(row, col) = (*freqMatrix)->val(row, col) / (double) t;
      }
    }

    oldSensorStates[i] = sensors[i]; // update old sensor state
    pIt++;
    i++;
  }

  // calculate H(x)
  this->calculateH_x(this->H_x);
  this->calculateH_yx(this->H_yx);
  // calculate Xsi(x)
  if (this->useXsiCalculation) {
    this->updateXsiFreqMatrixList(sensors);
    this->calculateH_Xsi(this->H_Xsi);
  }
  i = 0;
  // store the old sensor state before leaving this method
  for (list<matrix::Matrix*>::iterator freqMatrix = freqMatrixList.begin(); freqMatrix != freqMatrixList.end(); freqMatrix++) {
    oldSensorStates[i] = sensors[i]; // update old sensor state
  }

  //calculateMIs(MI);
  //cout << "MI[0]    = " << MI[0] << endl;
  //cout << "newMI[0] = " << newMI[0] << endl;
}

void MutualInformationController::stepNoLearning(const sensor* sensors, int sensornumber, motor* motors,
    int motornumber) {
  step(sensors, sensornumber, motors, motornumber);
}

int MutualInformationController::getState(double sensorValue) {
  // returns the state belonging to the sensor value
  int state = (int) (((double) (sensorIntervalCount - 1)) * ((sensorValue + 1.0) / 2) + 0.5);
  if (state > (sensorIntervalCount - 1))
    return (sensorIntervalCount - 1);
  if (state < 0)
    return 0;
  return state;
}

void MutualInformationController::updateMIs(const sensor* sensors) {
  /*
   NEW: calculate MIs with O(1), needs to have the old value!
   IMPORTANT: This calculation needs the OLD F matrix, so update the MI
   with this function first before updating the F matrix!
   calculation formula:
   MI(t) = 1/t ((t-1) (MI(t-1)-log(t-1)) +dS) + log t
   where t = number of timer_ticks
   where dS = +F[n,newState] * log(F[n,newState])
   +F[oldState,n] * log(F[oldState,n])
   +(F[oldState,newState]+1) * log(F[oldState,newState]+1)
   -(F[n,newState]+1) * log(F[n,newState]+1)
   -(F[oldState,n]+1) * log(F[oldState,n]+1)
   -F[oldState,newState] * log(F[oldState,newState])
   where n = sensorIntervalCount
   */
  int i = 0;
  for (list<matrix::Matrix*>::iterator F = freqMatrixList.begin(); F != freqMatrixList.end(); F++) {
    int newState = getState(sensors[i]);
    int oldState = this->getState(oldSensorStates[i]);
    // calculate dS
    int n = sensorIntervalCount; // use only for more compact formula
    double dS = 0.0;
    if (((*F)->val(n, newState)) > 0.0)
      dS = ((*F)->val(n, newState)) * log(((*F)->val(n, newState)));
    if (((*F)->val(oldState, n)) > 0.0)
      dS += ((*F)->val(oldState, n)) * log(((*F)->val(oldState, n)));
    if (((*F)->val(oldState, newState)) > 0.0)
      dS -= ((*F)->val(oldState, newState)) * log(((*F)->val(oldState, newState)));
    dS += (((*F)->val(oldState, newState)) + 1) * log(((*F)->val(oldState, newState)) + 1) - (((*F)->val(n, newState))
        + 1) * log(((*F)->val(n, newState)) + 1) - (((*F)->val(oldState, n)) + 1) * log(((*F)->val(oldState, n)) + 1);
    // updateMI with old MI and dS
    double t = (double) (this->t);
    double tminus1 = (double) (t - 1);
    if ((this->t) == 1) {
      // log(t-1) is infinite, use more simple formula
      MI[i] = dS / t + log(t);
    } else {
      MI[i] = ((tminus1) * (MI[i] - log(tminus1)) + dS) / t + log(t);
    }

    /*if (t<=10) {
     calculateMIs(MI);
     }*/

    i++;
  }

}

void MutualInformationController::updateXsiFreqMatrixList(const sensor* sensors) {
  // update sensor frequency and probability matrices
  int i = 0;
  int state;
  for (list<matrix::Matrix*>::iterator freqMatrix = xsiFreqMatrixList.begin(); freqMatrix != xsiFreqMatrixList.end(); freqMatrix++) {
    // calculate actual Xsi
    double xsi = sensors[i] - ainit * tanh(cinit * this->oldSensorStates[i]);
    if (t % 2 == 0)
      xsi = 0.9;
    else
      xsi = -0.9;
    cout << "xsi=" << xsi << endl;
    state = getState(xsi); // this is only useful if xsi is normally between -1 and +1!!!
    // set frequencies
    (*freqMatrix)->val(state, 0)++;
    // sum of frequency
    (*freqMatrix)->val(sensorIntervalCount, 0)++;

    i++;
  }

}

void MutualInformationController::calculateH_Xsi(double* H_Xsi) {
  int i = 0;
  for (list<matrix::Matrix*>::iterator F = xsiFreqMatrixList.begin(); F != xsiFreqMatrixList.end(); F++) {
    H_Xsi[i] = 0.0;
    for (int x = 0; x < sensorIntervalCount; x++) {
      if (((*F)->val(x, 0)) != 0) { // to avoid log(0)
        double val = ((*F)->val(x, 0) / ((double) t)) * log((*F)->val(x, 0) / ((double) t));
        (H_Xsi[i]) -= val;
      }
    }
    i++;
  }
}

void MutualInformationController::calculateH_x(double* H) {
  int i = 0;
  for (list<matrix::Matrix*>::iterator F = freqMatrixList.begin(); F != freqMatrixList.end(); F++) {
    H[i] = 0.0;
    for (int x = 0; x < sensorIntervalCount; x++) {
      if (((*F)->val(x, sensorIntervalCount)) != 0) { // to avoid log(0)
        double val = ((*F)->val(x, sensorIntervalCount) / ((double) t)) * log((*F)->val(x, sensorIntervalCount)
            / ((double) t));
        (H_x[i]) -= val;
      }
    }
    i++;
  }
}

void MutualInformationController::calculateH_yx(double* H_yx) {
  //
  int i = 0;
  for (list<matrix::Matrix*>::iterator F = freqMatrixList.begin(); F != freqMatrixList.end(); F++) {
    H_yx[i] = 0.0;
    for (int x = 0; x < sensorIntervalCount; x++) {
      for (int y = 0; y < sensorIntervalCount; y++) {
        if (((*F)->val(x, y)) != 0) {
          double val = (*F)->val(x, y) * log(((*F)->val(x, y)) / ((*F)->val(x, sensorIntervalCount)));
          (H_yx[i]) -= val / ((double) t);
        }
      }
    }
    i++;
  }
}

void MutualInformationController::calculateMIs(double* MI) {

  /*
   old formula: I = sum(over t-1) p(t-1) sum(over t) p(t|t-1) ln (p(t|t-1)/p(t))
   = sum(over t-1, t) p(t-1,t) ln (p(t-1,t)/p(t))
   p(t-1,t) = #{s_t-1 --> s_t}/t = F[t-1,t]/t
   p(t-1) = P[t-1,sensorIntervalCount];
   p(t) = P[sensorIntervalCount,t];
   old formula update: we use completely F for calculating MI
   */
  int i = 0;
  for (list<matrix::Matrix*>::iterator F = freqMatrixList.begin(); F != freqMatrixList.end(); F++) {
    MI[i] = 0.0;
    for (int x = 0; x < sensorIntervalCount; x++) {
      for (int y = 0; y < sensorIntervalCount; y++) {
        if (((*F)->val(x, y)) != 0) {
          double val = (*F)->val(x, y) * log(((*F)->val(x, y) * ((double) t)) / ((*F)->val(sensorIntervalCount, y)
              * (*F)->val(x, sensorIntervalCount)));
          (MI[i]) += val / ((double) t);
        }
      }
    }
    i++;
  }
}


/*******************************************************************/
/*  INSPECTABLE                                                    */
/*******************************************************************/
/*
 list<Inspectable::iparamkey> MutualInformationController::getInternalParamNames() const {
 // return names
 list<iparamkey> keyList;
 keyList += string("timer_ticks");
 keyList += getArrayNames(sensorNumber, "MI");
 keyList += getArrayNames(sensorNumber, "H(x)");
 if (this->useXsiCalculation)
 keyList += getArrayNames(sensorNumber, "H(Xsi)");
 int i = 0;
 if (showF > 0) {
 for (list<matrix::Matrix*>::const_iterator freqMatrix = freqMatrixList.begin(); freqMatrix
 != freqMatrixList.end(); freqMatrix++) {
 keyList += storeMatrixFieldNames(**freqMatrix, "F");
 i++;
 }
 }
 if (showP > 0) {
 for (list<matrix::Matrix*>::const_iterator probMatrix = probMatrixList.begin(); probMatrix
 != probMatrixList.end(); probMatrix++) {
 keyList += storeMatrixFieldNames(**probMatrix, "P");
 i++;
 }
 }
 for (list<matrix::Matrix*>::const_iterator freqMatrix = xsiFreqMatrixList.begin(); freqMatrix
 != xsiFreqMatrixList.end(); freqMatrix++) {
 keyList += storeMatrixFieldNames(**freqMatrix, "xsi");
 i++;
 }
 return keyList;
 }

 list<Inspectable::iparamval> MutualInformationController::getInternalParams() const {
 list<iparamval> valList;
 valList += (iparamval) t;
 valList += convertArrayToList(MI, sensorNumber);
 valList += convertArrayToList(H_x, sensorNumber);
 if (this->useXsiCalculation)
 valList += convertArrayToList(H_Xsi, sensorNumber);
 int i = 0;
 if (showF > 0) {
 for (list<matrix::Matrix*>::const_iterator freqMatrix = freqMatrixList.begin(); freqMatrix
 != freqMatrixList.end(); freqMatrix++) {
 valList += (*freqMatrix)->convertToList();
 i++;
 }
 }
 if (showP > 0) {
 for (list<matrix::Matrix*>::const_iterator probMatrix = probMatrixList.begin(); probMatrix
 != probMatrixList.end(); probMatrix++) {
 valList += (*probMatrix)->convertToList();
 i++;
 }
 }
 for (list<matrix::Matrix*>::const_iterator freqMatrix = xsiFreqMatrixList.begin(); freqMatrix
 != xsiFreqMatrixList.end(); freqMatrix++) {
 valList += (*freqMatrix)->convertToList();
 i++;
 }
 return valList;
 }*/
