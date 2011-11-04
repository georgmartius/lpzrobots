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
#ifndef _STATISTIC_MEASURE_H
#define _STATISTIC_MEASURE_H

#include "abstractmeasure.h"
#include "measuremodes.h"

/**
 * Class used by StatisticTools.
 * Provides
 */
class StatisticMeasure : public AbstractMeasure
{

public:
  StatisticMeasure(double& observedValue, const char* measureName, MeasureMode mode, long stepSpan, double additionalParam);

  virtual ~StatisticMeasure() {}

  virtual void step();

protected:
  double& observedValue; // the observed value from which the statistic is made
  MeasureMode mode; // the MeasureMode, e.g. ID, AVG, MED, PEAK, CONV,...
  long stepSpan; // determines the size of valueHistory
  double additionalParam;
  long oldestStepIndex; // indicates the index number in the valueHistory, which was the oldest step
  long newestStepIndex; // indicates the index number with the newest value, this is the oldestStepIndex one step before

  double* valueHistory;

  void internInit();


  /**************************************************************************************************/
  /* all the functions for calculating the values are below this line, add new variables if needed  */

  virtual double calculateSumValue();

  virtual double calculateAverageValue();

  virtual double calculateMovingAverageValue();

  virtual double calculateStepDifference();

  virtual double calculateNormalizedStepDifference();

  /* BEGIN convergence SECTION */
  virtual double testConvergence();
  // stores how much steps the convergence is reached,
  // if stepsReached==stepSpan, convergence criteria is 1, otherwise 0
  long stepsReached;
  /* END convergence SECTION */




};

#endif
