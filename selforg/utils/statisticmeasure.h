/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2007-12-06 10:18:10  der
 *   AbstractMeasure is now a abstract type for Measures,
 *   StatisticTools now supports AbstractMeasures,
 *   StatisticalMeasure, ComplexMeasure  now derived from
 *   AbstractMeasure,
 *   ComplexMeasure provides support for calculation e.g. entropy,
 *   uses Discretisizer,
 *   Discretisizer is a stand-alone class for support of discretisizing values
 *   TrackableMeasure derived from ComplexMeasure and provides support for calculating complex measures for Trackable objects
 *
 *   Revision 1.3  2007/09/28 08:48:21  robot3
 *   corrected some minor bugs, files are still in develop status
 *
 *   Revision 1.2  2007/09/27 10:49:39  robot3
 *   removed some minor bugs,
 *   added CONVergence test
 *   changed little things for support of the new WSM
 *
 *   Revision 1.1  2007/05/07 21:01:31  robot3
 *   statistictools is a class for easy visualization of measurements of observed values
 *   it is possible to add the observed value itself with mode ID
 *
 *                                                                         *
 ***************************************************************************/
#ifndef _STATISTIC_MEASURE_H
#define _STATISTIC_MEASURE_H

#include "abstractmeasure.h"
#include "measuremodes.h"

class StatisticMeasure : public AbstractMeasure
{

public:
  StatisticMeasure(double& observedValue, char* measureName, MeasureMode mode, long stepSpan, double additionalParam);

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

  /* BEGIN convergence SECTION */
  virtual double testConvergence();
  // stores how much steps the convergence is reached,
  // if stepsReached==stepSpan, convergence criteria is 1, otherwise 0
  long stepsReached;
  /* END convergence SECTION */




};

#endif
