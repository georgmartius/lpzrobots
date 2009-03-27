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
 *   Revision 1.1  2009-03-27 06:16:57  guettler
 *   support for gcc 4.3 compatibility (has to be checked), StatisticTools moves from utils to statistictools
 *
 *   Revision 1.3  2008/04/29 08:51:54  guettler
 *   -cosmetic changes of StatisticTools
 *   -StatisticTools now uses new function addInspectableValue of the
 *   interface Inspectable, not overloading getInternalParams and
 *   getInternalParamNames anymore
 *
 *   Revision 1.2  2008/01/14 09:09:23  der
 *   added stepSize. An abstractmeasure can now be calculated every stepSize
 *   steps.
 *
 *   Revision 1.1  2007/12/06 10:18:10  der
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
#ifndef _ABSTRACT_MEASURE_H
#define _ABSTRACT_MEASURE_H

#import <iostream>
#import "imeasure.h"

/**
 * Class used by StatisticTools.
 * Provides an interface for any kind of time series analysis.
 * Every step the StatisticTools calls step.
 * @sa StatisticTools
 * @sa HUDStatisticsManager
 * @sa IMeasure
 */
class AbstractMeasure : public IMeasure
{

public:

  AbstractMeasure(const char* measureName) : name(measureName), value(0.0), actualStep(0), stepSize(1) {}

  virtual ~AbstractMeasure() {}


  virtual std::string getName() const { return name; }

  virtual double getValue() const { return value; }

  virtual double& getValueAddress()  { return value; }

  virtual void setStepSize(int newStepSize) { stepSize=newStepSize; }

  virtual int getStepSize() const { return stepSize; }

  virtual long getActualStep() const { return actualStep; }


protected:
  std::string name;
  double value;  // this is the value which is determined, e.g. in AVG MeasureMode, it's the average!

  long actualStep; // actual step
  int stepSize;
};

#endif
