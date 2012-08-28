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
#ifndef _ABSTRACT_MEASURE_H
#define _ABSTRACT_MEASURE_H

#include <iostream>
#include "imeasure.h"

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

  AbstractMeasure(const char* measureName)
    : name(measureName), value(0.0), actualStep(0), stepSize(1), displayPrecision(6) {}

  virtual ~AbstractMeasure() {}


  virtual std::string getName() const { return name; }

  virtual double getValue() const { return value; }

  virtual double& getValueAddress()  { return value; }

  virtual void setStepSize(int newStepSize) { stepSize=newStepSize; }

  virtual int getStepSize() const { return stepSize; }

  virtual long getActualStep() const { return actualStep; }

  virtual void setDisplayPrecision(int digits){ displayPrecision=digits; }

  virtual int getDisplayPrecision() const { return displayPrecision; }


protected:
  std::string name;
  double value;  // this is the value which is determined, e.g. in AVG MeasureMode, it's the average!

  long actualStep; // actual step
  int stepSize;
  int displayPrecision;
};

#endif
