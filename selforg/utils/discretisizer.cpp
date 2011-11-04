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
#include "discretisizer.h"
#include <controller_misc.h>


Discretisizer::Discretisizer(int numberBins) : numberBins(numberBins) {
    this->automaticRange=true;
    this->mapToInterval=false;
    this->firstStep=true;
}


Discretisizer::Discretisizer(int numberBins, double minRange, double maxRange, bool mapToInterval) : numberBins(numberBins), minRange(minRange), maxRange(maxRange), mapToInterval(mapToInterval) {
    this->automaticRange=false;
    this->firstStep=true;
}

Discretisizer::~Discretisizer() {}

int Discretisizer::getBinNumber(double value) {
  this->findMinAndMaxRange(value); // first find lowest and highest sensor range
  this->findMinAndMaxValues(value);// second find lowest and highest sensor values
  this->firstStep=false;
  return this->discretisizeValue(value);
}

double Discretisizer::get(double value) {
  double binValue = (double) this->getBinNumber(value);
  // we know now the interval
  return -binValue/((double)numberBins)*(maxRange-minRange)+minRange;
}


double Discretisizer::getMinRange() {
    return this->minRange;
}

double Discretisizer::getMaxRange() {
    return this->maxRange;
}


void Discretisizer::findMinAndMaxRange(double value) {
    if (automaticRange) {
      if (firstStep) {
        minRange=value;
        maxRange=value;
      }
      if (value<minRange)
            minRange=value;
        if (value>maxRange)
            maxRange=value;
    }
}


void Discretisizer::findMinAndMaxValues(double value) {
    if (mapToInterval) {
      if (firstStep) {
        minValue=value;
        maxValue=value;
      }
      if (value<minValue)
            minValue=value;
        if (value>maxValue)
            maxValue=value;
    }
}

int Discretisizer::discretisizeValue(double valueToDiscretisize) {
    // if automatic range is enabled, then the new range is already set.
    // This means, nothing more has to be considered.
    if (!automaticRange) {
        if (mapToInterval) {
            // then the values will be mapped:
            // minValue eqv.to minRange,
            // maxValue eqv.to maxRange
            valueToDiscretisize=(valueToDiscretisize-minValue)/(maxValue-minValue)
                                *(maxRange-minRange)+minRange;
        } else { // no mapping, ensure that value is in range
            if (valueToDiscretisize<minRange)
                valueToDiscretisize=minRange;
            if (valueToDiscretisize>maxRange)
                valueToDiscretisize=maxRange;
        }
    }
    // value is now in range
   return roundValue((maxRange-valueToDiscretisize)*numberBins/(maxRange-minRange));
}

int Discretisizer::roundValue(double valueToRound) {
    return (int)(valueToRound<0?valueToRound-.5:valueToRound+.5);
}
