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
#ifndef DISCRETISIZER_H
#define DISCRETISIZER_H

/**
Use this class to get discrete values.
 
If no intervalCount is set, the count=1.
If no intervalRange is set, the range is automatically adjusted.
 
There are three possibilities:
1. automaticRange=true
 --> mapping is always disabled
2. automaticRange=false, mapToInterval=true
 --> real (found) range is mapped to specified range
3. automaticRange=false, mapToInterval=false
 --> no mapping, no range adjustment, values outside the specified
     range are set to minRange respectively maxRange 
 
*/
class Discretisizer {
public:


    /**
    * call this constructor if you don't like to decide which range for
    * the values are used, therefore the range ist found automatically.
    *
    *
    * Note: The adjustment of the range is enabled, if this method is called.
    * 
    * @param numberBins the number of bins "created"
    */
    Discretisizer(int numberBins);

    /**
    * call this constructor if you like to decide yourself which range for
    * the values are used.
    *
    * The third parameter decides if the originally range should be completely
    * mapped to the given interval range. If not, the values outside the given
    * interval range are set to minRange respectively maxRange.
    *
    * Note: The adjustment of the range is disabled, if this method is called.
    * 
    * @param numberBins the number of bins "created"
    * @param minRange the minimum of the interval
    * @param maxRange the maximum of the interval
    * @param mapToInterval decides if all values are mapped to the given
    */
  Discretisizer(int numberBins, double minRange, double maxRange, bool mapToInterval);

    virtual ~Discretisizer();

    /**
      returns the given value as an discretisized integer.
      this method returns a value between 0...numberbins-1.
      @param value the value to discretisize
      @return the bin number
    */
    virtual int getBinNumber(double value);

    /**
      returns the given value as an discretisized double.
      this method returns returns a value between minRange and maxRange
      @param value the value to discretisize
      @return discretisized value between minRange and maxRange
    */
    virtual double get
        (double value);

    virtual double getMinRange();

    virtual double getMaxRange();



protected:
    int numberBins;
    bool automaticRange;
    double minRange;
    double maxRange;
    double minValue;
    double maxValue;
    bool mapToInterval;
    bool firstStep;


    /**
    is used for automaticRange, sets min and max range.
    */
    virtual void findMinAndMaxRange(double value);

    /**
    is used for mapToInterval, sets min and max values.
    */
    virtual void findMinAndMaxValues(double value);

    /**
    is used for discretisizing values
    */
    virtual int discretisizeValue(double valueToDiscretisize);

    virtual int roundValue(double valueToRound);
};

#endif
