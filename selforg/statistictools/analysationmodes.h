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
#ifndef _ANALYSATIONMODES_H
#define _ANALYSATIONMODES_H

/**
 * usage of the statistictools with different analysation modes modes (examples):
 *
 */


/** analysation modes of statistical types. If you add a analysation mode, you have
 *  naturally to implement this analysation mode in statisticmeasure.cpp -
 *  see method StatisticMeasure::step() !
 */
enum AnalysationMode {
         /// returns the average value
                AM_AVG,
        /// returns the median value
                AM_MED,
        /// returns the minimum value
                AM_MIN,
        /// returns the maximum value
                AM_MAX,
        /// returns the range of the values
                AM_RANGE,
        /// returns the quartile Q_(1/4) value
                AM_Q1,
        /// returns the quartile Q_(3/4) value
                AM_Q3,
        /// returns the whisker (1.5*IQR) value
                AM_WHISKER,
        /// returns the whisker1 value
                AM_W1,
        /// returns the whisker3 value
                AM_W3,
        /// returns the inter quartile range (IQR) value
                AM_IQR,
        /// returns the count of extreme values
                AM_NUM_EXT,
        /// returns the special extreme value
                AM_EXT,
        /// returns the best value
                AM_BEST
};

#endif //_ANALYSATIONMODES_H
