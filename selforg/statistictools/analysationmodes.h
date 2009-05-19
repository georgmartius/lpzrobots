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
 *   Revision 1.1  2009-05-19 11:39:45  robot12
 *   add to statistictools some template analysation methodes (needed for boxplots)
 *
 *
 *                                                                         *
 ***************************************************************************/
#ifndef _ANALYSATIONMODES_H
#define _ANALYSATIONMODES_H

/**
 * usage of the statistictools with different analysation modes modes (examples):
 *
 */


/** analysation modes of statistical types. If you add a analysation mode, you have
 *  naturally to implement this analysationmode in statisticmeasure.cpp -
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
	/// returns the inter quartile range (IQR) value
		AM_IQR,
	/// returns the count of extreme values
		AM_NUM_EXT,
	/// returns the special extreme value
		AM_EXT
};

#endif //_ANALYSATIONMODES_H
