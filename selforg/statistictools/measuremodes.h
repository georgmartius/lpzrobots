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
 *   Revision 1.3  2008/04/24 11:57:00  der
 *   added new measure types
 *
 *   Revision 1.2  2008/03/12 10:57:07  der
 *   added moving average MOVAVG
 *
 *   Revision 1.1  2007/09/27 10:56:40  robot3
 *   The enum MeasureModes can be found as from now here. Seems to be better for
 *   guys who like to implement new statistic measures - the only relevant files
 *   are measuremodes.h (this one) and statisticmeasure.cpp.
 *
 *
 *                                                                         *
 ***************************************************************************/
#ifndef _MEASUREMODES_H
#define _MEASUREMODES_H

/**
 * usage of the statistictools with different measure modes (examples):
 *
 * adds following measure: the force of the nimm2, ID = the force itself,
 * 3 = has no effect?
 * stats->addMeasure(myNimm2->getForce(), "force", ID, 3);
 *
 * next: the average of the observed value (force), AVG = average,
 * 50 = average over 50 timesteps (stepspan=50)
 * stats->addMeasure(myNimm2->getForce(), "forceAvg50", AVG, 50);
 *
 * next: the medium of the oserved value (force), MED = medium,
 * 3 = medium over 3 timesteps (stepspan=3)
 * stats->addMeasure(myNimm2->getForce(), "forceMed3", MED, 3);
 *
 * next: the peak of the observed value above a given threshold
 * stats->addMeasure(myNimm2->getForce(), "forcePeak1", PEAK, 0, 1.0);
 *
 *
 */


/** measure modes of statistical types. If you add a measure mode, you have
 *  naturally to implement this measuremode in statisticmeasure.cpp -
 *  see method StatisticMeasure::step() !
 */
enum MeasureMode {
 	/// returns the value to observe itself
	ID,
	/// returns the average value
		AVG,
	/// returns the moving average value
                MOVAVG,
	/// returns the median value
		MED,
	/// returns the minimum value
		MIN,
	/// returns the maximum value
		MAX,
	/// returns only values above defined limit
		PEAK,
	/// returns the sum of the value generated over time (or stepSpan)
		SUM,
	/// returns 1 if convergence is reached, otherwise 0
	/// convergence criteria (epsilon) is given by addMeasure, the convergence
	/// is reached if value is falling below the criteria of the measure time (stepSpan)
		CONV,
    /// returns the difference between two successive steps
        STEPDIFF,
    /// returns the difference between two successive steps, normalized with number of steps
    NORMSTEPDIFF
};

#endif
