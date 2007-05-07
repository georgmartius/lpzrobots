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
 *   Revision 1.1  2007-05-07 21:01:31  robot3
 *   statistictools is a class for easy visualization of measurements of observed values
 *   it is possible to add the observed value itself with mode ID
 *
 *                                                                         *
 ***************************************************************************/
#ifndef _STATISTIC_MEASURE_H
#define _STATISTIC_MEASURE_H

#include "statistictools.h"

class StatisticMeasure {

public:
	StatisticMeasure(double& observedValue, char* measureName, MeasureMode mode, long stepSpan, double additionalParam);

	virtual ~StatisticMeasure() {}

	virtual void step();

	virtual std::string getName() const { return name; }

	virtual double getValue() const { return value; }

	virtual double& getValueAdress()  { return value; }


protected:
	std::string name;
	double value;
	double& observedValue;
	MeasureMode mode;
	long stepSpan; // determines the size of valueHistory
	double additionalParam;
	long actualStep; // actual step
	long oldestStepIndex; // indicates the index number in the valueHistory, which was the oldest step

	double* valueHistory;

	/**************************************************************************************************/
	/* all the functions for calculating the values below                                             */

	virtual double calculateSumValue();

	virtual double calculateAverageValue();

};

#endif
