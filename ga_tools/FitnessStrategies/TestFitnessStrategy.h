/***************************************************************************
 *   Copyright (C) 2005-2009 by Robot Group Leipzig                        *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    joergweide84@aol.com (robot12)                                       *
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
 *   This is a test implementation for IFitnessStrategy. It calculate      *
 *   from the first two double gens the function                           *
 *   10*(x²+2.5*y²-y)exp(1-(x²+y²)) + 2.4 + 0.1x² + 0.1y²                  *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.1  2009/06/15 13:58:37  robot12
 *   3 new fitness strategys and IFitnessStrategy and SumFitnessStragegy with comments.
 *
 *
 ***************************************************************************/

#ifndef TESTFITNESSSTRATEGY_H_
#define TESTFITNESSSTRATEGY_H_

//forward declaration
class Individual;

//ga_tools includes
#include "IFitnessStrategy.h"

/**
 * Test fitness strategy
 */
class TestFitnessStrategy: public IFitnessStrategy {
public:
	/**
	 * default constructor
	 */
	TestFitnessStrategy();

	/**
	 * default destructor
	 */
	virtual ~TestFitnessStrategy();

	/**
	 * calculate the test fitness value for a individual
	 * @param individual (const Individual*) the individual
	 * @return (double) the result
	 */
	virtual double getFitness(const Individual* individual);
};

#endif /* TESTFITNESSSTRATEGY_H_ */
