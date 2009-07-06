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
 *   This fitness strategy is for calculating the invert of a other        *
 *   fitness strategy.                                                     *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-07-06 15:06:35  robot12
 *   bugfix
 *
 *   Revision 1.1  2009/07/02 15:24:53  robot12
 *   update and add new class InvertedFitnessStrategy
 *
 *
 *
 ***************************************************************************/

#ifndef INVERTEDFITNESSSTRATEGY_H_
#define INVERTEDFITNESSSTRATEGY_H_

//forward declaration
class Individual;

//ga_tools includes
#include "IFitnessStrategy.h"

/**
 * This strategy calculate the invert to a other strategy. This can be used for other optimization targets than zero.
 * For example. Is the optimization target of the alg. infinity, than the inverted target is zero -> so the alg. can
 * optimize again zero and you have you infinity target.
 */
class InvertedFitnessStrategy: public IFitnessStrategy {
public:
	/**
	 * constructor
	 * Needs a other fitness strategy, which should be inverted.
	 * @param strategy (IFitnessStrategy*) the other fitness strategy
	 */
	InvertedFitnessStrategy(IFitnessStrategy* strategy);

	/**
	 * default destructor
	 */
	virtual ~InvertedFitnessStrategy();

	/**
	 * returns the inverse fitness value what the other strategy returns for the individual "individual".
	 *
	 * @param individual (const Individual*) calculate the fitness for this individual
	 * @return (double) The fitness value
	 */
	virtual double getFitness(const Individual* individual);

protected:
	/**
	 * The other strategy
	 */
	IFitnessStrategy* m_strategy;

private:
	/**
	 * disable the default constructor
	 */
	InvertedFitnessStrategy();
};

#endif /* INVERTEDFITNESSSTRATEGY_H_ */
