/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Joerg Weider   <joergweide84 at aol dot com> (robot12)               *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *    Joern Hoffmann <jhoffmann at informatik dot uni-leipzig dot de       *
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
