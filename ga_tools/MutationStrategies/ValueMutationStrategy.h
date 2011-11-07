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

#ifndef VALUEMUTATIONSTRATEGY_H_
#define VALUEMUTATIONSTRATEGY_H_

//forward declarations
class Gen;
class Individual;
class GenContext;
class SingletonGenFactory;
class IMutationFactorStrategy;

//ga_tools includes
#include "IMutationStrategy.h"

/**
 * this mutation strategy clculate a mutation factor by using a
 * mutation factor strategy an add this factor to the old gen.
 */
class ValueMutationStrategy : public IMutationStrategy {
public:
	/**
	 * constructor
	 * @param strategy (IMutationFactorStrategy*) this strategie is used to calculate the mutation factor.
	 * @param mutationProbability (int) the mutation probability which is give back.
	 */
	ValueMutationStrategy(IMutationFactorStrategy* strategy, int mutationProbability);

	/**
	 * default destructor
	 */
	virtual ~ValueMutationStrategy();

	/**
	 * mutate a gen
	 * @param context (GenContext*) the context in which the new gen comes (needed by the factory
	 * @param individual (Individual*) the individual, which the new gen becomes
	 * @param oldGen (Gen*) the old gen, which mutate
	 * @param oldContext (GenContext*) the Context in which the old gen are.
	 * @param factory (SingletonGenFactory*) the GenFactory which create the new gen.
	 * @return (Gen*) the new mutated gen
	 */
	virtual Gen* mutate(GenContext* context, Individual* individual, Gen* oldGen, GenContext* oldContext, SingletonGenFactory* factory);

	/**
	 * gives the Probability of a mutation back.
	 * @return
	 */
	virtual int getMutationProbability(void);

protected:
	/**
	 * the mutation factor strategy
	 */
	IMutationFactorStrategy* m_strategy;

	/**
	 * the mutation probability
	 */
	int m_mutationProbability;

private:
	/**
	 * disable the default constructor
	 */
	ValueMutationStrategy();
};

#endif /* VALUEMUTATIONSTRATEGY_H_ */
