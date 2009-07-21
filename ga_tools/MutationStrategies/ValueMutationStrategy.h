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
 *   This class is a implementation of the interface IMutationStrategy.    *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.3  2009/06/17 11:25:52  robot12
 *   finishing the mutation strategy and add some comments.
 *
 *   Revision 1.2  2009/05/14 15:29:56  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.1  2009/05/04 15:27:57  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/05/04 09:06:00  robot12
 *   some implements... Part7
 *
 *   Revision 1.1  2009/04/30 11:51:25  robot12
 *   some implements... new classes
 *
 *
 *
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
