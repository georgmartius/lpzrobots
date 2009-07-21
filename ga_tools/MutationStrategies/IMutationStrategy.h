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
 *   This is a interface of a strategy for the mutation which is used by   *
 *   the GenPrototype to give the GenFactory a knowledge of the kind how   *
 *   to mutate a gen.                                                      *
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
 *   Revision 1.1  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *
 *
 ***************************************************************************/

#ifndef IMUTATIONSTRATEGY_H_
#define IMUTATIONSTRATEGY_H_

//forward declarations
class Gen;
class GenContext;
class Individual;
class SingletonGenFactory;

/**
 * This interface gives the structur for the mutation of a gen.
 */
class IMutationStrategy {
public:
	/**
	 * default constructor
	 */
	IMutationStrategy();

	/**
	 * default destructor
	 */
	virtual ~IMutationStrategy();

	/**
	 * mutate a gen
	 * @param context (GenContext*) the context in which the new gen comes (needed by the factory
	 * @param individual (Individual*) the individual, which the new gen becomes
	 * @param oldGen (Gen*) the old gen, which mutate
	 * @param oldContext (GenContext*) the Context in which the old gen are.
	 * @param factory (SingletonGenFactory*) the GenFactory which create the new gen.
	 * @return (Gen*) the new mutated gen
	 */
	virtual Gen* mutate(GenContext* context, Individual* individual, Gen* oldGen, GenContext* oldContext, SingletonGenFactory* factory) = 0;

	/**
	 * gives the Probability of a mutation back.
	 * @return
	 */
	virtual int getMutationProbability(void) = 0;
};

#endif /* IMUTATIONSTRATEGY_H_ */
