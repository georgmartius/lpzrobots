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
