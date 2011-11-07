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

#ifndef TOURNAMENTSELECTSTRATEGY_H_
#define TOURNAMENTSELECTSTRATEGY_H_

//includes
#include <selforg/randomgenerator.h>

//forward declaration
class Generation;

//ga_tools includes
#include "ISelectStrategy.h"

/**
 * This class makes a select by randomized comparison of two individual of the old generation.
 * The worse individual dosn't comes in the next generation. If enough individual "killed",
 * comes the living in the next generation.
 *
 * With this method it is possible that a bad individual comes in the next generation. So you dosn't
 * becomes a to elite generation and save some alternatives in the gens.
 * This way is in this point better than the EliteSelectStrategy.
 */
class TournamentSelectStrategy : public ISelectStrategy {
public:
	/**
	 * constructor
	 * @param random (RandGen*) a random generator for the randomized select of two individual.
	 */
	TournamentSelectStrategy(RandGen* random);

	/**
	 * default destructor
	 */
	virtual ~TournamentSelectStrategy();

	/**
	 * implementation for the interface ISelectStrategy
	 * @param oldGeneration (Generation*) the old generation
	 * @param newGeneration (Generation*) the next generation
	 */
	virtual void select(Generation* oldGeneration, Generation* newGeneration);

protected:
	/**
	 * the random generator
	 */
	RandGen* m_random;

private:
	/**
	 * disable the default constructor
	 */
	TournamentSelectStrategy();
};

#endif /* TOURNAMENTSELECTSTRATEGY_H_ */
