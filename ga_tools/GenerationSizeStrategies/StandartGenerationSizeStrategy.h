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

#ifndef STANDARTGENERATIONSIZESTRATEGY_H_
#define STANDARTGENERATIONSIZESTRATEGY_H_

//forward declaration
class Generation;

//ga_tools includes
#include "IGenerationSizeStrategy.h"

/**
 * this class calculate the new generation size over the enhancement speed
 */
class StandartGenerationSizeStrategy : public IGenerationSizeStrategy{
public:
	/**
	 * constructor
	 * @param startSize (int) is the size of the first generation. is needed as basic of the calculation
	 * @param numGeneration (int) is the number of generation which the alg. will be create
	 */
	StandartGenerationSizeStrategy(int startSize, int numGeneration);

	/**
	 * default destructor
	 */
	virtual ~StandartGenerationSizeStrategy();

	/**
	 * this function calculate the new generation size
	 * @param oldGeneration (Generation*) the old generation
	 * @return (int) the new generation size
	 */
	virtual int calcGenerationSize(Generation* oldGeneration);

protected:
	/**
	 * is the first generation over?
	 */
	bool m_firstIsSet;

	/**
	 * the start size of the first generation
	 */
	int m_startSize;

	/**
	 * the number of generation
	 */
	int m_numGeneration;

	/**
	 * the best value of the first generation
	 */
	double m_best_first;

	/**
	 * the best value of the last generation.
	 */
	double m_best_old;

	/**
	 * the best value of the new generation.
	 */
	double m_best_new;

private:
	/**
	 * disable the default constructor
	 */
	StandartGenerationSizeStrategy();
};

#endif /* STANDARTGENERATIONSIZESTRATEGY_H_ */
