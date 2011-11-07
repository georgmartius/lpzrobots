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

#ifndef IFITNESSSTRATEGY_H_
#define IFITNESSSTRATEGY_H_

// forward declaration
class Individual;

/**
 * The interface for the fitness strategy of an individual.
 */
class IFitnessStrategy {
public:
	/**
	 * default constructor
	 * do nothing
	 */
	IFitnessStrategy();

	/**
	 * default destructor
	 * do nothing
	 */
	virtual ~IFitnessStrategy();

	/**
	 * declaration of the function getFitness. This function will give the fitness value
	 * of an individual back. For which individual is him called by 'individual'.
	 *
	 * @param individual (const Individual*) calculate the fitness for this individual
	 * @return (double) The fitness value
	 */
	virtual double getFitness(const Individual* individual) = 0;
};

#endif /* IFITNESSSTRATEGY_H_ */
