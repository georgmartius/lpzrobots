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

#ifndef TEMPLATECYCLEDGASIMULATIONFITNESSSTRATEGY_H_
#define TEMPLATECYCLEDGASIMULATIONFITNESSSTRATEGY_H_

#include <ga_tools/IFitnessStrategy.h>
#include <vector>

/**
 * the fitness strategy for this demonstration simulation
 */
class TemplateCycledGaSimulationFitnessStrategy: public IFitnessStrategy {
public:
	/**
	 * default constructor
	 */
	TemplateCycledGaSimulationFitnessStrategy();

	/**
	 * default destructor
	 */
	virtual ~TemplateCycledGaSimulationFitnessStrategy();

	/**
	 * this return the saved fitness value (entropy)
	 *
	 * Be careful because this method isn't safe for individuals, which are not saved in the storage!!!
	 *
	 * @param individual (for this we search the entropy)
	 * @return the fitness
	 */
	virtual double getFitness(const Individual* individual);

	/**
	 * a storage for all Entropies!
	 */
	std::vector<double*> m_storage;
};

#endif /* TEMPLATECYCLEDGASIMULATIONFITNESSSTRATEGY_H_ */
