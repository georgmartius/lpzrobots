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
 *   This class is a interface for the fitness strategy. It is used from   *
 *   the class individual an is for individual strategy design pattern.    *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-07-02 15:24:53  robot12
 *   update and add new class InvertedFitnessStrategy
 *
 *   Revision 1.2  2009/06/15 13:58:37  robot12
 *   3 new fitness strategys and IFitnessStrategy and SumFitnessStragegy with comments.
 *
 *   Revision 1.1  2009/05/04 15:27:56  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.3  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.2  2009/04/28 13:23:55  robot12
 *   some implements... Part2
 *
 *   Revision 1.1  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
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
