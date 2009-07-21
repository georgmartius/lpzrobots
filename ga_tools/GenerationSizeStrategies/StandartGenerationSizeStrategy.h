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
 *   This class is a implementation for the generation size strategy of the*
 *   genAlgEngine. It calculate the new generation size over the speed of  *
 *   enhancement between the generation. If it to slow so the generation   *
 *   size will be lower and is it to fast so the generation size will be   *
 *   greater.                                                              *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-07-21 08:37:58  robot12
 *   add some comments
 *
 *   Revision 1.2  2009/06/16 12:25:31  robot12
 *   finishing the generation size strategy and implements the comments.
 *
 *   Revision 1.1  2009/05/04 15:27:57  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.4  2009/05/04 09:06:00  robot12
 *   some implements... Part7
 *
 *   Revision 1.3  2009/05/04 07:06:14  robot12
 *   some implements... Part6
 *
 *   Revision 1.2  2009/04/30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.1  2009/04/30 11:51:25  robot12
 *   some implements... new classes
 *
 *
 *
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
