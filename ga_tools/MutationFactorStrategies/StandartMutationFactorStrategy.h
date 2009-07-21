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
 *   This class is a implementation of the IMutationFactorStrategy         *
 *   interface. It returns a value which is calculate by the variance of   *
 *   the gens in the given set.                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.2  2009/06/17 11:11:06  robot12
 *   finishing the mutationfactorstrategy and add some comments.
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/05/04 09:20:52  robot12
 *   some implements.. Finish --> first compile
 *
 *   Revision 1.1  2009/04/29 14:32:28  robot12
 *   some implements... Part4
 *
 *
 *
 ***************************************************************************/

#ifndef STANDARTMUTATIONFACTORSTRATEGY_H_
#define STANDARTMUTATIONFACTORSTRATEGY_H_

//includes
#include <vector>

//forward declarations
class Gen;
class IValue;

//ga_tools includes
#include "IMutationFactorStrategy.h"

/**
 * this strategy calculate the mutation factor by the variance of the gens in the giving set
 */
class StandartMutationFactorStrategy: public IMutationFactorStrategy {
public:
	/**
	 * default constructor
	 */
	StandartMutationFactorStrategy();

	/**
	 * default destructor
	 */
	virtual ~StandartMutationFactorStrategy();

	/**
	 * gives the varianz of the gens in the set as mutation factor back.
	 * @param gene (vector<Gen*>) the set of gens
	 * @return (IValue*) the mutation factor
	 */
	virtual IValue* calcMutationFactor(const std::vector<Gen*>& gene);
};

#endif /* STANDARTMUTATIONFACTORSTRATEGY_H_ */
