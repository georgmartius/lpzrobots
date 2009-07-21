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
 *   This class is a implementation for the ISelectStrategy. It make an    *
 *   elite select. This mean only the best individual comes in the next    *
 *   generation.                                                           *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.2  2009/06/25 13:34:17  robot12
 *   finish the select strategy and add some comments.
 *
 *   Revision 1.1  2009/05/04 15:27:56  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/04/30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.1  2009/04/30 11:51:26  robot12
 *   some implements... new classes
 *
 *
 *
 ***************************************************************************/

#ifndef ELITESELECTSTRATEGY_H_
#define ELITESELECTSTRATEGY_H_

//forward declaration
class Generation;

//ga_tools includes
#include "ISelectStrategy.h"

/**
 * This class makes a elite select and bring only the best individual in the next generation.
 */
class EliteSelectStrategy :public ISelectStrategy {
public:
	/**
	 * default constructor
	 */
	EliteSelectStrategy();

	/**
	 * default destructor
	 */
	virtual ~EliteSelectStrategy();

	/**
	 * select the individual in the old generation which should be in the new generation (only the best).
	 * @param oldGeneration (Generation*) the old generation
	 * @param newGeneration (Generation*) the new generation
	 */
	virtual void select(Generation* oldGeneration, Generation* newGeneration);
};

#endif /* ELITESELECTSTRATEGY_H_ */
