/***************************************************************************
 *   Copyright (C) 2005-2009 by Robot Group Leipzig                        *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
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
 *   This class implements the IGenerationSize strategy interface for the  *
 *   GenAlgEngine with a fix generation size.                              *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-06-16 12:25:32  robot12
 *   finishing the generation size strategy and implements the comments.
 *
 *   Revision 1.1  2009/05/04 15:27:57  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.4  2009/05/04 07:06:14  robot12
 *   some implements... Part6
 *
 *   Revision 1.3  2009/04/30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.2  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *
 *
 ***************************************************************************/

#ifndef FIXGENERATIONSIZESTRATEGY_H_
#define FIXGENERATIONSIZESTRATEGY_H_

//forward declaration
class Generation;

//ga_tools includes
#include "IGenerationSizeStrategy.h"

/**
 * This class implements the generation size strategy with a fix value which is over the constructor given
 */
class FixGenerationSizeStrategy: public IGenerationSizeStrategy {
public:
	/**
	 * constructor
	 * @param value (int) the fix value which is every time give back by calcGenerationSize.
	 */
	FixGenerationSizeStrategy(int value);

	/**
	 * default destructor
	 */
	virtual ~FixGenerationSizeStrategy();

	/**
	 * gives the fix value m_size as new generation size back.
	 * @param oldGeneration (Generation*) the old Generation. dont needed
	 * @return (int) m_size
	 */
	virtual int calcGenerationSize(Generation* oldGeneration);

protected:
	/**
	 * the fix generation size
	 */
	int m_size;

private:
	/**
	 * disable the default constructor
	 */
	FixGenerationSizeStrategy();
};

#endif /* FIXGENERATIONSIZESTRATEGY_H_ */
