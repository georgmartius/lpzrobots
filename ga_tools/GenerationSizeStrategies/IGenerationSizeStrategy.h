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
 *   This class is used by the GenAlgEngine to specify how big the next    *
 *   Generation should be. This is only a interface for this strategy      *
 *   which can the Alg. use.                                               *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-07-21 08:37:58  robot12
 *   add some comments
 *
 *   Revision 1.2  2009/06/16 12:25:32  robot12
 *   finishing the generation size strategy and implements the comments.
 *
 *   Revision 1.1  2009/05/04 15:27:57  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.3  2009/05/04 07:06:14  robot12
 *   some implements... Part6
 *
 *   Revision 1.2  2009/04/28 13:23:55  robot12
 *   some implements... Part2
 *
 *   Revision 1.1  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef IGENERATIONSIZESTRATEGY_H_
#define IGENERATIONSIZESTRATEGY_H_

//forward declaration
class Generation;

/**
 * This interface is to specify how big the next generation should be.
 */
class IGenerationSizeStrategy {
public:
	/**
	 * default constructor
	 * do nothing
	 */
	IGenerationSizeStrategy();

	/**
	 * default destructor
	 * do nothing
	 */
	virtual ~IGenerationSizeStrategy();

	/**
	 * declaration for a function which calculate the size of the next generation after oldGeneration
	 * @param oldGeneration (Generation*) the old generation
	 * @return (int) the new size
	 */
	virtual int calcGenerationSize(Generation* oldGeneration) = 0;
};

#endif /* IGENERATIONSIZESTRATEGY_H_ */
