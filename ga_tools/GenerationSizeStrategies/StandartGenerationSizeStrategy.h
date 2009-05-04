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
 *   Informative Beschreibung der Klasse                                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2009-05-04 15:27:57  robot12
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

class Generation;

#include "IGenerationSizeStrategy.h"

class StandartGenerationSizeStrategy : public IGenerationSizeStrategy{
public:
	StandartGenerationSizeStrategy(int startSize, int numGeneration);
	virtual ~StandartGenerationSizeStrategy();

	virtual int calcGenerationSize(Generation* oldGeneration);

protected:
	bool m_firstIsSet;
	int m_startSize;
	int m_numGeneration;
	double m_best_first;
	double m_best_old;
	double m_best_new;

private:
	StandartGenerationSizeStrategy();

	double findBest(Generation* oldGeneration);
};

#endif /* STANDARTGENERATIONSIZESTRATEGY_H_ */
