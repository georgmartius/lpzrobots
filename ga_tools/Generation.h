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
 *   Revision 1.3  2009-05-04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.5  2009/04/30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.4  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.3  2009/04/29 11:36:41  robot12
 *   some implements... Part3
 *
 *   Revision 1.2  2009/04/28 13:23:55  robot12
 *   some implements... Part2
 *
 *   Revision 1.1  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef GENERATION_H_
#define GENERATION_H_

#include <vector>
#include <selforg/randomgenerator.h>

class Individual;

class Generation {
public:
	Generation(int generationNumber, int size, int kill);
	virtual ~Generation();

	inline int getGenerationNumber(void)const {return m_generationNumber;}
	inline int getSize(void)const {return m_size;}
	inline int getCurrentSize(void)const {return m_individual.size();}
	inline int getKillRate(void)const {return m_kill;}

	inline Individual* getIndividual(int x)const {if(x<getCurrentSize())return m_individual[x];return NULL;}
	inline const std::vector<Individual*>& getAllIndividual(void)const {return m_individual;}

	void addIndividual(Individual* individual);

	// void select(Generation* newGeneration);
	void crosover(RandGen* random);

protected:
	int m_generationNumber;
	std::vector<Individual*> m_individual;

	int m_size;
	int m_kill;

private:
	Generation();
};

#endif /* GENERATION_H_ */
