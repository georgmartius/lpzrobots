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
 *   This class is used for grouping some individuals which representing   *
 *   one step in the gen. alg. (called generation). For this it save all   *
 *   individual which are part of this generation. Also it have an Number  *
 *   like a ID, which make this generation individual.                     *
 *                                                                         *
 *   All Generations inside the gen.alg. are only saved in the GenEngine.  *
 *                                                                         *
 *   $Log$
 *   Revision 1.5  2009-05-11 14:08:51  robot12
 *   patch some bugfix....
 *
 *   Revision 1.4  2009/05/07 14:47:47  robot12
 *   some comments
 *
 *   Revision 1.3  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
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
 *   Revision 1.1  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "Generation.h"

#include "SingletonIndividualFactory.h"
#include "Individual.h"

Generation::Generation() {
	// nothing
}

Generation::Generation(int generationNumber, int size, int kill) {
	m_generationNumber = generationNumber;
	m_size=size;
	m_kill = kill;
}

Generation::~Generation() {
	m_individual.clear();
}

void Generation::crosover(RandGen* random) {
	int r1,r2;					// 2 random number, which the alg. need
	Individual* ind;			// help variable to save an individual
	int count = 0;
	int active = getCurrentSize();

	while(getCurrentSize()<m_size) {		//create new individual, how long the planed size isn t reached
		r1 = ((int)(random->rand()*1000000.0))%active;		// the first random number
		r2 = r1;											// to come min one time inside the while loop
		while(r1==r2)
			r2 = ((int)(random->rand()*1000000.0))%active;	// the second random number

		count++;
		ind = SingletonIndividualFactory::getInstance()->createIndividual(m_individual[r1],m_individual[r2],random);	// create new individual with the 2 other individuals which are represented with the 2 random numbers
		addIndividual(ind);                                 // insert the new individual
	}
}

void Generation::addIndividual(Individual* individual) {
	m_individual.push_back(individual);
}
