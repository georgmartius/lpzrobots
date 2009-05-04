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
 *   Revision 1.1  2009-05-04 15:27:56  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/05/04 09:06:00  robot12
 *   some implements... Part7
 *
 *   Revision 1.1  2009/04/30 11:51:26  robot12
 *   some implements... new classes
 *
 *
 *
 ***************************************************************************/

#include "TournamentSelectStrategy.h"
#include <map>
#include "Generation.h"
#include "Individual.h"

TournamentSelectStrategy::TournamentSelectStrategy() {
	// nothing
}

TournamentSelectStrategy::TournamentSelectStrategy(RandGen* random) {
	m_random = random;
}

TournamentSelectStrategy::~TournamentSelectStrategy() {
	// nothing
}

void TournamentSelectStrategy::select(Generation* oldGeneration, Generation* newGeneration) {
	int live = oldGeneration->getCurrentSize() - oldGeneration->getKillRate();
	int size = newGeneration->getSize() - newGeneration->getCurrentSize();
	int num = oldGeneration->getCurrentSize();
	Individual* ind1;
	Individual* ind2;
	int r1,r2;
	std::map<int,Individual*> storage;
	std::map<int,Individual*>::iterator iter;

	for(int x=0; x<live && x<size; x++) {
		r1 = ((int)m_random->rand()) % num;
		r2 = ((int)m_random->rand()) % num;

		ind1 = oldGeneration->getIndividual(r1);
		ind2 = oldGeneration->getIndividual(r2);

		if(ind1->getFitness()>ind2->getFitness()) {
			if(storage[r1]==0)
				storage[r1]=ind1;
			else
				x--;
		}
		else {
			if(storage[r2]==0)
				storage[r2]=ind2;
			else
				x--;
		}
	}

	for(iter=storage.begin();iter!=storage.end();iter++) {
		newGeneration->addIndividual(iter->second);
	}
}
