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
 *   Revision 1.2  2009/04/30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.1  2009/04/30 11:51:26  robot12
 *   some implements... new classes
 *
 *
 *
 ***************************************************************************/

#include "RandomSelectStrategy.h"
#include <vector>
#include "Generation.h"

RandomSelectStrategy::RandomSelectStrategy() {
	// nothing
}

RandomSelectStrategy::RandomSelectStrategy(RandGen* random) {
	m_random = random;
}

RandomSelectStrategy::~RandomSelectStrategy() {
	// nothing
}

void RandomSelectStrategy::select(Generation* oldGeneration, Generation* newGeneration) {
	int kill = oldGeneration->getKillRate();
	int r1;
	std::vector<Individual*> list = oldGeneration->getAllIndividual();		// darf nicht const und & sein!!!
	double range;
	double min;

	// calc range and min of all fitness
	// TODO

	// kill some elements
	while(kill>0) {
		r1 = ((int)m_random->rand())%list.size();
		if(m_random->rand()*range+min>list[r1]->getFitness()) {
			list.erase(list.begin()+r1);
			kill--;
		}
	}

	// take the rest in the new generation
	for(int x=0;x<list.size() && x<newGeneration->getSize();x++) {
		newGeneration->addIndividual(list[x]->ind);
	}

	// clean
	while(list.size()>0) {
		delete list[0];
		list.erase(list.begin());
	}
	list.clear();
}
