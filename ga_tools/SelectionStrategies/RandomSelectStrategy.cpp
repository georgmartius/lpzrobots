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
 *   This class is a implementation for the ISelectStrategy. It make a     *
 *   select by randomized comparison of one individual with a random       *
 *   number. If it lost it dosn't comes in the next generation.            *
 *                                                                         *
 *   $Log$
 *   Revision 1.6  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.5  2009/06/25 13:34:17  robot12
 *   finish the select strategy and add some comments.
 *
 *   Revision 1.4  2009/05/07 14:47:46  robot12
 *   some comments
 *
 *   Revision 1.3  2009/05/06 14:35:14  robot12
 *   implements findBest in StandartGenerationSizeStrategy
 *
 *   Revision 1.2  2009/05/06 13:28:23  robot12
 *   some implements... Finish
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

#include "RandomSelectStrategy.h"

//includes
#include <selforg/statistictools.h>
#include <vector>

//ga_tools includes
#include "Generation.h"
#include "Individual.h"
#include "SingletonGenAlgAPI.h"

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
	int kill = oldGeneration->getKillRate();										//the kill rate
	int r1;																			//variable for a random number
	std::vector<Individual*> list;		// darf nicht const und & sein!!!			//the list of the individual which are "living"
	double range;																	//range of the fitness values
	double min;																		//the minimum of the fitness values
	int num = oldGeneration->getCurrentSize();										//number of individual
	std::vector<Individual*>::iterator iter;										//iterator for the list
	int test=0;																		//make sure that the function terminate...

	for(int y=0;y<num;y++) {												//insert the individual of the old generation in the living list
		list.push_back(oldGeneration->getIndividual(y));
	}

	// calc range and min of all fitness
	DOUBLE_ANALYSATION_CONTEXT* context = new DOUBLE_ANALYSATION_CONTEXT(*oldGeneration->getAllFitness());
	range = context->getRange();
	min = context->getMin();

	// kill some elements
	while(kill>0) {
		r1 = ((int)(m_random->rand()*1000000.0))%list.size();
		if(m_random->rand()*range+min<list[r1]->getFitness()) {				//if the random value over the range
			iter=list.begin();												//of values better than the fitness
			test=0;															//value of the random selected
			std::advance(iter,r1);											//individual, so it will be die.
			list.erase(iter);
			kill--;
		}
		else
			test++;

		if(test==10000)														//if they are no kills after 10000 test, so stop and make an elite select.
			break;
	}

	if(kill>0) {															//elite select, if not enough individual are killed
		ISelectStrategy* elite = SingletonGenAlgAPI::getInstance()->createEliteSelectStrategy();
		Generation* newold = new Generation(oldGeneration->getGenerationNumber(),oldGeneration->getSize()-oldGeneration->getKillRate()+kill,kill);
		// take the rest in the "new generation" with the name newold
		for(int x=0;x<(int)list.size() && x<oldGeneration->getSize()-oldGeneration->getKillRate()+kill;x++) {
			newold->addIndividual(list[x]);
		}
		elite->select(newold,newGeneration);

		// clean
		while(list.size()>0) {
			list.erase(list.begin());
		}
		list.clear();

		delete elite;

		return;
	}

	// take the rest in the new generation
	for(int x=0;x<(int)list.size() && x<newGeneration->getSize();x++) {
		newGeneration->addIndividual(list[x]);
	}

	// clean
	while(list.size()>0) {
		list.erase(list.begin());
	}
	list.clear();
}
