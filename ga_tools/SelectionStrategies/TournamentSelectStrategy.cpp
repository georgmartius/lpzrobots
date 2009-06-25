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
 *   This class is a implementation for the ISelectStrategy. It make a     *
 *   select by randomized comparison of two individual. The worse          *
 *   individual dosn't comes in the next generation.                       *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2009-06-25 13:34:17  robot12
 *   finish the select strategy and add some comments.
 *
 *   Revision 1.3  2009/05/14 15:29:56  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.2  2009/05/11 14:08:52  robot12
 *   patch some bugfix....
 *
 *   Revision 1.1  2009/05/04 15:27:56  robot12
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

//includes
#include <vector>

//ga_tools includes
#include "TournamentSelectStrategy.h"
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
	//int live = oldGeneration->getCurrentSize() - oldGeneration->getKillRate();
	//int size = newGeneration->getSize() - newGeneration->getCurrentSize();
	int kill = oldGeneration->getKillRate();
	int num = oldGeneration->getCurrentSize();
	Individual* ind1;									// the 2 individual which are tested
	Individual* ind2;
	int r1,r2;
	//std::map<int,Individual*> storage;
	//std::map<int,Individual*>::iterator iter;
	std::vector<Individual*> storage;					//list with all living individuals
	std::vector<Individual*>::iterator iter;			//iterator for the list
	double f1,f2;

	for(int y=0;y<num;y++) {							//take all individual in the list.
		storage.push_back(oldGeneration->getIndividual(y));
	}

	for(int x=0; x<kill && x<num; x++) {				//select two individual for the test
		r1 = ((int)(m_random->rand()*1000000.0)) % num;		//2 random indices for the individual list
		r2 = r1;
		while(r2==r1) {
			r2 = ((int)(m_random->rand()*1000000.0)) % num;
		}

		//ind1 = oldGeneration->getIndividual(r1);
		//ind2 = oldGeneration->getIndividual(r2);
		ind1 = storage[r1];									//become the 2 individual
		ind2 = storage[r2];

		f1 = ind1->getFitness();							//the fitness values of this individuals
		f2 = ind2->getFitness();

		f1*=f1;		// abs									//in absolute
		f2*=f2;		// abs

		if(f1<f2) {											//the test and than kill the worse
			/*if(storage[r1]==0)
				storage[r1]=ind1;
			else
				x--;*/
			storage.erase(std::find(storage.begin(),storage.end(),storage[r2]));
		}
		else {
			/*if(storage[r2]==0)
				storage[r2]=ind2;
			else
				x--;*/
			storage.erase(std::find(storage.begin(),storage.end(),storage[r1]));
		}

		num--;
	}

	/*char buffer[1024];
	sprintf(buffer,"fff_%i.txt",newGeneration->getGenerationNumber());
	FILE* fff = fopen(buffer,"w");*/
	for(iter=storage.begin();iter!=storage.end();iter++) {			//transfer the living individual in the new generation
		newGeneration->addIndividual(*iter);
		//fprintf(fff,"%lf\n",(*iter)->getFitness());
	}
	//fclose(fff);
}
