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
 *   This class is used for grouping some individuals which representing   *
 *   one step in the gen. alg. (called generation). For this it save all   *
 *   individual which are part of this generation. Also it have an Number  *
 *   like a ID, which make this generation individual.                     *
 *                                                                         *
 *   All Generations inside the gen.alg. are only saved in the GenEngine.  *
 *                                                                         *
 *   $Log$
 *   Revision 1.14  2009-10-21 14:08:07  robot12
 *   add restore and store functions to the ga package
 *
 *   Revision 1.13  2009/10/01 13:36:04  robot12
 *   add a methode to become a vector for all individuals which must be calculated (fitness value)
 *
 *   Revision 1.12  2009/08/11 12:57:38  robot12
 *   change the genetic algorithm (first crossover, second select)
 *
 *   Revision 1.11  2009/08/05 13:16:32  robot12
 *   add one clean up
 *
 *   Revision 1.10  2009/07/28 09:13:11  robot12
 *   add a clean up to update (delete context)
 *
 *   Revision 1.9  2009/07/21 08:39:01  robot12
 *   rename "crosover" to crossover
 *
 *   Revision 1.8  2009/06/29 15:30:11  robot12
 *   finishing Generation and add some comments
 *
 *   Revision 1.7  2009/05/14 15:29:54  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.6  2009/05/12 13:29:25  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.5  2009/05/11 14:08:51  robot12
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
#include <selforg/statistictools.h>

Generation::Generation() {
	// nothing
}

Generation::Generation(int generationNumber, int size, int numChildren) {
	m_generationNumber = generationNumber;
	m_size=size;
	m_numChildren = numChildren;


	//adds some variable to the inspectable context
	addInspectableValue("MIN",&m_min);
	addInspectableValue("W1",&m_w1);
	addInspectableValue("Q1",&m_q1);
	addInspectableValue("MED",&m_med);
	addInspectableValue("AVG",&m_avg);
	addInspectableValue("Q3",&m_q3);
	addInspectableValue("W3",&m_w3);
	addInspectableValue("MAX",&m_max);
	addInspectableValue("BEST",&m_best);
	addInspectableValue("SIZE",&m_dSize);
	addInspectableValue("CHILDREN",&m_dNumChildren);
}

Generation::~Generation() {
	m_individual.clear();
}

void Generation::crossover(RandGen* random) {
	int r1,r2;					// 2 random number, which the alg. need
	Individual* ind;			// help variable to save an individual
	int count = 0;
	int active = getCurrentSize();

	while(getCurrentSize()<m_size*2+m_numChildren) {		//create new individual, how long the planed size isn t reached
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

std::string Generation::getAllIndividualAsString(void)const {
	std::string result = "";

	for(std::vector<Individual*>::const_iterator iter=m_individual.begin();iter!=m_individual.end();iter++) {
		result += (*iter)->IndividualToString() + "\n";
	}

	return result;
}

std::vector<double>* Generation::getAllFitness(void)const {
	std::vector<double>* result = new std::vector<double>();

	for(std::vector<Individual*>::const_iterator iter = m_individual.begin(); iter != m_individual.end(); iter++) {
		result->push_back((*iter)->getFitness());
	}

	return result;
}

void Generation::update(double factor) {
	std::vector<double>* ptrFitnessVector = getAllFitness();
	DOUBLE_ANALYSATION_CONTEXT* context = new DOUBLE_ANALYSATION_CONTEXT(*ptrFitnessVector);

	m_q1 = context->getQuartil1();
	m_q3 = context->getQuartil3();
	m_med = context->getMedian();
	m_avg = context->getAvg();
	m_w1 = context->getWhisker1(factor);
	m_w3 = context->getWhisker3(factor);
	m_min = context->getMin();
	m_max = context->getMax();
	m_best = context->getBest();
	m_dSize = (double)m_size;
	m_dNumChildren = (double)m_numChildren;

	delete context;
	delete ptrFitnessVector;
}

std::vector<Individual*>* Generation::getAllUnCalculatedIndividuals(void)const {
  std::vector<Individual*>* result = new std::vector<Individual*>;

  for(std::vector<Individual*>::const_iterator iter = m_individual.begin(); iter != m_individual.end(); iter++) {
    if(!((*iter)->isFitnessCalculated()))
      result->push_back(*iter);
  }

  return result;
}

bool Generation::store(FILE* f)const {
  RESTORE_GA_GENERATION head;

  //test
  if(f==NULL) {
    printf("\n\n\t>>> [ERROR] <<<\nNo File to store GA [generation].\n\t>>> [END] <<<\n\n\n");
    return false;
  }

  head.number = m_generationNumber;
  head.numberIndividuals = m_individual.size();
  head.size = m_size;
  head.children = m_numChildren;
  head.q1 = m_q1;
  head.q3 = m_q3;
  head.w1 = m_w1;
  head.w3 = m_w3;
  head.min = m_min;
  head.max = m_max;
  head.avg = m_avg;
  head.med = m_med;
  head.best = m_best;

  for(unsigned int x=0;x<sizeof(RESTORE_GA_GENERATION);x++) {
    fprintf(f,"%c",head.buffer[x]);
  }

  for(int y=0;y<head.numberIndividuals;y++) {
    fprintf(f,"%i\n",m_individual[y]->getID());
  }

  return true;
}

bool Generation::restore(int numberGeneration, std::map<int,RESTORE_GA_GENERATION*> generationSet, std::map<int,std::vector<int> > linkSet) {
  int x,y;
  Generation* generation;
  RESTORE_GA_GENERATION* head;

  for(x=0;x<numberGeneration;x++) {
    head = generationSet[x];

    //create the generation
    generation = new Generation(x,head->size,head->children);

    //restore values
    generation->m_q1 = head->q1;
    generation->m_q3 = head->q3;
    generation->m_w1 = head->w1;
    generation->m_w3 = head->w3;
    generation->m_min = head->min;
    generation->m_max = head->max;
    generation->m_med = head->med;
    generation->m_avg = head->avg;
    generation->m_best = head->best;
    generation->m_dSize = (double)head->size;
    generation->m_dNumChildren = (double)head->children;

    //restore Individuals
    for(y=0;y<head->size;y++) {
      generation->m_individual.push_back(SingletonGenEngine::getInstance()->getIndividual(linkSet[x][y]));
    }

    SingletonGenEngine::getInstance()->addGeneration(generation);
  }

  return true;
}
