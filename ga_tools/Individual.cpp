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
 *   This class represent one individual of the complete gen. alg. It have *
 *   some gens and a fitness.                                              *
 *                                                                         *
 *   $Log$
 *   Revision 1.14  2009-11-05 14:07:41  robot12
 *   bugfix for restore and store
 *
 *   Revision 1.13  2009/10/23 10:47:45  robot12
 *   bugfix in store and restore
 *
 *   Revision 1.12  2009/10/21 14:08:06  robot12
 *   add restore and store functions to the ga package
 *
 *   Revision 1.11  2009/10/01 13:29:42  robot12
 *   now the individual save his own fitness value
 *
 *   Revision 1.10  2009/07/21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.9  2009/07/06 15:06:35  robot12
 *   bugfix
 *
 *   Revision 1.8  2009/06/29 14:52:14  robot12
 *   finishing Individual and add some comments
 *
 *   Revision 1.7  2009/05/14 15:29:56  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.6  2009/05/12 13:29:26  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.5  2009/05/11 14:08:52  robot12
 *   patch some bugfix....
 *
 *   Revision 1.4  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.6  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.5  2009/04/28 13:23:55  robot12
 *   some implements... Part2
 *
 *   Revision 1.4  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "Individual.h"

#include <algorithm>

Individual::Individual() {
  // nothing
}

Individual::Individual(std::string name, int id, Individual* p1, Individual* p2) {
  m_name = name;
  m_ID = id;
  m_mutated = false;
  m_parent1 = p1;
  m_parent2 = p2;
  m_fitnessCalculated = false;
}

Individual::~Individual() {
  // nothing
}

double Individual::getFitness() {
  if(!m_fitnessCalculated) {
    m_fitness = SingletonGenEngine::getInstance()->getFitness(this);
    m_fitnessCalculated = true;
  }

  return m_fitness;
}

double Individual::getFitnessC()const {
  if(!m_fitnessCalculated) {
    return SingletonGenEngine::getInstance()->getFitness(this);
  }

  return m_fitness;
}

void Individual::removeGen(Gen* gen) {
  std::vector<Gen*>::iterator itr = find(m_gene.begin(),m_gene.end(),gen);
  m_gene.erase(itr);
}
void Individual::removeGen(int x) {
  if(x<getSize())m_gene.erase(m_gene.begin()+x);
}

std::string Individual::IndividualToString(void)const {
  std::string result = "";

  for(std::vector<Gen*>::const_iterator iter = m_gene.begin();iter!=m_gene.end();iter++) {
    result += "" + (*iter)->toString() + "\t";
  }

  char buffer[128];
  sprintf(buffer,"% .12lf",getFitnessC());
  result += buffer;

  return result;
}

std::string Individual::RootToString(bool withMutation)const {
  std::string result = "";

  if(withMutation) {
    if(m_mutated) {
      result += "m,\t";
    }
    else {
      result += " ,\t";
    }
  }
  result += "\"" + m_name + "\"";

  if(m_parent1!=0)
    result += ",\t\"" + m_parent1->getName() + "\"";
  if(m_parent2!=0)
    result += ",\t\"" + m_parent2->getName() + "\"";

  return result;
}

bool Individual::store(FILE* f)const {
  RESTORE_GA_INDIVIDUAL head;
  RESTORE_GA_TEMPLATE<int> integer;

  //test
  if(f==NULL) {
    printf("\n\n\t>>> [ERROR] <<<\nNo File to store GA [individual].\n\t>>> [END] <<<\n\n\n");
    return false;
  }

  integer.value=(int)m_name.length();
  for(unsigned d=0;d<sizeof(RESTORE_GA_TEMPLATE<int>);d++){
    fprintf(f,"%c",integer.buffer[d]);
  }
  fprintf(f,"%s",m_name.c_str());

  head.ID = m_ID;
  head.numberGenes = m_gene.size();

  if(m_parent1==NULL)
    head.parent1 = -1;
  else
    head.parent1 = m_parent1->getID();

  if(m_parent2==NULL)
    head.parent2 = -1;
  else
    head.parent2 = m_parent2->getID();

  head.mutated = m_mutated;
  head.fitnessCalculated = m_fitnessCalculated;
  head.fitness = m_fitness;

  for(unsigned int x=0;x<sizeof(RESTORE_GA_INDIVIDUAL);x++) {
    fprintf(f,"%c",head.buffer[x]);
  }

  for(int y=0;y<head.numberGenes;y++) {
    integer.value = m_gene[y]->getID();
    for(unsigned e=0;e<sizeof(RESTORE_GA_TEMPLATE<int>);e++){
      fprintf(f,"%c",integer.buffer[e]);
    }
  }

  return true;
}

bool Individual::restore(int numberIndividuals,std::map<int,std::string>& nameSet,std::map<int,RESTORE_GA_INDIVIDUAL*>& individualSet, std::map<int,std::vector<int> >& linkSet, std::vector<Individual*>& storage) {
  int x,y;
  Individual* individual;
  RESTORE_GA_INDIVIDUAL* head;

  for(x=0;x<numberIndividuals;x++) {
    head = individualSet[x];

    //create individual
    individual = new Individual(nameSet[head->ID],head->ID);

    //restore values
    individual->m_mutated = head->mutated;
    individual->m_fitnessCalculated = head->fitnessCalculated;
    individual->m_fitness = head->fitness;

    //restore genes
    for(y=0;y<head->numberGenes;y++) {
      individual->m_gene.push_back(SingletonGenEngine::getInstance()->getGen(linkSet[x][y]));
    }

    //make sure that the individuals are in the right order
    //SingletonGenEngine::getInstance()->addIndividual(individual);
    if(storage.size()<=(unsigned int)head->ID)
      storage.resize(head->ID+1);
    storage[head->ID]=individual;
  }

  return true;
}

bool Individual::restoreParent(int numberIndividuals,std::map<int,RESTORE_GA_INDIVIDUAL*>& individualSet) {
  Individual *p1,*p2,*ind;
  RESTORE_GA_INDIVIDUAL* head;

  for(int x=0;x<numberIndividuals;x++) {
    head = individualSet[x];
    ind = SingletonGenEngine::getInstance()->getIndividual(head->ID);

    if(head->parent1!=-1 && head->parent2!=-1) {
      p1 = SingletonGenEngine::getInstance()->getIndividual(head->parent1);
      p2 = SingletonGenEngine::getInstance()->getIndividual(head->parent1);

      ind->m_parent1=p1;
      ind->m_parent2=p2;
    }
  }

  return true;
}
