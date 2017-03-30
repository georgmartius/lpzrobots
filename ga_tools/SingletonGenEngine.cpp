/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Joerg Weider   <joergweide84 at aol dot com> (robot12)               *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *    Joern Hoffmann <jhoffmann at informatik dot uni-leipzig dot de       *
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
 *                                                                         *
 ***************************************************************************/

#include "SingletonGenEngine.h"

#include <selforg/plotoptionengine.h>
#include <list>
#include <selforg/inspectableproxy.h>

#include "GenPrototype.h"
#include "Generation.h"
#include "Individual.h"
#include "ISelectStrategy.h"
#include "IGenerationSizeStrategy.h"
#include "Gen.h"
#include "GenContext.h"
#include "IFitnessStrategy.h"
#include "SingletonIndividualFactory.h"
#include "SingletonGenFactory.h"

SingletonGenEngine* SingletonGenEngine::m_engine = 0;

SingletonGenEngine::SingletonGenEngine() {
  m_actualGeneration = 0;
  m_cleanStrategies = false;
  m_selectStrategy = 0;
  m_generationSizeStrategy = 0;
  m_fitnessStrategy = 0;
}

SingletonGenEngine::~SingletonGenEngine() {
  std::vector<GenPrototype*>::iterator iterPro;
  std::vector<Generation*>::iterator iterGener;
  std::vector<Individual*>::iterator iterInd;
  std::vector<Gen*>::iterator iterGen;

  //delete all Prototypes
  while(m_prototype.size()>0) {
    iterPro = m_prototype.begin();
    delete (*iterPro);
    m_prototype.erase(iterPro);
  }
  m_prototype.clear();

  //delete all generation
  while(m_generation.size()>0) {
    iterGener = m_generation.begin();
    delete (*iterGener);
    m_generation.erase(iterGener);
  }
  m_generation.clear();

  //delete all individual
  while(m_individual.size()>0) {
    iterInd = m_individual.begin();
    delete (*iterInd);
    m_individual.erase(iterInd);
  }
  m_individual.clear();

  //delete all gens
  while(m_gen.size()>0) {
    iterGen = m_gen.begin();
    delete (*iterGen);
    m_gen.erase(iterGen);
  }
  m_gen.clear();

  SingletonGenFactory::destroyGenFactory();
  SingletonIndividualFactory::destroyFactory();

  // should we clean the strategies?
  if(m_cleanStrategies) {
    if(m_selectStrategy!=0) {
      delete m_selectStrategy;
      m_selectStrategy = 0;
    }

    if(m_generationSizeStrategy!=0) {
      delete m_generationSizeStrategy;
      m_generationSizeStrategy = 0;
    }

    if(m_fitnessStrategy!=0) {
      delete m_fitnessStrategy;
      m_fitnessStrategy = 0;
    }
  }
}

void SingletonGenEngine::generateFirstGeneration(int startSize, int numChildren, RandGen* random, bool withUpdate) {
  // clean the generations
  std::vector<Generation*>::iterator iterGener;
  while(m_generation.size()>0) {
    iterGener = m_generation.begin();
    delete (*iterGener);
    m_generation.erase(iterGener);
  }
  m_generation.clear();

  // clean the individuals
  std::vector<Individual*>::iterator iterInd;
  while(m_individual.size()>0) {
    iterInd = m_individual.begin();
    delete (*iterInd);
    m_individual.erase(iterInd);
  }
  m_individual.clear();

  // clean the gens
  std::vector<Gen*>::iterator iterGen;
  while(m_gen.size()>0) {
    iterGen = m_gen.begin();
    delete (*iterGen);
    m_gen.erase(iterGen);
  }
  m_gen.clear();

  // generate the first generation
  Generation* first = new Generation(-1,startSize,numChildren);
  addGeneration(first);
  m_actualGeneration = 0;

  // generate the first contexts
  GenContext* context;
  GenPrototype* prototype;
  for(unsigned int a=0;a<m_prototype.size();a++) {
    prototype = m_prototype[a];
    context = new GenContext(prototype);
    prototype->insertContext(first,context);
  }

  // generate the random individuals
  Individual* ind;
  for(int x=0;x<startSize;x++) {
    ind = SingletonIndividualFactory::getInstance()->createIndividual();
    first->addIndividual(ind);
  }
  select();
  crossover(random);

  // update generation
  if(withUpdate)
    first->update();
}

void SingletonGenEngine::prepareNextGeneration(int size, int numChildren) {
  // generate the next generation
  Generation* next = new Generation(m_actualGeneration++,size,numChildren);
  addGeneration(next);
  //m_actualGeneration++;

  // generate the next GenContext
  int num = m_prototype.size();
  GenContext* context;
  GenPrototype* prototype;
  for(int x=0;x<num;x++) {
    prototype = m_prototype[x];
    context = new GenContext(prototype);
    prototype->insertContext(next,context);
  }
}

void SingletonGenEngine::prepare(int startSize, int numChildren, InspectableProxy*& proxyGeneration, InspectableProxy*& proxyGene, RandGen* random, PlotOptionEngine* plotEngine, PlotOptionEngine* plotEngineGenContext, bool withUpdate) {
  Generation* generation;
  std::list<Inspectable*> actualContextList;

  // create first generation
  generateFirstGeneration(startSize,numChildren, random, withUpdate);

  // Control values
  generation = getActualGeneration();
  if(plotEngine!=0) {
    actualContextList.clear();
    actualContextList.push_back(generation);
    proxyGeneration = new InspectableProxy(actualContextList);
    plotEngine->addInspectable(&(*proxyGeneration));
    plotEngine->init();
    plotEngine->plot(1.0);
  }
  if(plotEngineGenContext!=0) {
    actualContextList.clear();
    for(std::vector<GenPrototype*>::const_iterator iter = m_prototype.begin(); iter!=m_prototype.end(); iter++) {
      actualContextList.push_back((*iter)->getContext(getActualGeneration()));
      (*iter)->getContext(getActualGeneration())->update();
    }
    proxyGene = new InspectableProxy(actualContextList);
    plotEngineGenContext->addInspectable(&(*proxyGene));
    plotEngineGenContext->init();
    plotEngineGenContext->plot(1.0);
  }
}

int SingletonGenEngine::getNextGenerationSize() {
  return m_generationSizeStrategy->calcGenerationSize(getActualGeneration());
}

void SingletonGenEngine::measureStep(double time, InspectableProxy*& proxyGeneration, InspectableProxy*& proxyGene, PlotOptionEngine* plotEngine, PlotOptionEngine* plotEngineGenContext) {
  std::list<Inspectable*> actualContextList;
  Generation* generation;

  if(plotEngine!=0) {
    actualContextList.clear();
    generation = getActualGeneration();
    actualContextList.push_back(generation);
    proxyGeneration->replaceList(actualContextList);
    plotEngine->plot(time);
  }
  if(plotEngineGenContext!=0) {
    actualContextList.clear();
    for(std::vector<GenPrototype*>::const_iterator iter = m_prototype.begin(); iter!=m_prototype.end(); iter++) {
      actualContextList.push_back((*iter)->getContext(getActualGeneration()));
    }
    proxyGene->replaceList(actualContextList);
    plotEngineGenContext->plot(time);
  }
}

void SingletonGenEngine::runGenAlg(int startSize, int numChildren, int numGeneration, RandGen* random, PlotOptionEngine* plotEngine, PlotOptionEngine* plotEngineGenContext) {
  InspectableProxy* actualContext;
  InspectableProxy* actualGeneration;

  // create first generation
  prepare(startSize,numChildren,actualGeneration,actualContext,random,plotEngine,plotEngineGenContext);

  // generate the other generations
  for(int x=0;x<numGeneration;x++) {
    select();
    crossover(random);
    update();

    printf("Generaion %i:\tabgeschlossen.\n",x);

    measureStep((double)(x+2),actualGeneration,actualContext,plotEngine,plotEngineGenContext);

    // Abbruchkriterium fehlt noch!!!
    // TODO
  }
}

void SingletonGenEngine::select(bool createNextGeneration) {
  //std::cout<<"createNextGeneration:"<<(createNextGeneration?" yes\n":" no\n");
  if(createNextGeneration)
    getInstance()->prepareNextGeneration(m_generationSizeStrategy->calcGenerationSize(getActualGeneration()),getActualGeneration()->getNumChildren());

  //std::cout<<"begin select\n";
  getInstance()->m_selectStrategy->select(getInstance()->m_generation[getInstance()->m_actualGeneration-1],getInstance()->m_generation[getInstance()->m_actualGeneration]);
  //std::cout<<"select OK\n";

  //std::cout<<"copy Gens\n";
  // insert the old gens in the new GenContext.
  const std::vector<Individual*>& old = getInstance()->m_generation[getInstance()->m_actualGeneration]->getAllIndividual();
  std::vector<Individual*>::const_iterator iter;
  int num;
  Gen* gen;
  GenPrototype* prototype;
  GenContext* newContext;

  for(iter=old.begin();iter!=old.end();iter++) {
    num = (*iter)->getSize();
    for(int x=0; x<num; x++) {
      gen = (*iter)->getGen(x);
      prototype = gen->getPrototype();
      newContext = prototype->getContext(getInstance()->m_generation[getInstance()->m_actualGeneration]);
      newContext->addGen(gen);
    }
  }

  //std::cout<<"alles OK\n";
}

void SingletonGenEngine::crossover(RandGen* random) {
  m_generation[m_actualGeneration]->crossover(random);
}

void SingletonGenEngine::update(double factor) {
  m_generation[m_actualGeneration]->update(factor);

  for(std::vector<GenPrototype*>::const_iterator iter = m_prototype.begin(); iter!=m_prototype.end(); iter++) {
    (*iter)->getContext(m_generation[m_actualGeneration])->update(factor);
  }
}

double SingletonGenEngine::getFitness(const Individual* individual) {
  return m_fitnessStrategy->getFitness(individual);
}

Individual* SingletonGenEngine::getBestIndividual(void) {
  const std::vector<Individual*>& storage = getActualGeneration()->getAllIndividual();
  Individual* result = storage[0];
  double value = result->getFitness();
  double test;
  int num = storage.size();

  for(int x=1;x<num;x++) {
    test = storage[x]->getFitness();
    if(test<value) {
      value = test;
      result = storage[x];
    }
  }

  return result;
}

std::string SingletonGenEngine::getIndividualRoot(bool withMutation)const {
  std::string result="";

  for(std::vector<Individual*>::const_iterator iter=m_individual.begin();iter!=m_individual.end();iter++) {
    result += (*iter)->RootToString(withMutation) + "\n";
  }

  return result;
}

std::string SingletonGenEngine::getAllIndividualAsString(void)const {
  std::string result = "";

  for(std::vector<Individual*>::const_iterator iter=m_individual.begin();iter!=m_individual.end();iter++) {
    result += (*iter)->IndividualToString() + "\n";
  }

  return result;
}

bool SingletonGenEngine::store(FILE* f) const{
  RESTORE_GA_HEAD head;
  unsigned int x;

  //test
  if(f==NULL) {
    printf("\n\n\t>>> [ERROR] <<<\nNo File to store GA.\n\t>>> [END] <<<\n\n\n");
    return false;
  }

  //head
  head.generationNumber = m_actualGeneration;
  head.cleanStrategies = m_cleanStrategies;
  head.numGeneration = m_generation.size()-1;
  head.numGenes = m_gen.size();
  head.numIndividuals = m_individual.size();
  for(x=0;x<sizeof(RESTORE_GA_HEAD);x++) {
    fprintf(f, "%c",head.buffer[x]);
  }

  //generation
  for(x=m_generation.size();x>0;x--) {
    if(!m_generation[x-1]->store(f)) {
      printf("\n\n\t>>> [ERROR] <<<\nError by writing the generations in the file.\n\t>>> [END] <<<\n\n\n");
      return false;
    }
  }

  //individuals
  for(x=0;x<m_individual.size();x++) {
    if(!m_individual[x]->store(f)) {
      printf("\n\n\t>>> [ERROR] <<<\nError by writing the individuals in the file.\n\t>>> [END] <<<\n\n\n");
      return false;
    }
  }

  // genes
  for(x=0;x<m_gen.size();x++) {
    if(!m_gen[x]->store(f)) {
      printf("\n\n\t>>> [ERROR] <<<\nError by writing the genes in the file.\n\t>>> [END] <<<\n\n\n");
      return false;
    }
  }

  return true;
}

bool SingletonGenEngine::restore(FILE* f, InspectableProxy*& proxyGeneration, InspectableProxy*& proxyGene, PlotOptionEngine* plotEngine, PlotOptionEngine* plotEngineGenContext) {
  RESTORE_GA_HEAD head;
  RESTORE_GA_GENERATION* generation;
  RESTORE_GA_INDIVIDUAL* individual;
  RESTORE_GA_GENE* gene;
  RESTORE_GA_TEMPLATE<int> integer;
  std::string nameGenePrototype;
  std::string name;
  char* buffer;
  int toread;
  GenPrototype* prototype=0;
  unsigned int x;
  int y,z;
  RandGen random;
  Generation* active;
  std::list<Inspectable*> actualContextList;
  std::vector<Gen*> geneStorage;
  std::vector<Individual*> individualStorage;

  //test
  if(f==NULL) {
    printf("\n\n\t>>> [ERROR] <<<\nNo File to restore GA.\n\t>>> [END] <<<\n\n\n");
    return false;
  }

  //head
  for(x=0;x<sizeof(RESTORE_GA_HEAD);x++) {
    if(fscanf(f, "%c", &head.buffer[x])!=1) return false;
  }
  m_actualGeneration = head.generationNumber;
  m_cleanStrategies = head.cleanStrategies;

  SingletonGenFactory::getInstance()->setNumber(head.numGenes);
  SingletonIndividualFactory::getInstance()->setNumber(head.numIndividuals);

  //generation
  for(y=0;y<head.numGeneration;y++) {
    generation = new RESTORE_GA_GENERATION;

    for(x=0;x<sizeof(RESTORE_GA_GENERATION);x++) {
      if(fscanf(f, "%c", &generation->buffer[x])!=1) return false;
    }

    m_restoreGeneration[generation->number] = generation;

    for(x=0;x<(unsigned int)generation->numberIndividuals;x++) {
      for(z=0;z<(int)sizeof(RESTORE_GA_TEMPLATE<int>);z++){
        if(fscanf(f,"%c",&integer.buffer[z])!=1) return false;
      }
      z=integer.value;
      m_restoreIndividualInGeneration[generation->number].push_back(z);
    }
  }

  //individual
  for(y=0;y<head.numIndividuals;y++) {
    individual = new RESTORE_GA_INDIVIDUAL;

    for(z=0;z<(int)sizeof(RESTORE_GA_TEMPLATE<int>);z++){
      if(fscanf(f,"%c",&integer.buffer[z])!=1) return false;
    }
    toread=integer.value;
    buffer = new char[toread];
    for(x=0;x<(unsigned int)toread;x++) {
      if(fscanf(f,"%c",&buffer[x])!=1) return false;
    }
    buffer[x]='\0';
    name = buffer;
    delete[] buffer;

    for(x=0;x<sizeof(RESTORE_GA_INDIVIDUAL);x++) {
      if(fscanf(f, "%c", &individual->buffer[x])!=1) return false;
    }

    m_restoreIndividual[individual->ID] = individual;
    m_restoreNameOfIndividuals[individual->ID] = name;

    for(x=0;x<(unsigned int)individual->numberGenes;x++) {
      for(z=0;z<(int)sizeof(RESTORE_GA_TEMPLATE<int>);z++){
        if(fscanf(f,"%c",&integer.buffer[z])!=1) return false;
      }
      z=integer.value;
      m_restoreGeneInIndividual[individual->ID].push_back(z);
    }
  }

  //gene
  for(y=0;y<head.numGenes;y++) {
    gene = new RESTORE_GA_GENE;

    for(z=0;z<(int)sizeof(RESTORE_GA_TEMPLATE<int>);z++){
      if(fscanf(f,"%c",&integer.buffer[z])!=1) return false;
    }
    toread=integer.value;
    buffer = new char[toread];
    for(x=0;x<(unsigned int)toread;x++) {
      if(fscanf(f,"%c",&buffer[x])!=1) return false;
    }
    buffer[x]='\0';
    nameGenePrototype = buffer;
    delete[] buffer;

    for(x=0;x<sizeof(RESTORE_GA_GENE);x++) {
      if(fscanf(f, "%c", &gene->buffer[x])!=1) return false;
    }

    //find prototype
    for(x=0;x<(unsigned int)m_prototype.size();x++) {
      if(m_prototype[x]->getName().compare(nameGenePrototype)==0) {
        prototype = m_prototype[x];
        break;
      }
    }

    if(!prototype->restoreGene(f,gene,geneStorage)) {
      printf("\n\n\t>>> [ERROR] <<<\nError by restoring the genes.\n\t>>> [END] <<<\n\n\n");
      return false;
    }
  }

  //add genes to the engine
  for(x=0;x<(unsigned int)head.numGenes;x++) {
    addGen(geneStorage[x]);
  }

  //restore individual
  if(!Individual::restore(head.numIndividuals,m_restoreNameOfIndividuals,m_restoreIndividual,m_restoreGeneInIndividual,individualStorage)) {
    printf("\n\n\t>>> [ERROR] <<<\nError by restoring the individuals.\n\t>>> [END] <<<\n\n\n");
    return false;
  }
  //add individuals to the engine
  for(x=0;x<(unsigned int)head.numIndividuals;x++) {
    addIndividual(individualStorage[x]);
  }
  if(!Individual::restoreParent(head.numIndividuals,m_restoreIndividual)) {
    printf("\n\n\t>>> [ERROR] <<<\nError by restoring the individuals parent links.\n\t>>> [END] <<<\n\n\n");
    return false;
  }

  //restore generation
  if(!Generation::restore(head.numGeneration,m_restoreGeneration,m_restoreIndividualInGeneration)) {
    printf("\n\n\t>>> [ERROR] <<<\nError by restoring the generation.\n\t>>> [END] <<<\n\n\n");
    return false;
  }

  //restore gene context
  if(!GenContext::restore()) {
    printf("\n\n\t>>> [ERROR] <<<\nError by restoring the context.\n\t>>> [END] <<<\n\n\n");
    return false;
  }

  // Control values
  active = m_generation[0];
  if(plotEngine!=0) {
    actualContextList.clear();
    actualContextList.push_back(active);
    proxyGeneration = new InspectableProxy(actualContextList);
    plotEngine->addInspectable(&(*proxyGeneration));
    plotEngine->init();
    plotEngine->plot(1.0);
  }
  if(plotEngineGenContext!=0) {
    actualContextList.clear();
    for(std::vector<GenPrototype*>::const_iterator iter = m_prototype.begin(); iter!=m_prototype.end(); iter++) {
      actualContextList.push_back((*iter)->getContext(active));
      (*iter)->getContext(active)->update();
    }
    proxyGene = new InspectableProxy(actualContextList);
    plotEngineGenContext->addInspectable(&(*proxyGene));
    plotEngineGenContext->init();
    plotEngineGenContext->plot(1.0);
  }

  y=getActualGenerationNumber();
  for(x=1;x<=(unsigned int)y;x++) {
    m_actualGeneration = x;
    update();
    measureStep(x+1, proxyGeneration, proxyGene, plotEngine, plotEngineGenContext);
  }

  select();
  crossover(&random);

  return true;
}
