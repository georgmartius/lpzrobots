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
 *   This class is the engine of the gen. alg. It control all elements     *
 *   inside, prepare the next steps and hold the alg. on running.          *
 *                                                                         *
 *   $Log$
 *   Revision 1.11  2009-10-21 14:08:06  robot12
 *   add restore and store functions to the ga package
 *
 *   Revision 1.10  2009/08/11 12:57:38  robot12
 *   change the genetic algorithm (first crossover, second select)
 *
 *   Revision 1.9  2009/07/28 13:19:27  robot12
 *   add member variable m_cleanStrategies
 *
 *   Revision 1.8  2009/07/21 08:39:01  robot12
 *   rename "crosover" to crossover
 *
 *   Revision 1.7  2009/07/15 12:53:36  robot12
 *   some bugfix's and new functions
 *
 *   Revision 1.6  2009/07/06 15:06:35  robot12
 *   bugfix
 *
 *   Revision 1.5  2009/07/02 15:25:41  robot12
 *   update
 *
 *   Revision 1.4  2009/06/30 10:30:04  robot12
 *   GenEngine finish and some comments added
 *
 *   Revision 1.3  2009/05/12 13:29:26  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.2  2009/05/11 14:08:52  robot12
 *   patch some bugfix....
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.6  2009/04/30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.5  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.4  2009/04/29 14:32:28  robot12
 *   some implements... Part4
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

#ifndef SINGLETONGENENGINE_H_
#define SINGLETONGENENGINE_H_

//includes
#include <vector>
#include <map>
#include <string>
#include <selforg/randomgenerator.h>
#include <selforg/inspectableproxy.h>
#include <selforg/storeable.h>
#include <restore.h>

//forward declaration
class Gen;
class GenPrototype;
class GenContext;
class Individual;
class Generation;
class IMutationStrategy;
class IMutationFactorStrategy;
class ISelectStrategy;
class IGenerationSizeStrategy;
class IRandomStrategy;
class IValue;
class IFitnessStrategy;

//forward declaration for LPZROBOTS
class PlotOptionEngine;

/**
 * This is the engine of the gen. alg.
 *
 * Over this is the class as singleton concepted. Only one engine for a run.
 */
class SingletonGenEngine : public Storeable{
public:
	/**
	 * this function returns a set of all registered GenPrototypes.
	 * @return (vector<GenPrototype*>&) the set
	 */
	inline const std::vector<GenPrototype*>& getSetOfGenPrototyps(void) const {return m_prototype;}

	/**
	 * this function return the number of Generation inside.
	 * @return (int) the number of Generation in the storage
	 */
	inline int getNumGeneration(void) const {return m_generation.size();}

	/**
	 * this function returns one Generation
	 * @param x (int) the index of the Generation which is searched
	 * @return (Generation*) the searched Generation. If x not a right index, than the result is zero.
	 */
	inline Generation* getGeneration(int x) {if(x<getNumGeneration())return m_generation[x];return NULL;}

	/**
	 * returns the actual generation number, on which the alg. work
	 * @return (int) the actual generation number
	 */
	inline int getActualGenerationNumber(void) const {return m_actualGeneration;}

	/**
	 * returns the actual Generation.
	 * @return (Generation*) the actual generation
	 */
	inline Generation* getActualGeneration(void) {return m_generation[m_actualGeneration];}

	/**
	 * returns the number of individual inside the alg.
	 * @return (int) number of individual
	 */
	inline int getNumIndividual(void) const {return m_individual.size();}

	/**
	 * returns one individual
	 * @param x (int) index of the individual which is searched.
	 * @return (Individual*) the searched individual
	 */
	inline Individual* getIndividual(int x) const {if(x<getNumIndividual())return m_individual[x];return NULL;}

	/**
	 * registered a GenPrototype in the engine.
	 * @param prototype (GenPrototype*) the prototype which should be registered.
	 */
	inline void addGenPrototype(GenPrototype* prototype) {m_prototype.push_back(prototype);}

	/**
	 * registered a Gen in the engine.  Normal only used by the alg. self.
	 * @param gen (Gen*) the Gen which should be registered.
	 */
	inline void addGen(Gen* gen) {m_gen.push_back(gen);}

	/**
	 * returns one gene
	 * @param x (int) index of the gene which is searched
	 * @return (Gen*) the searched gene
	 */
	inline Gen* getGen(int x)const {if((unsigned int)x<m_gen.size())return m_gen[x];return NULL;}

	/**
	 * registered a individual in the engine.  Normal only used by the alg. self.
	 * @param individual (Individual*) the individual
	 */
	inline void addIndividual(Individual* individual) {m_individual.push_back(individual);}

	/**
	 * add a Generation to the alg. Normal only used by the alg. self.
	 * @param generation (Generation*) the generation
	 */
	inline void addGeneration(Generation* generation) {m_generation.push_back(generation);}

	/**
	 * prepare the alg. and create his fisrt generation.
	 * @param startSize (int) Number of individual with which the alg. will be start.
	 * @param numChildren (int) Number of individual which will be created by crossover.
	 * @param random (RandGen*) A random generator
	 * @param withUpdate (bool) if true, than makes this function on the end a update.
	 */
	void generateFirstGeneration(int startSize, int numChildren, RandGen* random, bool withUpdate = true);

	/**
	 * prepare the next generation. Mean create the GenContext for every GenPrototype.
	 * @param size (int) size of the next generation
	 * @param numChildren (int) Number of individual which will be created by crossover
	 */
	void prepareNextGeneration(int size, int numChildren);

	/**
	 * prepare the first generation with "generateFirstGeneration" and init the measures.
	 * @param startSize (int) Number of individual with which the alg. will be start.
	 * @param numChildren (int) Number of individual which will be created by crossover
	 * @param proxyGeneration (InspectableProxy*&) the generation which is controlled by measures
	 * @param proxyGene (InspectableProxy*&) the proxy for control measure
	 * @param random (RandGen*) A random generator
	 * @param plotEngine (PlotOptionEngine*) logging the data for later control
	 * @param plotEngineGenContext (PlotOptionEngine*) logging the data of the GenContext for later control
	 * @param withUpdate (bool) is needed for the function "generateFirstGeneration"
	 */
	void prepare(int startSize, int numChildren, InspectableProxy*& proxyGeneration, InspectableProxy*& proxyGene, RandGen* random, PlotOptionEngine* plotEngine = 0, PlotOptionEngine* plotEngineGenContext = 0, bool withUpdate = true);

	/**
	 * return the size of the next generation.
	 * @return (int) the size
	 */
	int getNextGenerationSize();

	/**
	 * makes a step in the measure
	 * @param time (double) the time stamp in the measure
	 * @param proxyGeneration (InspectableProxy*&) the generation which is controlled by measures
	 * @param proxyGene (InspectableProxy*&) the proxy for control measure
	 */
	void measureStep(double time,InspectableProxy*& proxyGeneration, InspectableProxy*& proxyGene, PlotOptionEngine* plotEngine = 0, PlotOptionEngine* plotEngineGenContext = 0);

	/**
	 * this function is for a automatically run of the gen. alg.
	 * @param startSize (int) Number of individual with which the alg. will be start.
	 * @param numChildren (int) Number of individual which will be created by crossover.
	 * @param numGeneration (int) Number of generation after this the alg. will end.
	 * @param random (RandGen*) a random generator
	 * @param plotEngine (PlotOptionEngine*) logging the data for later control
	 * @param plotEngineGenContext (PlotOptionEngine*) logging the data of the GenContext for later control
	 */
	void runGenAlg(int startSize, int numChildren, int numGeneration, RandGen* random, PlotOptionEngine* plotEngine = 0, PlotOptionEngine* plotEngineGenContext = 0);

	/**
	 * make the select of the actual generation and transfer it to the next generation
	 * @param createNextGeneration (bool) normal=true. should be the next generation prepare
	 */
	void select(bool createNextGeneration=true);

	/**
	 * in the actual generation it will generate the children of the living individual
	 * @param random (RanGen*) random generator
	 */
	void crossover(RandGen* random);

	/**
	 * this function makes a update on the statistical data of the gen. alg.
	 * @param factor (double) the factor is for the whisker distance
	 */
	void update(double factor = 1.5);

	/**
	 * decid the select strategy
	 * @param strategy (ISelectStrategy*) the select strategy of the alg.
	 */
	inline void setSelectStrategy(ISelectStrategy* strategy) {m_selectStrategy = strategy;}

	/**
	 * decide the generation size strategy.
	 * @param strategy (IGenerationSizeStrategy*) the generation size strategy of the alg.
	 */
	inline void setGenerationSizeStrategy(IGenerationSizeStrategy* strategy) {m_generationSizeStrategy = strategy;}

	/**
	 * decide the fitness strategy.
	 * @param strategy (IFitnessStrategy*) the fitness strategy of the alg.
	 */
	inline void setFitnessStrategy(IFitnessStrategy* strategy) {m_fitnessStrategy = strategy;}

	/**
	 * calculate for a individual the fitness value.
	 * @param individual (Individual*) the individual for which the fitness should be calculated
	 * @return (double) the fitness value.
	 */
	double getFitness(const Individual* individual);

	/**
	 * returns the best individual (where the fitness is next to zero) which the alg. have found.
	 * @return (Individual*) the best.
	 */
	Individual* getBestIndividual(void);

	/**
	 * returns a complete list of the inheritance of all individual.
	 * @param withMutation (bool) normal=true. if every individual should be marked if it mutated.
	 * @return (string) the list
	 */
	std::string getIndividualRoot(bool withMutation=true)const;

	/**
	 * returns a list of all individual and there gens values.
	 * @return (string) the list
	 */
	std::string getAllIndividualAsString(void)const;

	/** stores the object to the given file stream (binary).
	 */
	virtual bool store(FILE* f) const;

	/** loads the object from the given file stream (binary).
	 */
	virtual bool restore(FILE* f);

	/**
	 * returns the only existing engine.
	 * @return (SingletonGenEngine*) the engine
	 */
	inline static SingletonGenEngine* getInstance(void) { if(m_engine==0) m_engine = new SingletonGenEngine();return m_engine; }

	/**
	 * destroy the only existing engine.
	 */
	inline static void destroyGenEngine(bool cleanStrategies=false) {getInstance()->m_cleanStrategies=cleanStrategies; if(m_engine!=0){delete m_engine;m_engine=0;}}

protected:
	/**
	 * managment storage for all Genprototypes.
	 */
	std::vector<GenPrototype*> m_prototype;

	/**
	 * managment storage for all Generation
	 */
	std::vector<Generation*> m_generation;

	/**
	 * management storage for all Individual
	 */
	std::vector<Individual*> m_individual;

	/**
	 * management storage for all Gens
	 */
	std::vector<Gen*> m_gen;

	/**
	 * the number of the actual generation
	 */
	int m_actualGeneration;

	/**
	 * the select strategy of the alg.
	 */
	ISelectStrategy* m_selectStrategy;

	/**
	 * the fitness strategy of the alg.
	 */
	IFitnessStrategy* m_fitnessStrategy;

	/**
	 * the generation size strategy of the alg.
	 */
	IGenerationSizeStrategy* m_generationSizeStrategy;

	/**
	 * the one and only GenEngine.
	 */
	static SingletonGenEngine* m_engine;

	/**
	 * flag for clean the seted strategies
	 */
	bool m_cleanStrategies;

	/**
	 * Map for restoring the generations from a run before
	 */
	std::map<int,RESTORE_GA_GENERATION*> m_restoreGeneration;

	/**
	 * Map for restoring the individuals from a run before
	 */
	std::map<int,RESTORE_GA_INDIVIDUAL*> m_restoreIndividual;

	/**
   * Map for restoring the individual generation link from a run before
   */
	std::map<int,std::vector<int> > m_restoreIndividualInGeneration;

	/**
   * Map for restoring the genes individual link from a run before
   */
	std::map<int,std::vector<int> > m_restoreGeneInIndividual;

	/**
	 * Map for restoring the names of the individuals
	 */
	std::map<int,std::string> m_restoreNameOfIndividuals;

private:
	/**
	 * disable the default constructor
	 */
	SingletonGenEngine();

	/**
	 * disable the default destructor
	 * @return
	 */
	virtual ~SingletonGenEngine();
};

#endif /* SINGLETONGENENGINE_H_ */
