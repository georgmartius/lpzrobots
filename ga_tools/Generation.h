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

#ifndef GENERATION_H_
#define GENERATION_H_

// standard includes
#include <string>
#include <vector>
#include <map>
#include <selforg/randomgenerator.h>
#include <selforg/inspectable.h>

// forward declarations
class Individual;
struct RESTORE_GA_GENERATION;

// gen. alg. includes

/**
 * The Generation class
 *
 *   This class is used for grouping some individuals which representing
 *   one step in the gen. alg. (called generation). For this it save all
 *   individual which are part of this generation. Also it have an Number
 *   like a ID, which make this generation individual.
 *
 *   All Generations inside the gen.alg. are only saved in the GenEngine.
 */
class Generation : public Inspectable {
public:
	/**
	 * constructor to create a Generation. Information which the class need are
	 * the generation number, the size of it and how many individual don t come
	 * in the next generation (killRate).
	 *
	 * @param generationNumber (int) The ID of the Generation.
	 * @param size (int) The Size of this Generation. Means how many individual are lives in this generation
	 * @param numChildren (int) Number of individual which will be created by crossover
	 */
	Generation(int generationNumber, int size, int numChildren);

	/**
	 * destructor to delete a GenContext.
	 */
	virtual ~Generation();

	/**
	 * [inline], [const]
	 * This function gives the ID (number) of the generation back.
	 *
	 * @return (int) The ID
	 */
	inline int getGenerationNumber(void)const {return m_generationNumber;}

	/**
	 * [inline], [const]
	 * This function gives the size which is planed for this generation back.
	 *
	 * @return (int) The planed size
	 */
	inline int getSize(void)const {return m_size;}

	/**
	 * [inline], [const]
	 * This function gives the actual size (number of individuals inside the generation) back.
	 *
	 * @return (int) current size
	 */
	inline int getCurrentSize(void)const {return m_individual.size();}

	/**
	 * [inline], [const]
	 * This function gives the number of children back, which will be created by crossover.
	 *
	 * @return (int) the number of children
	 */
	inline int getNumChildren(void)const {return m_numChildren;}

	/**
	 * [individual], [const]
	 * This function gives one individual from this generation back.
	 *
	 * @param x (int) the index of the searched individual
	 *
	 * @return (Individual*) The individual. If 0, if the param x is not inside the index range
	 */
	inline Individual* getIndividual(int x)const {if(x<getCurrentSize())return m_individual[x];return NULL;}

	/**
	 * [inline], [const]
	 * This function gives all individual back.
	 *
	 * @return (vector<Individual*>&) all individual inside the generation
	 */
	inline const std::vector<Individual*>& getAllIndividual(void)const {return m_individual;}

	/**
   * [inline], [const]
   * This function gives all individual back which aren't have the fitness value calculated.
   *
   * @return (vector<Individual*>&) all individual inside the generation
   */
  std::vector<Individual*>* getAllUnCalculatedIndividuals(void)const;

	/**
	 * This function insert an individual in the generation.
	 *
	 * @param individual (Individual*) the individual which should be insert in the generation
	 */
	void addIndividual(Individual* individual);

	/**
	 * This function makes an crossOver whit the existing individuals to become from the current size the planed size.
	 *
	 * @param random (RandGen*) a pseudo number generator.
	 */
	void crossover(RandGen* random);

	/**
	 * returns a string which represent all individual in this generation.
	 *
	 * @return (string) the string
	 */
	std::string getAllIndividualAsString(void)const;

	/**
	 * returns all fitness values from the individuals.
	 *
	 * @return (vector<double> the fitness values.
	 */
	std::vector<double>* getAllFitness(void)const;

	/**
	 * This function updates the statistical values
	 * @param factor (double) normal 1.5    Is needed for the data analysation
	 */
	void update(double factor = 1.5);

	/**
	 * store a generation in a file
	 * @param f (FILE*) the file in which should be stored
	 * @return (bool) true if all ok
	 */
	bool store(FILE* f)const;

	/**
	 * restore all generation from a restore structure
	 *
	 * remember the individuals must be restored before
	 *
	 * @param numberGeneration (int) number of generations which should be restored
	 * @param generationSet (map<int,RESTORE_GA_GENERATION*>) the structures which should be restored
	 * @param linkSet (map<int,vector<int>>) the linkings between the generation and the individuals
	 * @return (bool) true if all ok
	 */
	static bool restore(int numberGeneration, std::map<int,RESTORE_GA_GENERATION*>& generationSet, std::map<int,std::vector<int> >& linkSet);

protected:
	/**
	 * (int)
	 * The generation number (ID)
	 */
	int m_generationNumber;

	/**
	 * (vector<Individual*>)
	 * The storage for the individuals, which are part of this generation. (NO deleting)
	 */
	std::vector<Individual*> m_individual;

	/**
	 * (int)
	 * The planed size of the generation.
	 */
	int m_size;

	/**
	 * (int)
	 * The number of children
	 */
	int m_numChildren;

private:
	/**
	 * disable the default constructor
	 */
	Generation();

	/**
	 * the under quartil
	 */
	double m_q1;

	/**
	 * the upper quartil
	 */
	double m_q3;

	/**
	 * the min
	 */
	double m_min;

	/**
	 * the max
	 */
	double m_max;

	/**
	 * the average
	 */
	double m_avg;

	/**
	 * the median
	 */
	double m_med;

	/**
	 * the under whisker
	 */
	double m_w1;

	/**
	 * the upper whisker
	 */
	double m_w3;

	/**
	 * the best fitness value inside the generation
	 */
	double m_best;

	/**
	 * the number of individual inside the generation (will be)
	 */
	double m_dSize;

	/**
	 * the number of individual which will be created by crossover.
	 */
	double m_dNumChildren;
};

#endif /* GENERATION_H_ */
