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
 *   Revision 1.8  2009-07-21 08:39:01  robot12
 *   rename "crosover" to crossover
 *
 *   Revision 1.7  2009/06/29 15:30:11  robot12
 *   finishing Generation and add some comments
 *
 *   Revision 1.6  2009/05/14 15:29:54  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.5  2009/05/12 13:29:25  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.4  2009/05/07 14:47:47  robot12
 *   some comments
 *
 *   Revision 1.3  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.5  2009/04/30 14:32:34  robot12
 *   some implements... Part5
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
 *   Revision 1.1  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef GENERATION_H_
#define GENERATION_H_

// standard includes
#include <string>
#include <vector>
#include <selforg/randomgenerator.h>
#include <selforg/inspectable.h>

// forward declarations
class Individual;

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
	 * @param kill (int) How many individual will be die.
	 */
	Generation(int generationNumber, int size, int kill);

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
	 * This function gives the killRate back.
	 *
	 * @return (int) the killRate
	 */
	inline int getKillRate(void)const {return m_kill;}

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
	 * The killRate
	 */
	int m_kill;

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
	 * the number of individual which will be killed by swaping the generation
	 */
	double m_dKill;
};

#endif /* GENERATION_H_ */
