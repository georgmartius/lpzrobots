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
 *   This class represent one individual of the complete gen. alg. It have *
 *   some gens and a fitness.                                              *
 *                                                                         *
 *   $Log$
 *   Revision 1.7  2009-06-29 14:52:14  robot12
 *   finishing Individual and add some comments
 *
 *   Revision 1.6  2009/05/12 13:29:25  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.5  2009/05/11 14:08:51  robot12
 *   patch some bugfix....
 *
 *   Revision 1.4  2009/05/04 15:27:56  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.7  2009/04/30 14:32:34  robot12
 *   some implements... Part5
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

#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

//includes
#include <vector>
#include <string>

//ga_tools includes
#include "Gen.h"
#include "SingletonGenEngine.h"

/**
 * This class represent one individual of the complete gen. alg. It have some gens and a fitness.
 */
class Individual {
public:
	/**
	 * constructor
	 * Create the individual with the name "name"
	 * @param name (sting) the name of the new individual
	 * @param id (int) an unique ID
	 * @param p1 (Individual*) parent 1 or zero if it random generate
	 * @param p2 (Individual*) parent 2 or zero if it random generate
	 */
	Individual(std::string name, int id, Individual* p1=0, Individual* p2=0);

	/**
	 * default destructor
	 */
	virtual ~Individual();

	/**
	 * return the ID of the individual
	 * @return (int) the ID
	 */
	inline int getID(void)const {return m_ID;}

	/**
	 * return the name of the individual
	 * @return (string) the name
	 */
	inline std::string getName(void)const {return m_name;}

	/**
	 * return the size of the individual. This mean the number of gens inside the individual.
	 * @return (int) number of gens
	 */
	inline int getSize(void)const {return m_gene.size();}

	/**
	 * return a gen of the individual
	 * @param x (the index of the gen
	 * @return (Gen*) the searched gen. NULL if the index isn't right
	 */
	inline Gen* getGen(int x)const {if(x<getSize())return m_gene[x];return NULL;}

	/**
	 * add a gen to the individual.
	 * @param gen (Gen*) the new gen
	 */
	inline void addGen(Gen* gen) {m_gene.push_back(gen);}

	/**
	 * returns all gens of the individual
	 * @return (vector<Gen*>&) all gens
	 */
	inline const std::vector<Gen*>& getGene(void)const {return m_gene;}

	/**
	 * remove a specified gen from the individual
	 * @param gen (Gen*) the gen which should removed from the individual
	 */
	void removeGen(Gen* gen);

	/**
	 * remove a specified gen from the individual
	 * @param x (the index of the gen, which should b removed.
	 */
	void removeGen(int x);

	/**
	 * this function calculate the fitness value of the individual
	 * @return
	 */
	double getFitness()const;

	/**
	 * this select the individual as a product of mutation.
	 */
	inline void setMutated(void) {m_mutated=true;}

	/**
	 * returns parent 1 of the individual
	 * @return (Individual*) parent 1 (could be zero if individual a random generation)
	 */
	inline const Individual* getParent1(void)const {return m_parent1;}

	/**
	 * returns parent 2 of the individual
	 * @return (Individual*) parent 2 (could be zero if individual a random generation)
	 */
	inline const Individual* getParent2(void)const {return m_parent2;}

	/**
	 * test if the individual a product of mutation
	 * @return (bool) true if it a product of mutation
	 */
	inline bool isMutated(void)const {return m_mutated;}

	/**
	 * returns a string, which repesent the individual (for logging)
	 * @return (string) the string representation of the individual
	 */
	std::string IndividualToString(void)const;

	/**
	 * returns a string with the parents and a mark for mutation
	 * @param withMutation (bool) should mutation part of the string
	 * @return (string) the result
	 */
	std::string RootToString(bool withMutation=true)const;

protected:
	/**
	 * the name of the individual
	 */
	std::string m_name;

	/**
	 * the ID of the individual
	 */
	int m_ID;

	/**
	 * the gens inside the individual
	 */
	std::vector<Gen*> m_gene;

	/**
	 * parent 1
	 */
	Individual* m_parent1;

	/**
	 * parent 2
	 */
	Individual* m_parent2;

	/**
	 * remember if the individual a product of mutation
	 */
	bool m_mutated;

private:
	/**
	 * disable the default constructor
	 */
	Individual();
};

#endif /* INDIVIDUAL_H_ */
