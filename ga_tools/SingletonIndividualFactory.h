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
 *   This class is a factory for the class individual. It use the          *
 *   GenFactory to create a new individual, because the individual is a    *
 *   combination of Gens.                                                  *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2009-06-29 15:20:25  robot12
 *   finishing Individual Factory and add some comments
 *
 *   Revision 1.2  2009/05/11 14:08:52  robot12
 *   patch some bugfix....
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.5  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.4  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef SINGLETONINDIVIDUALFACTORY_H_
#define SINGLETONINDIVIDUALFACTORY_H_

//includes
#include <selforg/randomgenerator.h>

//forward declaration
class Gen;
class Generation;

//ga_tools includes
#include "Individual.h"

/**
 * this is a factory for the individual class. It use the SingletonGenFactory to create new individuals.
 * It have 2 methods to create a individual. (random and recombination)
 *
 * Over this is the class as singleton concepted. Only one Factory for a run.
 */
class SingletonIndividualFactory {
public:
	/**
	 * this method gives the one and only existing factory back.
	 * @return (SingletonIndividualFactory*) the factory
	 */
	inline static SingletonIndividualFactory* getInstance(void) {
		if(m_factory==0)m_factory = new SingletonIndividualFactory;
		return m_factory;
	}

	/**
	 * destroy the only existing factory
	 */
	inline static void destroyFactory(void) {delete m_factory; m_factory=NULL;}

	// 2 methods to create an individual
	/**
	 * random creation by random creation of Gen for every GenPrototype.
	 * @param name (string) the name of the new individual. Will be automaticly created
	 * @return (Individual*) the new individual
	 */
	Individual* createIndividual(std::string name=createName())const;																		// random
	/**
	 * create a new individual by recombination of the gens of there parents
	 * @param individual1 (Individual*) parent 1
	 * @param individual2 (Individual*) parent 2
	 * @param random (RandGen*) a random generator
	 * @param name (string) the name of the new individual. Will be automaticly created
	 * @return (Individual*) the new individual
	 */
	Individual* createIndividual(Individual* individual1, Individual* individual2, RandGen* random, std::string name=createName())const;	// recombinate

private:
	/**
	 * the one and only factory
	 */
	static SingletonIndividualFactory* m_factory;

	/**
	 * a counter for individual IDs
	 */
	static int m_number;

	/**
	 * disable the default constructor
	 * only usable for itself
	 */
	SingletonIndividualFactory();

	/**
	 * disable the default destructor
	 * only usable for itself
	 */
	virtual ~SingletonIndividualFactory();

	/**
	 * methode to generate a name for the new individual automatically
	 * @return (string) the name
	 */
	inline static std::string createName(void) {std::string s = "Ind ";char buffer[128];sprintf(buffer,"%i",m_number);s+=buffer;return s;}
};

#endif /* SINGLETONINDIVIDUALFACTORY_H_ */
