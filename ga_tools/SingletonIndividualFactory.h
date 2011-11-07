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
	inline static void destroyFactory(void) {if(m_factory!=0){delete m_factory; m_factory=NULL;}}

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

	//reset m_number inside restore
	/**
	 * set the member variable m_number to number
	 * @param number (int) the new value
	 */
	inline void setNumber(int number) {m_number=number;}

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
