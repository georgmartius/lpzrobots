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
 *   Informative Beschreibung der Klasse                                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-05-11 14:08:52  robot12
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

#include <selforg/randomgenerator.h>

class Gen;
class Generation;

#include "Individual.h"

class SingletonIndividualFactory {
public:
	inline static SingletonIndividualFactory* getInstance(void) {
		if(m_factory==0)m_factory = new SingletonIndividualFactory;
		return m_factory;
	}

	inline static void destroyFactory(void) {delete m_factory; m_factory=NULL;}

	// 2 methodes to create an individual
	Individual* createIndividual(std::string name=createName())const;																		// random
	Individual* createIndividual(Individual* individual1, Individual* individual2, RandGen* random, std::string name=createName())const;	// recombinate

private:
	static SingletonIndividualFactory* m_factory;
	static int m_number;

	SingletonIndividualFactory();
	virtual ~SingletonIndividualFactory();

	inline static std::string createName(void) {std::string s = "Ind ";char buffer[128];sprintf(buffer,"%i",m_number);s+=buffer;return s;}
};

#endif /* SINGLETONINDIVIDUALFACTORY_H_ */
