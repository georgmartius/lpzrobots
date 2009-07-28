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
 *   This class is a factory for the class Gen. It use some parameters and *
 *   a GenPrototype to create new Gens. (randomized gens or mutated gens)  *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2009-07-28 09:38:23  robot12
 *   update the Singleton::destroy
 *
 *   Revision 1.3  2009/07/21 08:37:58  robot12
 *   add some comments
 *
 *   Revision 1.2  2009/06/29 14:27:13  robot12
 *   finishing the Gen Factory and add some comments
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.3  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.2  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef SINGLETONGENFACTORY_H_
#define SINGLETONGENFACTORY_H_

//forward declaration
class GenContext;
class Gen;
class Individual;
class GenPrototype;
class IValue;

/**
 * This is the factory for the class Gen. It gives 3 Methodes to generate new gens. (random,value and mutation)
 * Over this is the class as singleton concepted. Only one Factory for a run.
 *
 * It use by every method the GenPrototype to be independent from the type of the Gen.
 */
class SingletonGenFactory {
public:
	/**
	 * this method is to become the only existing factory
	 * @return (SingletonGenFactory*) the one and only factory
	 */
	inline static SingletonGenFactory* getInstance(void) {if(m_factory==0)m_factory = new SingletonGenFactory();return m_factory;}

	/**
	 * this method is to destroy the one and only factory.
	 */
	inline static void destroyGenFactory(void) {if(m_factory!=0){delete m_factory;m_factory=0;}}

	// 3 methodes to create an Gen
	/**
	 * random generation of a new gen.
	 * @param context (GenContext*) the context of the new Gen
	 * @param individual (Individual*) the individual, where the gen is part of.
	 * @param prototype (GenPrototype*) the prototype of the gen, which should be create.
	 * @return (Gen*) the new Gen
	 */
	Gen* createGen(GenContext* context, Individual* individual, GenPrototype* prototype)const;
	/**
	 * this function generate a new Gen by mutate a old Gen
	 * @param context (GenContext*) the context of the new Gen
	 * @param individual (Individual*) the individual, where the gen is part of.
	 * @param prototype (GenPrototype*) the prototype of the gen, which should be create.
	 * @param oldContext (GenContext*) the Context of the old Gen
	 * @param oldIndividual (Individual*) the individua, where the olg gen is part of.
	 * @param oldGen (Gen*) the old Gen
	 * @param mutate (bool) should be mutate?
	 * @return (Gen*) the new (or old gen)
	 */
	Gen* createGen(GenContext* context, Individual* individual, GenPrototype* prototype, GenContext* oldContext, Individual* oldIndividual, Gen* oldGen, bool mutate=false)const;		// copy + mutation
	/**
	 * create a new Gen by a giving value
	 * @param context (GenContext*) the context of the new Gen
	 * @param individual (Individual*) the individual, where the gen is part of.
	 * @param prototype (GenPrototype*) the prototype of the gen, which should be create.
	 * @param value (IValue*) the value of the new gen
	 * @return
	 */
	Gen* createGen(GenContext* context, Individual* individual, GenPrototype* prototype, IValue* value);																				// value

private:
	/**
	 * the one and only factory
	 */
	static SingletonGenFactory* m_factory;

	/**
	 * counter for giving Gens a individual ID
	 */
	static int m_number;

	/**
	 * disable the default constructor
	 */
	SingletonGenFactory();

	/**
	 * disable destructor
	 */
	virtual ~SingletonGenFactory();
};

#endif /* SINGLETONGENFACTORY_H_ */
