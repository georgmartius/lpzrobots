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

#ifndef GENPROTOTYPE_H_
#define GENPROTOTYPE_H_

// standard includes
#include <string>
#include <map>

// forward declarations
class Generation;
class GenContext;
class IValue;
class Gen;

// gen. alg. includes
#include "IRandomStrategy.h"
#include "IMutationStrategy.h"
#include "restore.h"

/**
 * The GenPrototype class.
 *
 *   This class is used for group some gens and is needed from the
 *   genFactory. It saves all GenContexte which are use this prototype.
 *   The group of gens becomes whit it an name.
 *
 *   The prototypes inside the gen. alg. are saved in the GenContext, in
 *   the Gen and in the GenEngine (only here can be deleted!!!).
 */
class GenPrototype {
public:
	/**
	 * constructor to create a GenPrototype. Information which the class need are
	 * the name of the gen pool group, a strategy how can a Gen of this group be
	 * created (for the IValue) and a strategy how can a Gen mutate.
	 *
	 * @param name (string) Name of the group
	 * @param randomStrategy (IRandomStrategy*) the strategy for creating a gen
	 * @param mutationStrategy (IMutationStrategy*) the strategy for mutating a gen
	 */
	GenPrototype(std::string name, IRandomStrategy* randomStrategy, IMutationStrategy* mutationStrategy);

	/**
	 * destructor to delete a GenContext.
	 */
	virtual ~GenPrototype();

	/**
	 * [inline], [const]
	 * This function gives the name of the prototype back.
	 *
	 * @return (string) the name
	 */
	inline std::string getName(void)const {return m_name;}

	/**
	 * [inline], [const]
	 * This function gives a random value (IValue) which are with the randomStrategy is generated back.
	 */
	inline IValue* getRandomValue(void)const {return m_randomStrategy->getRandomValue();}

	/**
	 * This function insert a GenContext in the GenPrototype.
	 *
	 * @param generation (Generation*) to which Generation is the Context related.
	 * @param context (GenContext*) the context which should be insert
	 */
	void insertContext(Generation* generation, GenContext* context);

	/**
	 * This function gives the context which is relatedto the Eneration "generation" back.
	 *
	 * @param generation (Generation*) the related generation
	 *
	 * @return (GenContext*) the searched context
	 */
	GenContext* getContext(Generation* generation);

	/**
	 * [const]
	 * This function mutate the given gen.
	 *
	 * @param context (GenContext*) param is needed by the mutationStrategy
	 * @param individual (Individual*) param is needed by the mutationStrategy
	 * @param oldGen (Gen*) the gen which should be mutate
	 * @param oldContext (GenContext*) param is needed by the mutationStrategy
	 *
	 * @return (Gen*) The new mutated gen
	 */
	Gen* mutate(GenContext* context, Individual* individual, Gen* oldGen, GenContext* oldContext)const;

	/**
	 * [const]
	 * This function gives the mutation probability back (from the mutation strategy)
	 *
	 * @return (int) The mutation probability. Maybe the Typ int will be changed.
	 */
	int getMutationProbability(void)const;

	/**
	 * restore gene and the value
	 * @param f (FILE*) here is the value inside
	 * @param gene (RESTORE_GA_GENE*) this gene is to restore
	 * @return (bool) true if all ok
	 */
	bool restoreGene(FILE* f, RESTORE_GA_GENE* gene, std::vector<Gen*>& storage);

protected:
	/**
	 * (string)
	 * the name
	 */
	std::string m_name;

	/**
	 * (map<Generation*, GenContext*>)
	 * The storage for the GenContexte. It related the contexte to the generations.
	 */
	std::map<Generation*,GenContext*> m_context;

	/**
	 * (IRandomStrategy*)
	 * the random strategy
	 */
	IRandomStrategy* m_randomStrategy;

	/**
	 * (IMutationStrategy*)
	 * the mutation strategy
	 */
	IMutationStrategy* m_mutationStrategy;

private:
	/**
	 * disable the default constructor
	 */
	GenPrototype();
};

#endif /* GENPROTOTYPE_H_ */
