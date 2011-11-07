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

#ifndef GENCONTEXT_H_
#define GENCONTEXT_H_

// standard includes
#include <vector>
#include <algorithm>
#include <selforg/inspectable.h>

// forward declarations
class Gen;
class GenPrototype;

// gen. alg. includes

/**
 * The GenContext class.
 *   This class is used for create a context for some gens. This mean it
 *   saves all gens which have the same prototype and are a part of an
 *   individual which are in ONE generation. It can be useful for some
 *   statistical calculation or for optimizing the mutation factor.
 *
 *   The Gen Context is inside the gen. alg. only saved in the
 *   GenPrototype.
 */
class GenContext : public Inspectable {
public:
	/**
	 * constructor to create a GenContext. Information which the class need are
	 * the prototype (name an group of gens).
	 *
	 * @param prototype (GenPrototype*) Pointer to the prototype.
	 */
	GenContext(GenPrototype* prototype);

	/**
	 * destructor to delete a GenContext.
	 */
	virtual ~GenContext();

	/**
	 * [inline], [const]
	 *
	 * This function gives the prototype for hich are the context is make back.
	 *
	 * @return (GenPrototype*) the prototype
	 */
	inline GenPrototype* getPrototype(void)const {return m_prototype;}

	/**
	 * [inline]
	 * This function add a Gen to the Context.
	 *
	 * @param gen (Gen*) the new Gen, which should be added
	 */
	inline void addGen(Gen* gen) {m_storage.push_back(gen);}

	/**
	 * [inline]
	 * This function removes one gen which is saved inside the context (but NO deleting of the gen!!!).
	 *
	 * param gen (Gen*) The gen, which should be removed
	 */
	inline void removeGen(Gen* gen) {std::vector<Gen*>::iterator itr = std::find(m_storage.begin(),m_storage.end(),gen); m_storage.erase(itr);}

	/**
	 * [inline], [const]
	 * This function gives all gens which are saved in this context back.
	 *
	 * @return (vector<Gen*>&) list with all gens.
	 */
	inline const std::vector<Gen*>& getGene(void)const {return m_storage;}

	/**
	 * This function makes an update on the statistical values
	 * @param factor (double) this factor is normal 1.5 and is for the whisker distance in the analysation
	 */
	void update(double factor=1.5);

	/**
	 * restore all GenContext
	 * @return (bool) if all ok
	 */
	static bool restore();

protected:
	/**
	 * (vector<Gen*>
	 * Storage for all Genes which are saved in this context.
	 */
	std::vector<Gen*> m_storage;

	/**
	 * (GenPrototyp*)
	 * the prototype for which are the context is.
	 */
	GenPrototype* m_prototype;

	/**
	 * the min value of the gens
	 */
	double m_min;

	/**
	 * the under whisker of the gens
	 */
	double m_w1;

	/**
	 * the under quartil of the gens
	 */
	double m_q1;

	/**
	 * the median of the gens
	 */
	double m_med;

	/**
	 * the average of the gens
	 */
	double m_avg;

	/**
	 * the upper quartil of the gens
	 */
	double m_q3;

	/**
	 * the upper whisker of the gens
	 */
	double m_w3;

	/**
	 * 
	 * the max value of the gens
	 */
	double m_max;

	/*int m_numExtream;
	std::vector<double> m_extream;*/

private:
	/**
	 * disable default constructor
	 */
	GenContext();
};

#endif /* GENCONTEXT_H_ */
