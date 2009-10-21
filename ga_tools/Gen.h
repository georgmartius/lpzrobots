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
 *   This class is used for representing one gen in the gen. alg.          *
 *   It has one ID which make it individual and a name (string)            *
 *   which group it with other gens to a gen pool.                         *
 *   Also it has a IValue which is used to save the real value.            *
 *   An IValue can be a number, a matrix, a 3D Modell or something else.   *
 *                                                                         *
 *   Places for saving the gen inside the gen. alg. are the GenContext,    *
 *   the Individual and the GenEngine. Deleting only in the GenEngine!     *
 *                                                                         *
 *   $Log$
 *   Revision 1.8  2009-10-21 14:08:06  robot12
 *   add restore and store functions to the ga package
 *
 *   Revision 1.7  2009/07/21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.6  2009/05/12 13:29:25  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.5  2009/05/07 14:47:47  robot12
 *   some comments
 *
 *   Revision 1.4  2009/05/04 15:27:56  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.6  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.5  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef GEN_H_
#define GEN_H_

// standard includes
#include <string>

// forward declarations

// gen. alg. includes
#include "IValue.h"
#include "GenPrototype.h"

/**
 * The Gen class.
 *
 *   This class is used for representing one gen in the gen. alg.
 *   It has one ID which make it individual and an name (string)
 *   which group it with other gens to a gen pool.
 *   Also it has a IValue which is used to save the real value.
 *   An IValue can be a number, a matrix, a 3D Modell or something else.
 *
 *   Places for saving the gen inside the gen. alg. are the GenContext,
 *   the Individual and the GenEngine. Deleting only in the GenEngine!
 */
class Gen {
public:
	/**
	 * constructor to create a gen. Information which the class need are
	 * the prototype (name an group of gens) and the id, which the gen
	 * identified.
	 *
	 * @param prototype (GenPrototype*) Pointer to the prototype.
	 * @param id (int) ID of the gen
	 */
	Gen(GenPrototype* prototype, int id);

	/**
	 * destructor to delete a gen.
	 */
	virtual ~Gen(void);

	/**
	 * [const]
	 * This function gives the Name of this Gen (name of the prototype) back.
	 *
	 * @return (string) Name of the Gen.
	 */
	std::string getName(void)const;

	/**
	 * [inline], [const]
	 * This function gives the value which is saved in the Gen back.
	 *
	 * @return (IValue*) The value
	 */
	inline IValue* getValue(void)const {return m_value;}

	/**
	 * [inline]
	 * This function change the saved pointer to the IValue. So the Gen changed his value.
	 *
	 * @param value (IVaue*) the new Value
	 */
	inline void setValue(IValue* value) {m_value=value;}

	/**
	 * [inline], [const]
	 * This function gives the ID of the Gen back.
	 *
	 * @return (int) The ID
	 */
	inline int getID(void)const {return m_ID;}

	/**
	 * [const]
	 * This function gives the prototype of the Gen back.
	 *
	 * @return (GenPrototyp*) The prototype
	 */
	GenPrototype* getPrototype(void)const;

	/**
	 *  [const]
	 *  This function returns a string representation of this Gen.
	 *
	 *  @return (string) The Gen in string - Form
	 */
	std::string toString(bool onlyValue = true)const;

	/**
	 * store the gene in a file
	 * @param f (FILE*) the file to store
	 * @return (bool) true if all ok
	 */
	bool store(FILE* f)const;

protected:
	/**
	 * (IValue*)
	 * The value of the Gen.
	 */
	IValue* m_value;

	/**
	 * (GenPrototyp*)
	 * The prototype of the Gen. After creating unchangeable.
	 */
	GenPrototype* m_prototype;

	/**
	 * (int)
	 * The ID of the Gen. The ID is individual. Every Gen has his own.
	 */
	int m_ID;

private:
	/**
	 * disable default constructor
	 */
	Gen(void);
};

#endif /* GEN_H_ */
