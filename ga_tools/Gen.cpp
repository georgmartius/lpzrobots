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
 *   Revision 1.9  2009-10-23 10:47:45  robot12
 *   bugfix in store and restore
 *
 *   Revision 1.8  2009/10/21 14:08:06  robot12
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
 *   Revision 1.4  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.5  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "Gen.h"
#include "restore.h"

Gen::Gen(void) {
	// nothing
}

Gen::Gen(GenPrototype* prototype, int id) {
	m_prototype = prototype;
	m_value = NULL;
	m_ID = id;
}

Gen::~Gen(void) {
	delete m_value;
	m_value = NULL;
}

std::string Gen::getName(void)const {
	return m_prototype->getName();
}

GenPrototype* Gen::getPrototype(void)const {
	return m_prototype;
}

std::string Gen::toString(bool onlyValue)const {
	std::string result = "";

	if(!onlyValue) {
		char buffer[128];

		sprintf(buffer, "%i", m_ID);

		result += "\"" + getName() + "\",\t" + buffer + ",\t";
	}

	result += (std::string)(*m_value);

	return result;
}

bool Gen::store(FILE* f)const {
  RESTORE_GA_GENE head;

  //test
  if(f==NULL) {
    printf("\n\n\t>>> [ERROR] <<<\nNo File to store GA [gene].\n\t>>> [END] <<<\n\n\n");
    return false;
  }

  head.ID = m_ID;

  fprintf(f,"%i\n%s",(int)m_prototype->getName().length(),m_prototype->getName().c_str());

  for(unsigned int x=0;x<sizeof(RESTORE_GA_GENE);x++) {
    fprintf(f,"%c",head.buffer[x]);
  }

  return m_value->store(f);
}
