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
 *   Revision 1.5  2009-05-11 14:08:51  robot12
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

#include <vector>
#include <string>

#include "Gen.h"
#include "SingletonGenEngine.h"

class Individual {
public:
	Individual(std::string name, int id, Individual* p1=0, Individual* p2=0);
	virtual ~Individual();

	inline int getID(void)const {return m_ID;}
	inline std::string getName(void)const {return m_name;}

	inline int getSize(void)const {return m_gene.size();}
	inline Gen* getGen(int x)const {if(x<getSize())return m_gene[x];return NULL;}
	inline void addGen(Gen* gen) {m_gene.push_back(gen);}
	inline const std::vector<Gen*>& getGene(void)const {return m_gene;}

	void removeGen(Gen* gen);
	void removeGen(int x);
	double getFitness()const;

	inline void setMutated(void) {m_mutated=true;}
	inline const Individual* getParent1(void)const {return m_parent1;}
	inline const Individual* getParent2(void)const {return m_parent2;}
	inline bool isMutated(void)const {return m_mutated;}

protected:
	std::string m_name;
	int m_ID;
	std::vector<Gen*> m_gene;
	Individual* m_parent1;
	Individual* m_parent2;
	bool m_mutated;

private:
	/**
	 * disable the default constructor
	 */
	Individual();
};

#endif /* INDIVIDUAL_H_ */
