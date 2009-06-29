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
 *   Revision 1.8  2009-06-29 14:52:14  robot12
 *   finishing Individual and add some comments
 *
 *   Revision 1.7  2009/05/14 15:29:56  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.6  2009/05/12 13:29:26  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.5  2009/05/11 14:08:52  robot12
 *   patch some bugfix....
 *
 *   Revision 1.4  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
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

#include "Individual.h"

Individual::Individual() {
	// nothing
}

Individual::Individual(std::string name, int id, Individual* p1, Individual* p2) {
	m_name = name;
	m_ID = id;
	m_mutated = false;
	m_parent1 = p1;
	m_parent2 = p2;
}

Individual::~Individual() {
	// nothing
}

double Individual::getFitness()const {
	return SingletonGenEngine::getInstance()->getFitness(this);
}

void Individual::removeGen(Gen* gen) {
	std::vector<Gen*>::iterator itr = find(m_gene.begin(),m_gene.end(),gen);
	m_gene.erase(itr);
}
void Individual::removeGen(int x) {
	if(x<getSize())m_gene.erase(m_gene.begin()+x);
}

std::string Individual::IndividualToString(void)const {
	std::string result = "";

	for(std::vector<Gen*>::const_iterator iter = m_gene.begin();iter!=m_gene.end();iter++) {
		result += "" + (*iter)->toString() + "\t";
	}

	char buffer[128];
	sprintf(buffer,"% .12lf",getFitness());
	result += buffer;

	return result;
}

std::string Individual::RootToString(bool withMutation)const {
	std::string result = "";

	if(withMutation) {
		if(m_mutated) {
			result += "m,\t";
		}
		else {
			result += " ,\t";
		}
	}
	result += "\"" + m_name + "\"";

	if(m_parent1!=0)
		result += ",\t\"" + m_parent1->getName() + "\"";
	if(m_parent2!=0)
		result += ",\t\"" + m_parent2->getName() + "\"";

	return result;
}
