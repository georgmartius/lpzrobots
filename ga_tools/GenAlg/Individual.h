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
 *   Revision 1.4  2009-04-27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

#include "types.h"

#include <vector>
#include <string>

#include "Gen.h"
#include "IFitnessStrategie.h"
#include "Generation.h"

class Individual {
public:
	Individual(std::string name, int id);
	virtual ~Individual();

	inline int getID(void)const {return m_ID;}
	inline std::string getName(void)const {return m_name;}

	inline int getSize(void)const {return m_gene.size();}
	inline Gen* getGen(int x)const {if(x<getSize())return m_gene[x];return NULL;}
	inline void addGen(Gen* gen) {m_gene.push_back(gen);}
	inline void removeGen(Gen* gen) {
		std::vector<Gen*>::iterator itr = find(m_gene.begin(),m_gene.end(),gen);
		m_gene.erase(itr);
	}
	inline void removeGen(int x) {if(x<getSize())m_gene.erase(m_gene.begin()+x);}

	inline double getFitness(void) {if(!m_fitnessIsCalculated && m_fitnessStrategie!=0){m_fitness = m_fitnessStrategie->getFitness(m_gene);m_fitnessIsCalculated=true;}return m_fitness;}
	inline static void setFitnessStrategie(IFitnessStrategie* strategie) {m_fitnessStrategie = strategie;}

protected:
	std::string m_name;
	int m_ID;
	std::vector<Gen*> m_gene;
	static IFitnessStrategie* m_fitnessStrategie = 0;
	double m_fitness;

private:
	/**
	 * disable the default constructor
	 */
	Individual();
};

#endif /* INDIVIDUAL_H_ */
