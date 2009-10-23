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
 *   This class is used for create a context for some gens. This mean it   *
 *   saves all gens which have the same prototype and are a part of an     *
 *   individual which are in ONE generation. It can be useful for some     *
 *   statistical calculation or for optimizing the mutation factor.        *
 *                                                                         *
 *   The Gen Context is inside the gen. alg. only saved in the             *
 *                                                                         *
 *   $Log$
 *   Revision 1.7  2009-10-23 10:47:45  robot12
 *   bugfix in store and restore
 *
 *   Revision 1.6  2009/10/21 14:08:06  robot12
 *   add restore and store functions to the ga package
 *
 *   Revision 1.5  2009/07/28 09:12:47  robot12
 *   add a clean up to update (delete context)
 *
 *   Revision 1.4  2009/07/21 08:37:58  robot12
 *   add some comments
 *
 *   Revision 1.3  2009/06/29 14:32:51  robot12
 *   finishing the GenContext and add some comments
 *
 *   Revision 1.2  2009/05/07 14:47:47  robot12
 *   some comments
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/04/30 11:35:54  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.1  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "GenContext.h"
#include <selforg/statistictools.h>
#include "TemplateValue.h"
#include "Gen.h"
#include "GenPrototype.h"
#include "SingletonGenEngine.h"
#include "Generation.h"
#include "Individual.h"

GenContext::GenContext() {
	// nothing
}

GenContext::GenContext(GenPrototype* prototype) {
	m_prototype = prototype;

	std::string name = prototype->getName();

	//add some variable to the inspectables
	addInspectableValue(name+"MIN",&m_min);
	addInspectableValue(name+"W1",&m_w1);
	addInspectableValue(name+"Q1",&m_q1);
	addInspectableValue(name+"MED",&m_med);
	addInspectableValue(name+"AVG",&m_avg);
	addInspectableValue(name+"Q3",&m_q3);
	addInspectableValue(name+"W3",&m_w3);
	addInspectableValue(name+"MAX",&m_max);
}

GenContext::~GenContext() {
	m_storage.clear();
}

void GenContext::update(double factor) {
	std::vector<double> list;
	TemplateValue<double>* tValue;

	for(std::vector<Gen*>::const_iterator iter=m_storage.begin();iter!=m_storage.end();iter++) {
		tValue = dynamic_cast<TemplateValue<double>*>((*iter)->getValue());
		if(tValue!=0)
			list.push_back(tValue->getValue());
	}

	DOUBLE_ANALYSATION_CONTEXT* context = new DOUBLE_ANALYSATION_CONTEXT(list);

	m_q1 = context->getQuartil1();
	m_q3 = context->getQuartil3();
	m_med = context->getMedian();
	m_avg = context->getAvg();
	m_w1 = context->getWhisker1(factor);
	m_w3 = context->getWhisker3(factor);
	m_min = context->getMin();
	m_max = context->getMax();

	delete context;
}

bool GenContext::restore() {
  int numGeneration = SingletonGenEngine::getInstance()->getActualGenerationNumber();
  int x,y,z,v;
  const std::vector<GenPrototype*>& prototypeSet = SingletonGenEngine::getInstance()->getSetOfGenPrototyps();
  int numPrototypes = prototypeSet.size();
  int numIndividuals;
  Generation* generation;
  GenPrototype* prototype;
  GenContext* context;
  Gen* gen;
  Individual* individual;

  generation = SingletonGenEngine::getInstance()->getGeneration(0);
  numIndividuals = generation->getCurrentSize();
  for(y=0;y<numPrototypes;y++) {
    prototype = prototypeSet[y];
    context = new GenContext(prototype);
    for(z=0;z<numIndividuals;z++) {
      individual = generation->getIndividual(z);
      for(v=0;v<numPrototypes;v++) {
        gen = individual->getGen(v);
        if(gen->getPrototype()==prototype)
          break;
      }
      context->addGen(gen);
    }
    context->update();
    prototype->insertContext(generation,context);
  }

  for(x=0;x<numGeneration;x++) {
    generation = SingletonGenEngine::getInstance()->getGeneration(x+1);
    numIndividuals = generation->getCurrentSize();
    for(y=0;y<numPrototypes;y++) {
      prototype = prototypeSet[y];
      context = new GenContext(prototype);
      for(z=0;z<numIndividuals;z++) {
        individual = generation->getIndividual(z);
        for(v=0;v<numPrototypes;v++) {
          gen = individual->getGen(v);
          if(gen->getPrototype()==prototype)
            break;
        }
        context->addGen(gen);
      }
      context->update();
      prototype->insertContext(generation,context);
    }
  }

  return true;
}
