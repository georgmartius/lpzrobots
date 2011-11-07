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

#include "GenContext.h"
#include <selforg/statistictools.h>
#include "TemplateValue.h"
#include "Gen.h"
#include "GenPrototype.h"
#include "SingletonGenEngine.h"
#include "Generation.h"
#include "Individual.h"

GenContext::GenContext() :
  Inspectable("GenContext") {
  // nothing
}

GenContext::GenContext(GenPrototype* prototype) :
  Inspectable(prototype->getName()) {
  m_prototype = prototype;

  std::string name = prototype->getName();

  //add some variable to the inspectables
  addInspectableValue(name + "MIN", &m_min);
  addInspectableValue(name + "W1", &m_w1);
  addInspectableValue(name + "Q1", &m_q1);
  addInspectableValue(name + "MED", &m_med);
  addInspectableValue(name + "AVG", &m_avg);
  addInspectableValue(name + "Q3", &m_q3);
  addInspectableValue(name + "W3", &m_w3);
  addInspectableValue(name + "MAX", &m_max);
}

GenContext::~GenContext() {
  m_storage.clear();
}

void GenContext::update(double factor) {
  std::vector<double> list;
  TemplateValue<double>* tValue;

  for (std::vector<Gen*>::const_iterator iter = m_storage.begin(); iter != m_storage.end(); iter++) {
    tValue = dynamic_cast<TemplateValue<double>*> ((*iter)->getValue());
    if (tValue != 0)
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
  int x, y, z, v;
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
  for (y = 0; y < numPrototypes; y++) {
    prototype = prototypeSet[y];
    context = new GenContext(prototype);
    for (z = 0; z < numIndividuals; z++) {
      individual = generation->getIndividual(z);
      for (v = 0; v < numPrototypes; v++) {
        gen = individual->getGen(v);
        if (gen->getPrototype() == prototype)
          break;
      }
      context->addGen(gen);
    }
    context->update();
    prototype->insertContext(generation, context);
  }

  for (x = 0; x < numGeneration; x++) {
    generation = SingletonGenEngine::getInstance()->getGeneration(x + 1);
    numIndividuals = generation->getCurrentSize();
    for (y = 0; y < numPrototypes; y++) {
      prototype = prototypeSet[y];
      context = new GenContext(prototype);
      for (z = 0; z < numIndividuals; z++) {
        individual = generation->getIndividual(z);
        for (v = 0; v < numPrototypes; v++) {
          gen = individual->getGen(v);
          if (gen->getPrototype() == prototype)
            break;
        }
        context->addGen(gen);
      }
      context->update();
      prototype->insertContext(generation, context);
    }
  }

  return true;
}
