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

#include "SingletonGenFactory.h"

//ga_tools includes
#include "Gen.h"
#include "GenContext.h"
#include "GenPrototype.h"
#include "Individual.h"
#include "IValue.h"
#include "SingletonGenEngine.h"

int SingletonGenFactory::m_number = 0;
SingletonGenFactory* SingletonGenFactory::m_factory = 0;

SingletonGenFactory::SingletonGenFactory() {
        // nothing
}

SingletonGenFactory::~SingletonGenFactory() {
        // nothing
}

Gen* SingletonGenFactory::createGen(GenContext* context, Individual* individual, GenPrototype* prototype)const {
        Gen* gen = new Gen(prototype, m_number);
        m_number++;
        IValue* value = prototype->getRandomValue();                //create a new random value

        gen->setValue(value);

        context->addGen(gen);                                                                //save gen
        individual->addGen(gen);
        SingletonGenEngine::getInstance()->addGen(gen);

        return gen;
}

Gen* SingletonGenFactory::createGen(GenContext* context, Individual* individual, GenPrototype* prototype, GenContext* oldContext, Individual* oldIndividual, Gen* oldGen, bool mutate)const {
        if(mutate) {
                return prototype->mutate(context, individual, oldGen, oldContext);                        //mutate
        }


        Gen* gen = new Gen(prototype,m_number);                                                                                        //copy gen

        m_number++;

        IValue* value = oldGen->getValue();

        gen->setValue(value);

        context->addGen(gen);
        individual->addGen(gen);
        SingletonGenEngine::getInstance()->addGen(gen);

        return gen;
}

Gen* SingletonGenFactory::createGen(GenContext* context, Individual* individual, GenPrototype* prototype, IValue* value) {
        Gen* gen = new Gen(prototype,m_number);

        m_number++;

        gen->setValue(value);

        context->addGen(gen);
        individual->addGen(gen);
        SingletonGenEngine::getInstance()->addGen(gen);

        return gen;
}
