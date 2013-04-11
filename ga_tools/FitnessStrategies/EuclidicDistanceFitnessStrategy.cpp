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

#include "EuclidicDistanceFitnessStrategy.h"

#include "Individual.h"
#include "Gen.h"
#include "IValue.h"
#include "TemplateValue.h"
#include "IFitnessStrategy.h"

#include <math.h>

// if the gen not a double gen so it is calculated with zero
#define STANDART_FACTOR_FOR_UNKNOWN_DATA_TYP 0.0

EuclidicDistanceFitnessStrategy::EuclidicDistanceFitnessStrategy() {
        // nothing
}

EuclidicDistanceFitnessStrategy::~EuclidicDistanceFitnessStrategy() {
        // nothing
}

double EuclidicDistanceFitnessStrategy::getFitness(const Individual* individual) {
        double sum = 0.0;                                        //the sum and later the result and so the fitness of the individual.
        int num = individual->getSize();        //number of gens inside the individual
        Gen* gen;                                                        //the actual used gen
        IValue* value;                                                //the value of the gen
        TemplateValue<double>* tValue;                //the casted value of the gen (double gen)

        // take all gens
        for(int x=0; x<num; x++) {
                gen = individual->getGen(x);        //become gen from individual
                value = gen->getValue();                //become the value from the gen
                tValue = dynamic_cast<TemplateValue<double>* >(value);        //cast the value to a double gen
                if(tValue == 0) { //UNKNOWN DATA TYP        //test if it is really a double gen
                        sum += STANDART_FACTOR_FOR_UNKNOWN_DATA_TYP;
                }
                else {
                        sum += tValue->getValue() * tValue->getValue();                //euclid = sqrt(a²+b²+c²+...) so calculate first the sum of the ²
                }
        }

        return sqrt(sum);                        // as next calculate the sqrt from the sum an return
}
