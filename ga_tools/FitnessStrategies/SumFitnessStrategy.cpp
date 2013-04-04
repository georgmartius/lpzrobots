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

#include "SumFitnessStrategy.h"

#include "Individual.h"
#include "Gen.h"
#include "IValue.h"
#include "TemplateValue.h"
#include "IFitnessStrategy.h"

// if it isn't a double gen add one to the sum.
#define STANDART_FACTOR_FOR_UNKNOWN_DATA_TYP 1.0

SumFitnessStrategy::SumFitnessStrategy() {
        // nothing
}

SumFitnessStrategy::~SumFitnessStrategy() {
        // nothing
}

double SumFitnessStrategy::getFitness(const Individual* individual) {
        double sum = 0.0;                                                //the sum and on the end the resulting fitness value
        int num = individual->getSize();                //number of gens inside the individual
        Gen* gen;                                                                //the actual gen
        IValue* value;                                                        //the value of the gen
        TemplateValue<double>* tValue;                        //the casted value of the gen

        for(int x=0; x<num; x++) {
                gen = individual->getGen(x);                //become the gen from the individual
                value = gen->getValue();                        //become the value of the gen
                tValue = dynamic_cast<TemplateValue<double>* >(value);        //cast the value to double gen
                if(tValue == 0) { //UNKNOWN DATA TYP        //test the value if it is really a double gen
                        sum += STANDART_FACTOR_FOR_UNKNOWN_DATA_TYP;
                }
                else {
                        sum += tValue->getValue();                // add the value
                }
        }

        return sum;                //return the result
}
