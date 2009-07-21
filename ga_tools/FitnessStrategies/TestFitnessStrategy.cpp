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
 *   This is a test implementation for IFitnessStrategy. It calculate      *
 *   from the first two double gens the function                           *
 *   10*(x²+2.5*y²-y)exp(1-(x²+y²)) + 2.4 + 0.1x² + 0.1y²                  *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.1  2009/06/15 13:58:37  robot12
 *   3 new fitness strategys and IFitnessStrategy and SumFitnessStragegy with comments.
 *
 *
 ***************************************************************************/

#include "TestFitnessStrategy.h"

#include "Individual.h"
#include "Gen.h"
#include "IValue.h"
#include "TemplateValue.h"
#include "IFitnessStrategy.h"

#include <math.h>

#define STANDART_FACTOR_FOR_UNKNOWN_DATA_TYP 1.0

TestFitnessStrategy::TestFitnessStrategy() {
	// nothing
}

TestFitnessStrategy::~TestFitnessStrategy() {
	// nothing
}

double TestFitnessStrategy::getFitness(const Individual* individual) {
	double x;
	double y;
	double f;						//the result
	Gen* gen;						//the actual gen
	IValue* value;					//the value of the gen
	TemplateValue<double>* tValue;	//the casted value

	gen = individual->getGen(0);	//become the first gen from the individual
	value = gen->getValue();		//become the value from the gen
	tValue = dynamic_cast<TemplateValue<double>* >(value);	//cast it
	if(tValue == 0) { //UNKNOWN DATA TYP		//test if it really a double gen
		x = STANDART_FACTOR_FOR_UNKNOWN_DATA_TYP;
	}
	else {
		x = tValue->getValue();
	}

	// the same for the second gen
	gen = individual->getGen(1);
	value = gen->getValue();
	tValue = dynamic_cast<TemplateValue<double>* >(value);
	if(tValue == 0) { //UNKNOWN DATA TYP
		y = STANDART_FACTOR_FOR_UNKNOWN_DATA_TYP;
	}
	else {
		y = tValue->getValue();
	}

	//for debuging
	/*double xq = x*x;
	double yq = y*y;
	double xq01 = 0.1*xq;
	double yq01 = 0.1*yq;
	double y25 = 2.5*yq;
	double xq_y25 = xq+y25;
	double xq_y25_y = xq_y25 - y;
	double xq_yq = xq+yq;
	double ein_xq_yq = 1-xq_yq;
	double exp_ein_xq_yq = exp(ein_xq_yq);
	double xq_y25_y_exp_ein_xq_yq = xq_y25_y*exp_ein_xq_yq;
	xq_y25_y_exp_ein_xq_yq *= 10.0;
	xq_y25_y_exp_ein_xq_yq += 2.4;
	xq_y25_y_exp_ein_xq_yq += xq01;
	xq_y25_y_exp_ein_xq_yq += yq01;

	f=xq_y25_y_exp_ein_xq_yq;*/

	//calculate the function
	f = (((x*x) + (2.5*y*y) - y) * exp(1 - ((x*x) + (y*y))) * 10.0) + 2.4 + (0.1*x*x) + (0.1*y*y);

	//return the result.
	return f;
}
