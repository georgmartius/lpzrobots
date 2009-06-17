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
 *   This class is a implementation of the IMutationFactorStrategy         *
 *   interface. It returns a value which is calculate by the variance of   *
 *   the gens in the given set.                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2009-06-17 11:11:06  robot12
 *   finishing the mutationfactorstrategy and add some comments.
 *
 *   Revision 1.3  2009/05/11 14:08:51  robot12
 *   patch some bugfix....
 *
 *   Revision 1.2  2009/05/06 13:28:23  robot12
 *   some implements... Finish
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/05/04 09:20:52  robot12
 *   some implements.. Finish --> first compile
 *
 *   Revision 1.1  2009/04/29 14:32:28  robot12
 *   some implements... Part4
 *
 *
 *
 ***************************************************************************/

#include "StandartMutationFactorStrategy.h"

//includes
#include <selforg/randomgenerator.h>
#include <stdlib.h>
#include <math.h>

//ga_tools includes
#include "Gen.h"
#include "IValue.h"
#include "TemplateValue.h"

StandartMutationFactorStrategy::StandartMutationFactorStrategy() {
	// nothing
}

StandartMutationFactorStrategy::~StandartMutationFactorStrategy() {
	// nothing
}

IValue* StandartMutationFactorStrategy::calcMutationFactor(const std::vector<Gen*>& gene) {
	double sum = 0.0;								//this would be the sum of the gens
	double durch;									//this would be the average of the gens
	double result;									//this would be the result of the function
	int num = gene.size();							//the number of gens in the set
	int x;											//help variable
	IValue* iValue;									//the value from the actual gen
	TemplateValue<double>* tValue;					//the casted value from the actual gen
	RandGen random;									//a random generator
	int rand = ((int)(random.rand()*10000))%2;		//a random value (zero or one)

	static TemplateValue<double> storage(0.0);		//a storage for casted values.

	for(x=0;x<num;x++) {
		iValue = gene[x]->getValue();				//become a value from a gen
		tValue = dynamic_cast<TemplateValue<double>* >(iValue);	//caste the value
		if(tValue!=0) { // KNOWN DATA TYP			//if it is a double value add it to sum
			sum += tValue->getValue();
		}
	}
	durch = sum / (double)num;						//the average is the sum divided by the number of gens.

	sum = 0.0;										//reset sum

	for(x=0;x<num;x++) {							//now calculate the varianz = sqrt(sum((xi - ^xi)²) / n-1)
		iValue = gene[x]->getValue();
		tValue = dynamic_cast<TemplateValue<double>* >(iValue);
		if(tValue!=0) { // KNOWN DATA TYP
			sum += (tValue->getValue() - durch) * (tValue->getValue() - durch);
		}
	}
	result = sqrt(sum / (double)(num-1));

	if(rand==0)										//if the random value zero than mult -1 to the result.
		result*=-1.0;

	storage.setValue(result);						//take the result in the storage and give the storage back.

	return &storage;
}
