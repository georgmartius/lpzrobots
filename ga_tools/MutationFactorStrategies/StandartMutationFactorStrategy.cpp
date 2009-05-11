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
 *   Revision 1.3  2009-05-11 14:08:51  robot12
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

#include <selforg/randomgenerator.h>
#include <stdlib.h>
#include <math.h>

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
	double sum = 0.0;
	double durch;
	double result;
	int num = gene.size();
	int x;
	IValue* iValue;
	TemplateValue<double>* tValue;
	RandGen random;
	int rand = ((int)random.rand())%2;

	static TemplateValue<double> storage(0.0);

	for(x=0;x<num;x++) {
		iValue = gene[x]->getValue();
		tValue = dynamic_cast<TemplateValue<double>* >(iValue);
		if(tValue!=0) { // KNOWN DATA TYP
			sum += tValue->getValue();
		}
	}
	durch = sum / (double)num;

	sum = 0.0;

	for(x=0;x<num;x++) {
		iValue = gene[x]->getValue();
		tValue = dynamic_cast<TemplateValue<double>* >(iValue);
		if(tValue!=0) { // KNOWN DATA TYP
			sum += (tValue->getValue() - durch) * (tValue->getValue() - durch);
		}
	}
	result = sqrt(sum / (double)(num-1));

	if(rand==0)
		result*=-1.0;

	storage.setValue(result);

	return &storage;
}
