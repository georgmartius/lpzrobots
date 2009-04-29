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
 *   Revision 1.1  2009-04-29 14:32:28  robot12
 *   some implements... Part4
 *
 *
 *
 ***************************************************************************/

#include "StandartMutationFactorStrategie.h"
#include <selforg/randomgenerator.h>

StandartMutationFactorStrategie::StandartMutationFactorStrategie() {
	// nothing
}

StandartMutationFactorStrategie::~StandartMutationFactorStrategie() {
	// nothing
}

IValue* StandartMutationFactorStrategie::calcMutationFactor(const std::vector<Gen*>& gene) {
	IValue* sum = 0;
	IValue* oldSum = 0;
	IValue* absSum = 0;
	IValue* durch = 0;
	int num = gene.size();
	int x;
	RandGen random;
	int rand = ((int)random->rand())%2;

	sum = gene[0]->getValue();
	for(x=0;x<num-1;x++) {
		sum = (*sum) + (*gene[x+1]->getValue());
		if(oldSum != 0){delete oldSum; oldSum=0;}
		oldSum = sum;
	}
	oldSum = 0;
	durch = (*sum) / (double)num;
	if(sum!=0){delete sum; sum=0;}

	sum = (*gene[0]->getValue()) - (*durch);
	absSum = sum->abs();
	for(x=0;x<num-1;x++) {
		oldSum = sum;
		sum = (*gene[x+1]->getValue()) - (*durch);
		absSum = sum->abs();
		if(sum!=0){delete sum; sum=0;}
		sum = (*oldSum) + (*absSum);
		if(oldSum!=0){delete oldSum; oldSum=0;}
		if(absSum!=0){delete absSum; absSum=0;}
	}
	oldSum = 0;
	absSum = (*sum) / (double)num;
	if(sum!=0){delete sum; sum=0;}
	if(durch!=0){delete durch; durch=0;}

	return absSum;
}
