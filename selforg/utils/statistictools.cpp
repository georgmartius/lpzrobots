/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2007-09-28 08:48:21  robot3
 *   corrected some minor bugs, files are still in develop status
 *
 *   Revision 1.2  2007/05/08 10:18:15  der
 *   added a function for starting the measure after a given time.
 *   made some tests
 *
 *   Revision 1.1  2007/05/07 21:01:32  robot3
 *   statistictools is a class for easy visualization of measurements of observed values
 *   it is possible to add the observed value itself with mode ID
 *
 *                                                                         *
 ***************************************************************************/
#include "statistictools.h"
#include "statisticmeasure.h"


void StatisticTools::doOnCallBack() {
	// update all statistic measures
	if (beginMeasureCounter>0)
		beginMeasureCounter--;
	else for (std::list<StatisticMeasure*>::iterator i=activeMeasures.begin();i!=activeMeasures.end();i++) {
		(*i)->step();
	}
}

double& StatisticTools::addMeasure(double& observedValue, char* measureName, MeasureMode mode, long stepSpan, double additionalParam) {
  return this->getMeasure(observedValue,measureName,mode,stepSpan,additionalParam)->getValueAdress();
}

StatisticMeasure* StatisticTools::getMeasure(double& observedValue, char* measureName, MeasureMode mode, long stepSpan, double additionalParam) {
  StatisticMeasure* newMeasure = new StatisticMeasure(observedValue, measureName, mode, stepSpan, additionalParam);
  this->activeMeasures.push_back(newMeasure);
  return newMeasure;
}

void StatisticTools::beginMeasureAt(long step) {
	this->beginMeasureCounter=step;
}

std::list<Inspectable::iparamkey> StatisticTools::getInternalParamNames() const  {
	std::list<Inspectable::iparamkey> list;
	for (std::list<StatisticMeasure*>::const_iterator i=activeMeasures.begin();i!=activeMeasures.end();i++) {
		list+=(*i)->getName();
	}
	return list;
}

std::list<Inspectable::iparamval> StatisticTools::getInternalParams() const {
	std::list<Inspectable::iparamval> list;
	for (std::list<StatisticMeasure*>::const_iterator i=activeMeasures.begin();i!=activeMeasures.end();i++) {
		list+=(*i)->getValue();
	}
	return list;
}
