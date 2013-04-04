/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
#include "statistictools.h"
#include "statisticmeasure.h"
#include "complexmeasure.h"


void StatisticTools::doOnCallBack(BackCaller* source, BackCaller::CallbackableType type /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE */) {
    // update all statistic measures
    if (beginMeasureCounter>0)
        beginMeasureCounter--;
    else
        for (std::list<AbstractMeasure*>::iterator i=activeMeasures.begin();i!=activeMeasures.end();i++) {
          if (((*i)->getActualStep())%((*i)->getStepSize())==0)
            (*i)->step();
        }
}

double& StatisticTools::addMeasure(double& observedValue, const char* measureName, MeasureMode mode, long stepSpan, double additionalParam) {
    StatisticMeasure* newMeasure = this->getMeasure(observedValue,measureName,mode,stepSpan,additionalParam);
    return  newMeasure->getValueAddress();
}

StatisticMeasure* StatisticTools::getMeasure(double& observedValue, const char* measureName, MeasureMode mode, long stepSpan, double additionalParam) {
    StatisticMeasure* newMeasure = new StatisticMeasure(observedValue, measureName, mode, stepSpan, additionalParam);
    this->activeMeasures.push_back(newMeasure);
          addInspectableValue(newMeasure->getName(),&newMeasure->getValueAddress(), "measure registered at "+ getNameOfInspectable());
  return newMeasure;
}

AbstractMeasure* StatisticTools::getMeasure(const std::string& measureName) const{
  FOREACHC(std::list<AbstractMeasure*>,activeMeasures, m){
    if((*m)->getName()==measureName){
      return (*m);
    }
  }
  return 0;
}

double& StatisticTools::addMeasure(AbstractMeasure* measure) {
  this->activeMeasures.push_back(measure);
  addInspectableValue(measure->getName(),&measure->getValueAddress(), "measure registered at "+ getNameOfInspectable());
  return  measure->getValueAddress();
}

double& StatisticTools::addMeasureList(std::list<AbstractMeasure*> measureList) {
  FOREACH(std::list<AbstractMeasure*>,measureList,measure) {
    addMeasure(*measure);
  }
  return measureList.front()->getValueAddress();
}

double& StatisticTools::addMeasureList(std::list<ComplexMeasure*> measureList) {
  FOREACH(std::list<ComplexMeasure*>,measureList,measure) {
    addMeasure(*measure);
  }
  return measureList.front()->getValueAddress();
}

double& StatisticTools::addMeasureList(std::list<StatisticMeasure*> measureList) {
  FOREACH(std::list<StatisticMeasure*>,measureList,measure) {
    addMeasure(*measure);
  }
  return measureList.front()->getValueAddress();
}



void StatisticTools::beginMeasureAt(long step) {
    this->beginMeasureCounter=step;
}



/*template
<class type,
type zero(void),
bool lower(const type&, const type&),
bool higher(const type&, const type&),
type doubleDiv(const type&, const double&),
type doubleMul(const type&, const double&),
type add(const type&, const type&),
type sub(const type&, const type&),
type mul(const type&, const type&),
type div(const type&, const type&)>
DOUBLE_ANALYSATION_CONTEXT* StatisticTools::getAnalysation(std::vector<type> values) {
        return new DOUBLE_ANALYSATION_CONTEXT(values);
}*/

/*template
<class type,
type zero(void),
bool lower(const type&, const type&),
bool higher(const type&, const type&),
type doubleDiv(const type&, const double&),
type doubleMul(const type&, const double&),
type add(const type&, const type&),
type sub(const type&, const type&),
type mul(const type&, const type&),
type div(const type&, const type&)>
type StatisticTools::getAnalisation(DOUBLE_ANALYSATION_CONTEXT* tvAnalysation,        AnalysationMode mode, unsigned int feature) {
        switch(mode){
        case AM_AVG:
                return tvAnalysation->getAvg();
        case AM_MIN:
                return tvAnalysation->getMin();
        case AM_MAX:
                return tvAnalysation->getMax();
        case AM_RANGE:
                return tvAnalysation->getRange();
        case AM_IQR:
                return tvAnalysation->getIQR();
        case AM_MED:
                return tvAnalysation->getMedian();
        case AM_WHISKER:
                return tvAnalysation->getWhisker(1.5);
        case AM_Q1:
                return tvAnalysation->getQuartil1();
        case AM_Q3:
                return tvAnalysation->getQuartil3();
        case AM_W1:
                return tvAnalysation->getWhisker1(1.5);
        case AM_W3:
                return tvAnalysation->getWhisker3(1.5);
        case AM_NUM_EXT:
                return (type)tvAnalysation->getNumExtrems(1.5);
        case AM_EXT:
                return tvAnalysation->getExtrem(1.5,feature);
        }
}*/

/*template
<class type,
type zero(void),
bool lower(const type&, const type&),
bool higher(const type&, const type&),
type doubleDiv(const type&, const double&),
type doubleMul(const type&, const double&),
type add(const type&, const type&),
type sub(const type&, const type&),
type mul(const type&, const type&),
type div(const type&, const type&)>
type StatisticTools::getAnalisation(std::vector<type> values, AnalysationMode mode, unsigned int feature) {
        DOUBLE_ANALYSATION_CONTEXT* context = GET_TYPE_ANALYSATION(type)(values);
        return GET_TYPE_ANALYSATION(type)(context,mode,feature);
}*/
