/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.5  2009-08-10 07:39:54  guettler
 *   implements new Callbackable interface
 *
 *   Revision 1.4  2009/07/21 08:47:33  robot12
 *   add some comments
 *
 *   Revision 1.3  2009/06/29 12:43:37  robot12
 *   3 new functions to work whit the new part data analysation.
 *
 *   Revision 1.2  2009/05/19 11:39:45  robot12
 *   add to statistictools some template analysation methodes (needed for boxplots)
 *
 *   Revision 1.1  2009/03/27 06:16:57  guettler
 *   support for gcc 4.3 compatibility (has to be checked), StatisticTools moves from utils to statistictools
 *
 *   Revision 1.10  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.9  2008/04/29 09:56:21  guettler
 *   -debug printouts removed
 *
 *   Revision 1.8  2008/04/29 08:51:54  guettler
 *   -cosmetic changes of StatisticTools
 *   -StatisticTools now uses new function addInspectableValue of the
 *   interface Inspectable, not overloading getInternalParams and
 *   getInternalParamNames anymore
 *
 *   Revision 1.7  2008/01/17 09:59:27  der
 *   complexmeasure: preparations made for predictive information,
 *   fixed a minor bug
 *   statisticmeasure, statistictools: added support for adding
 *   std::list<AbstractMeasure*> to StatisticTools, some minor
 *   improvements
 *
 *   Revision 1.6  2008/01/14 09:09:23  der
 *   added stepSize. An abstractmeasure can now be calculated every stepSize
 *   steps.
 *
 *   Revision 1.5  2007/12/06 10:18:10  der
 *   AbstractMeasure is now a abstract type for Measures,
 *   StatisticTools now supports AbstractMeasures,
 *   StatisticalMeasure, ComplexMeasure  now derived from
 *   AbstractMeasure,
 *   ComplexMeasure provides support for calculation e.g. entropy,
 *   uses Discretisizer,
 *   Discretisizer is a stand-alone class for support of discretisizing values
 *   TrackableMeasure derived from ComplexMeasure and provides support for calculating complex measures for Trackable objects
 *
 *   Revision 1.4  2007/09/28 09:15:25  robot3
 *   extended comments
 *
 *   Revision 1.3  2007/09/28 08:48:21  robot3
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
  addInspectableValue(newMeasure->getName(),&newMeasure->getValueAddress());
  return newMeasure;
}

double& StatisticTools::addMeasure(AbstractMeasure* measure) {
  this->activeMeasures.push_back(measure);
  addInspectableValue(measure->getName(),&measure->getValueAddress());
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
type StatisticTools::getAnalisation(DOUBLE_ANALYSATION_CONTEXT* tvAnalysation,	AnalysationMode mode, unsigned int feature) {
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
