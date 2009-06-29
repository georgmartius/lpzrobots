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
 *   Revision 1.4  2009-06-29 15:24:23  robot12
 *   patch a warning
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
 *   Revision 1.10  2008/04/29 09:56:21  guettler
 *   -debug printouts removed
 *
 *   Revision 1.9  2008/04/29 08:51:54  guettler
 *   -cosmetic changes of StatisticTools
 *   -StatisticTools now uses new function addInspectableValue of the
 *   interface Inspectable, not overloading getInternalParams and
 *   getInternalParamNames anymore
 *
 *   Revision 1.8  2008/01/17 09:59:27  der
 *   complexmeasure: preparations made for predictive information,
 *   fixed a minor bug
 *   statisticmeasure, statistictools: added support for adding
 *   std::list<AbstractMeasure*> to StatisticTools, some minor
 *   improvements
 *
 *   Revision 1.7  2007/12/06 10:18:10  der
 *   AbstractMeasure is now a abstract type for Measures,
 *   StatisticTools now supports AbstractMeasures,
 *   StatisticalMeasure, ComplexMeasure  now derived from
 *   AbstractMeasure,
 *   ComplexMeasure provides support for calculation e.g. entropy,
 *   uses Discretisizer,
 *   Discretisizer is a stand-alone class for support of discretisizing values
 *   TrackableMeasure derived from ComplexMeasure and provides support for calculating complex measures for Trackable objects
 *
 *   Revision 1.6  2007/10/01 13:27:47  robot3
 *   documentation
 *
 *   Revision 1.5  2007/09/28 10:08:49  robot3
 *   fixed memory bugs, statistics are from now on aligned right
 *
 *   Revision 1.4  2007/09/28 08:48:21  robot3
 *   corrected some minor bugs, files are still in develop status
 *
 *   Revision 1.3  2007/09/27 10:49:39  robot3
 *   removed some minor bugs,
 *   added CONVergence test
 *   changed little things for support of the new WSM
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
#ifndef _STATISTIC_TOOLS_H
#define _STATISTIC_TOOLS_H

// begin forward declarations
class AbstractMeasure;
class StatisticMeasure;
class ComplexMeasure;
// end forward declarations

#include "inspectable.h"
#include "callbackable.h"
#include "measuremodes.h"
#include "analysationmodes.h"
#include "templatevalueanalysation.h"

#define GET_TYPE_ANALYSATION(type) getAnalysation<type,defaultZero,defaultLower<type>,defaultHigher<type>,defaultDoubleDiv<type>,defaultDoubleMul<type>,defaultAdd<type>,defaultSub<type>,defaultMul<type>,defaultDiv<type> >
#define GET_DOUBLE_ANALYSATION GET_TYPE_ANALYSATION(double)

class StatisticTools : public Inspectable, public Callbackable {

public:
  StatisticTools() : beginMeasureCounter(0) { }

	/**
	 * adds a variable to observe and measure the value
	 * @param observedValue    the value to observe.
	 * @param measureName      the name of the measured value
	 * @param mode             the mode of measure
	 * @param stepSpan         in most cases the stepSpan is important to get
	 * the measured value of a number of steps, like AVG:
	 * if stepSpan = 0, AVG is calculated over all steps
	 * if stepSpan = n, AVG is calculated over the LAST n steps
	 * The same counts for all the other MeasureModes.
	 * @param additionalParam  is used for example for mode PEAK, the param is the limit value,
	 * all values minus limit are displayed, values below the limit are set to 0.
  	 * In CONV mode (test the convergence), this value is the epsilon criteria.
	 * @return measured value as adress. So it is possible to measure this value again
	 */
  virtual double& addMeasure(double& observedValue, const char* measureName, MeasureMode mode, long stepSpan, double additionalParam=0);

  /**
   * Same as the method above, but instead of getting the calculated value back (the adress), you get
   * the StatisticMeasure itself
   */
  virtual StatisticMeasure* getMeasure(double& observedValue,const char* measureName, MeasureMode mode, long stepSpan, double additionalParam=0);


  /**
   * You can add another abstract measure you like. in some cases (e.g. complex
   * measures) it is better to let the measure decide how it likes to be initialized
   * @param measure the measure to add
   * @return the address value of the measure
   */
  virtual double& addMeasure(AbstractMeasure* measure);

  /**
   * You can add another abstract measure you like. in some cases (e.g. complex
   * measures) it is better to let the measure decide how it likes to be initialized
   * With this method you can add a list of AbstractMeasures.
   * @param measureList the list of measures to add
   * @return the address value of the first measure
   */
  virtual double& addMeasureList(std::list<AbstractMeasure*> measureList);


    /**
   * You can add another abstract measure you like. in some cases (e.g. complex
   * measures) it is better to let the measure decide how it likes to be initialized
   * With this method you can add a list of AbstractMeasures.
   * @param measureList the list of measures to add
   * @return the address value of the first measure
   */
  virtual double& addMeasureList(std::list<ComplexMeasure*> measureList);


    /**
   * You can add another abstract measure you like. in some cases (e.g. complex
   * measures) it is better to let the measure decide how it likes to be initialized
   * With this method you can add a list of AbstractMeasures.
   * @param measureList the list of measures to add
   * @return the address value of the first measure
   */
  virtual double& addMeasureList(std::list<StatisticMeasure*> measureList);



	/**
	 * starts the measure at a specific time. This is useful if there are
	 * values that have to be ignored at simulation start.
	 * @param step number of steps (normally simsteps) to wait for beginning the measures
	 */
	virtual void beginMeasureAt(long step);

  /**
   * Tells you wether the measures have already been started.
   * @return true if measures have already been started, otherwise false
   */
  virtual bool measureStarted() { return (beginMeasureCounter==0?true:false); }


	/**
	 * CALLBACKABLE INTERFACE
	 *
	 *	this method is invoked when a callback is done from the class where this
	 * class is for callback registered
	 */
	virtual void doOnCallBack();

protected:
	std::list<AbstractMeasure*> activeMeasures;
	long beginMeasureCounter;
};


/**
 * use this function if you want more than one value analysed
 * class type must implement following operators:
 *
 * @param values (vector<type>) values for the analysation
 * @return (TemplateValueAnalysation) the analysation context
 */
template
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
ANALYSATION_CONTEXT* getAnalysation(std::vector<type> values) {
	return new ANALYSATION_CONTEXT(values);
}

/**
 * class type must implement following operators:
 *
 * @param tvAnalysation (TemplateValueAnalysation) the analysation context
 * @param mode (AnalysationMode) what value you want
 * @param feature (unsigned int) special param. for mode
 * @return (type) the value you want
 */
template
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
type getAnalysation(ANALYSATION_CONTEXT* tvAnalysation, AnalysationMode mode, unsigned int feature = 0) {
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
	case AM_BEST:
		return tvAnalysation->getBest();
	default:
		return zero();
	}
}

/**
 * class type must implement following operators:
 *
 * @param values (vector<type>) values for the analysation
 * @param mode (AnalysationMode) what value you want
 * @param feature (unsigned int) special param. for mode
 * @return (type) the value you want
 */
template
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
type getAnalysation(std::vector<type> values, AnalysationMode mode, unsigned int feature = 0) {
	ANALYSATION_CONTEXT* context = GET_TYPE_ANALYSATION(type)(values);
	return GET_TYPE_ANALYSATION(type)(context,mode,feature);
}

#endif
