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
 *  windowstatistics provides cool stuff for displaying statistics on the  *
 *  graphics window.                                                       *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2007-09-27 10:48:13  robot3
 *   first version of the WSM
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __WINDOWSTATISTICS_H
#define __WINDOWSTATISTICS_H

#import <selforg/statistictools.h>

namespace osgText {
class Text;
}

namespace osg {
class Geode;
}


namespace lpzrobots {

/**
 * manages all the stuff displaying statistics on the graphics window.
 * This is a experimental version, so do not to be afraid changing this crazy
 * code.
 *
 * This class uses the implementation of the class StatisticTools, which is
 * generalized to make nice statistics. Instead of passing the values to the
 * guilogger (INSPECTABLE interface), we simply diplay this values on the graphics
 * window.
 *
 * So how it works:
 * - overwriting the method addMeasure gives us the ability to create the needed text
 *   object, then storing in the textObjectList.
 */
class WindowStatisticsManager : public StatisticTools {

public:
  /**
   * creates the WindowStatisticsManager, normally done by class Base.
   * @param geode this is the graphical node at wich the text objects are hooked in.
   */
  WindowStatisticsManager(osg::Geode* geode);

  	/**
	 * adds a variable to observe (on the window) and measure the value
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
  virtual double& addMeasure(double& observedValue, char* measureName, MeasureMode mode, long stepSpan, double additionalParam=0);


	/**
	 * CALLBACKABLE INTERFACE
	 *
	 *	this method is invoked when a callback is done from the class where this
	 * class is for callback registered, it is overwritten
	 */
  virtual void doOnCallBack();

protected:

  /**
   * Nested class WindowStatistic, which puts the measure and the graphics text together.
   */
  class WindowStatistic {
  public:

    WindowStatistic(StatisticMeasure* measure, osgText::Text* text) : measure(measure),
      text(text) {}

    ~WindowStatistic() {}

    StatisticMeasure* getMeasure() { return measure; }

    osgText::Text* getText() { return text; }

  private:
    StatisticMeasure* measure;
    osgText::Text* text;
  };

/// the struct list which holds the measures and the appropiate text
  std::list<WindowStatistic*> windowStatisticList;

  // position of first graphical text
  float xInitPosition;
  float yInitPosition;
  float zInitPosition;
  float yOffset;

  // graphical node
  osg::Geode* geode;

};


}

#endif
