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
#ifndef _TRACKABLE_MEASURE_H
#define _TRACKABLE_MEASURE_H

#include "complexmeasure.h"
#include "trackable.h"
#include <list>
#include "position.h"

/**
 * NOTE: SPEED and ANGSPEED is not implemented yet!
 */
enum TrackMode {
    POS, /// takes the position for the measure
    SPEED, /// takes the speed for the measure
    ANGSPEED /// takes the angular speed for the measure
};

/// defines which dimensions should be count.
enum Dimensions { X = 1, Y = 2, Z = 4 };


class TrackableMeasure : public ComplexMeasure {

public:

  /**
   * creates a new TrackableMeasure. The position of the Trackables in the
   * given list is counted in a fequency list. The possible positions are
   * given by the cornerPointList, which contains the cornerPoints of the
   * arena where the trackables are placed.
   * the complex measure ist then based on the frequency list
   * @param trackableList the list of the Trackables
   * @param cornerPointList the list with the cornerPoints of the arena
   * @param dimensions which dimensions do you like to count? Note that
   * the needed memory is (numberBins^ndim), but calculation costs are O(1)
   * @param cmode which type of complex measure should be evaluated?
   * @param numberBins number of bins used for discretisation
   */
  TrackableMeasure(std::list<Trackable*> trackableList,const char* measureName  ,ComplexMeasureMode cmode,std::list<Position> cornerPointList, short dimensions, int numberBins);

  //virtual ~TrackableMeasure();

  /**
   * defined by AbstractMeasure. This method is called from StatisticTools
      for updating the measure in every simStep (ODE).
   */
  virtual void step();


protected:
  std::list<Trackable*> trackableList;
  ComplexMeasureMode    cmode;
  TrackMode             tmode;

  virtual double findRange(std::list<Position>  positionList,short dim, bool min);

  virtual void addDimension(short dim, std::list<Position> cornerPointList);

};

#endif
