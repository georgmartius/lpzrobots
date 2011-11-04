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
#include "trackablemeasure.h"
#include "discretisizer.h"
#include "stl_adds.h"


TrackableMeasure::TrackableMeasure(std::list<Trackable*> trackableList,const char* measureName  ,ComplexMeasureMode cmode,std::list<Position> cornerPointList, short dimensions, int numberBins) : ComplexMeasure(measureName,cmode, numberBins) ,trackableList(trackableList)
{
  tmode=POS;
  if (dimensions & X)
    addDimension(0, cornerPointList);
  if (dimensions & Y)
    addDimension(1, cornerPointList);
  if (dimensions & Z)
    addDimension(2, cornerPointList);
  initF();
}

void TrackableMeasure::addDimension(short dim, std::list<Position> cornerPointList)
{
  double minValue = this->findRange(cornerPointList,0,true);
  double maxValue = this->findRange(cornerPointList,0,false);
  discretisizerList.push_back(new Discretisizer( numberBins, minValue, maxValue, false ));
  observedValueList.push_back(new double);
}

double TrackableMeasure::findRange(std::list<Position> positionList,short dim, bool min)
{
  double range = -1e20;
  if (min)
    range*=-1;
  FOREACH(std::list<Position>, positionList, i){
    double val = (i)->toArray()[dim];
    if (min) {
      if (val<range)
        range=val;
    } else {
      if (val>range)
        range=val;
    }
  }
  return range;
}

/*
TrackableMeasure::~TrackableMeasure()
{
  trackableList.erase(trackableList.begin(), trackableList.end());
}
*/

void TrackableMeasure::step()
{
  FOREACH(std::list<Trackable*>, trackableList, i)
  {
    Position pos;
    pos =(*i)->getPosition();
    /*if (tmode & POS) {
      pos =(*i)->getPosition();
    } else if (tmode & SPEED) {
      pos =(*i)->getSpeed();
    } else {
      pos =(*i)->getAngularSpeed();
    }*/
    int j=0;
    FOREACH(std::list<double*>,observedValueList,oVal)
    {
      *(*oVal)= pos.toArray()[j++];
    }
    ComplexMeasure::step();
  }
}

