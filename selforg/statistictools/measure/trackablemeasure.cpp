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
*   Revision 1.3  2009-08-03 14:09:48  jhoffmann
*   Remove some compiling warnings, memory leaks; Add some code cleanups
*
*   Revision 1.2  2009/07/15 12:59:05  robot12
*   one bugfixe in constructor (parameter type char* to const char*)
*
*   Revision 1.1  2009/03/27 06:16:58  guettler
*   support for gcc 4.3 compatibility (has to be checked), StatisticTools moves from utils to statistictools
*
*   Revision 1.1  2007/12/06 10:18:10  der
*   AbstractMeasure is now a abstract type for Measures,
*   StatisticTools now supports AbstractMeasures,
*   StatisticalMeasure, ComplexMeasure  now derived from
*   AbstractMeasure,
*   ComplexMeasure provides support for calculation e.g. entropy,
*   uses Discretisizer,
*   Discretisizer is a stand-alone class for support of discretisizing values
*   TrackableMeasure derived from ComplexMeasure and provides support for calculating complex measures for Trackable objects
*
*   Revision 1.6  2007/10/10 13:18:06  martius
*   math.h
*
*   Revision 1.5  2007/10/10 13:17:14  martius
*   use fabs instead of abs
*
*   Revision 1.4  2007/10/01 13:27:47  robot3
*   documentation
*
*   Revision 1.3  2007/09/28 08:48:20  robot3
*   corrected some minor bugs, files are still in develop status
*
*   Revision 1.2  2007/09/27 10:49:39  robot3
*   removed some minor bugs,
*   added CONVergence test
*   changed little things for support of the new WSM
*
*   Revision 1.1  2007/05/07 21:01:31  robot3
*   statistictools is a class for easy visualization of measurements of observed values
*   it is possible to add the observed value itself with mode ID
*
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

