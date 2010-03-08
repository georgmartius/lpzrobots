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
*   Revision 1.5  2010-03-08 15:43:30  martius
*   order of initialization
*
*   Revision 1.4  2009/08/11 12:05:43  robot12
*   disabled test output
*
*   Revision 1.3  2009/08/10 07:39:31  guettler
*   -example use of new SparseArray.
*   -some test implementations for entropy (to be tested, corrected)
*
*   Revision 1.2  2009/08/03 14:09:48  jhoffmann
*   Remove some compiling warnings, memory leaks; Add some code cleanups
*
*   Revision 1.1  2009/03/27 06:16:57  guettler
*   support for gcc 4.3 compatibility (has to be checked), StatisticTools moves from utils to statistictools
*
*   Revision 1.5  2008/05/30 11:58:27  martius
*   use cmath instead of math.h
*
*   Revision 1.4  2008/02/14 14:43:09  der
*   made some enhancements
*
*   Revision 1.3  2008/01/17 09:59:27  der
*   complexmeasure: preparations made for predictive information,
*   fixed a minor bug
*   statisticmeasure, statistictools: added support for adding
*   std::list<AbstractMeasure*> to StatisticTools, some minor
*   improvements
*
*   Revision 1.2  2007/12/07 08:52:37  der
*   made some tests
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
#include "complexmeasure.h"


#include "discretisizer.h"
#include "sparsearray.h"
#include <cmath>
#include "stl_adds.h"
#include <assert.h>
#include <cstdlib>


ComplexMeasure::ComplexMeasure( const char* measureName, ComplexMeasureMode mode, int numberBins ) : AbstractMeasure( measureName ), mode( mode ), numberBins( numberBins ), historyIndexList(0), F(0)
{
  historySize=2;
  historyIndex=-1;
  binNumberHistory = ( int* ) malloc( sizeof( double ) * historySize );
}


ComplexMeasure::~ComplexMeasure()
{

  if (binNumberHistory)
    free(binNumberHistory);

  if (historyIndexList)
    free(historyIndexList);

  observedValueList.erase(observedValueList.begin(), observedValueList.end());
  discretisizerList.erase(discretisizerList.begin(), discretisizerList.end());
}


void ComplexMeasure::step()
{
  /*if (actualStep%1000==0)
  {
    std::cout << "Size of F = " << (float)F.getRealSize() /1024 << " kbytes";
    std::cout << " (instead of " << sizeof(int) * F.size() / 1024 << " kbytes for an array of size " << F.size() << ")" << std::endl;
  }*/
  if (observedValueList.size()==0)
    return;
  int valNumber = 0;
  int binNumber=0;
  std::list<Discretisizer*>::iterator di = discretisizerList.begin();
  std::list<int> binList;
  switch (mode)
  {
  case MI:
  case PINF:/*
    binNumber = ( *di ) ->getBinNumber( *(*oValue));
    binList.push_back(binNumber);
    for (int i=0; i<historysize;i++)
    {
      binNumber += (int) pow( numberBins,i+1)* binNumberHistory[historyIndexList[i]];
      binList.push_back(binNumberHistory[historyIndexList[i]]);
    }*/
    break;
  default: // ENT, ENTSLOW
    FOREACH( std::list<double*>, observedValueList, oValue )
    {
      binNumber += (int) pow( numberBins,valNumber++)* ( *di ) ->getBinNumber( *(*oValue));
      di++;
    }
    break;
  }

  actualStep++;
  switch ( mode )
  {
  case ENT:
    updateEntropy( binNumber );
    // now make update of F
    F[ binNumber ] = F[ binNumber ] +1;
    break;
  case ENTSLOW:
    //  case ENT:
    F[ binNumber ] = F[binNumber]+1;
    computeEntropy();
    break;
  case MI:
  case PINF:/*
    calculatePI(binList);
    updateFforPI(binNumber, binList);*/
    break;
  default:
    break;
  }
  historyIndex++;
  if (historyIndex==historySize)
    historyIndex=0;
binNumberHistory[historyIndex]=binNumber;
}

  /*
void ComplexMeasure::updateFforPI(int binNumber, std::list<int> binList)
{
  F[binNumber]++;
  int i=0;
  FOREACH( std::list<int>, binList, bin )  {
    F[numberBins*numberBins+i*numberBins+bin]++;
  }
}*/



void ComplexMeasure::calculatePInf()
{
  // calculate PI
  double val = 0.0;
  for ( int i = 0; i < fSize;i++ )
  {
    if ( F[ i ] > 0 )
    {
      val += (((double)F[ i ])/((double)actualStep)) * log(((double) F[ i ]) /((double)actualStep));
    }
  }
  value = -val;
}

void ComplexMeasure::addObservable(double& observedValue,double minValue, double maxValue)
{
  observedValueList.push_back(&observedValue);
  Discretisizer* dis = new Discretisizer(numberBins,minValue,maxValue,false);
  discretisizerList.push_back(dis);
  initF();
}


void ComplexMeasure::updateEntropy( int binNumber )
{
  // calculate dS
  double dS = 0;
  double val = F[ binNumber ];
  double t = actualStep;
  if ( val > 0 )
  { // calculating log(0) is not smart ;)
    dS = (- val * log(val)) + ((val+1) * log(val+1));
  }
  // update Entropy with old Entropy and dS
  if (actualStep>1)
  {
    value= - ((1./t) * (((value - log(t-1.))*(t-1.)) + dS)) + log(t);
  }
}


void ComplexMeasure::computeEntropy()
{
  // calculate Entropy = - sum {from forall i in F} log F[i]
  double val = 0.0;
  for ( int i = 0; i < fSize;i++ )
  {
    if ( F[ i ] > 0 )
    {
      val += (((double)F[ i ])/((double)actualStep)) * log(((double) F[ i ]) /((double)actualStep));
    }
  }
  value = -val;
}


void ComplexMeasure::initF()
{
  // determine fSize
  if (mode==MI)
    fSize = (int) pow( numberBins, observedValueList.size() ) + (historyIndexNumber+1) * numberBins;
  else
    fSize = (int) pow( numberBins, observedValueList.size() );
  F.reallocate(fSize);
}

