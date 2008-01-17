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
*   Revision 1.3  2008-01-17 09:59:27  der
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
#include <math.h>
#include "stl_adds.h"
#include <assert.h>


ComplexMeasure::ComplexMeasure( const char* measureName, ComplexMeasureMode mode, int numberBins ) : AbstractMeasure( measureName ), mode( mode ), numberBins( numberBins ) {
  historySize=2;
  historyIndex=-1;
  binNumberHistory = ( int* ) malloc( sizeof( double ) * historySize );
}


ComplexMeasure::~ComplexMeasure()
{
  if ( F )
    free( F );
  if (binNumberHistory)
    free(binNumberHistory);
}


void ComplexMeasure::step()
{
  if (observedValueList.size()==0)
    return;
  int valNumber = 0;
  int binNumber=0;
  std::list<Discretisizer*>::iterator di = discretisizerList.begin();
  FOREACH( std::list<double*>, observedValueList, oValue )
  {
    binNumber += (int) pow( numberBins,valNumber++)* ( *di ) ->getBinNumber( *(*oValue));
    di++;
  }
  actualStep++;
  switch ( mode )
  {
  case ENT:
    if (actualStep<10)
    {
      F[ binNumber ] ++;
      computeEntropy();
    }
    else
    {
      updateEntropy( binNumber );
      // now make update of F
      F[ binNumber ] ++;
    }
    break;
  case ENTSLOW:
    F[ binNumber ] ++;
    computeEntropy();
    break;
  case MI:
    updateMI(binNumber);
    F[ binNumber ] ++;
    break;
  default:
    break;
  }
  historyIndex++;
  if (historyIndex==historySize)
    historyIndex=0;
  binNumberHistory[historyIndex]=binNumber;
}


void ComplexMeasure::updateMI(int binNumber) {/*
  int newState = binNumber;
  int oldState = binNumberHistory[historyIndex];
    // calculate dS
  int n = numberBins; // use only for more compact formula
  double dS=0.0;
  if (((*F)->val(n,newState))>0.0)
    dS=((*F)->val(n,newState)) * log(((*F)->val(n,newState)));
  if (((*F)->val(oldState,n))>0.0)
    dS+=((*F)->val(oldState,n)) * log(((*F)->val(oldState,n)));
  if (((*F)->val(oldState,newState))>0.0)
    dS-=((*F)->val(oldState,newState)) * log(((*F)->val(oldState,newState)));
  dS+=(((*F)->val(oldState,newState))+1) * log(((*F)->val(oldState,newState))+1)
    -(((*F)->val(n,newState))+1) * log(((*F)->val(n,newState))+1)
    -(((*F)->val(oldState,n))+1) * log(((*F)->val(oldState,n))+1);
    // updateMI with old MI and dS
  double t = (double)(this->t);
  double tminus1 = (double)(t-1);
  if ((this->t)==1)
  {
      // log(t-1) is infinite, use more simple formula
    MI[i]=dS / t + log(t);
  }
  else
  {
    MI[i]=((tminus1)*(MI[i] - log(tminus1)) + dS) / t + log(t);
  }
  */
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
  if ( F[ binNumber ] > 0 )
  { // calculating log(0) is not smart ;)
    dS = ((double)( F[ binNumber ] + 1 )) * log((double) (F[ binNumber ] + 1) ) -
         ((double) F[ binNumber ] )* log((double) F[ binNumber ] );
  }// else {
  //dS = ;
  //}
  // update Entropy with old Entropy and dS
  if ( actualStep > 1 )
  {
    std::cout << "dS=" << dS << ",actualStep=" << actualStep << ",value=" << value << ",log(actualStep-1)=" << log(double(actualStep-1)) << ",log((double)(actualStep))=" << log(((double)actualStep)) << std::endl;
    value = ( ((double)(actualStep-1)) * (value - log((double) (actualStep-1))- dS  )) / ((double)(actualStep)) + log((double) (actualStep));
  }
  else
    value=0;
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
    fSize = (int) pow( numberBins+1, observedValueList.size() );
  else
    fSize = (int) pow( numberBins, observedValueList.size() );
  // if ( F )
  //  free( F );
  F = ( int* ) malloc( sizeof( double ) * fSize );
  for ( int i = 0; i < fSize;i++ )
    F[ i ] =0;
}

