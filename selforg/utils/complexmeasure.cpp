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
*   Revision 1.1  2007-12-06 10:18:10  der
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


ComplexMeasure::ComplexMeasure( char* measureName, ComplexMeasureMode mode, int numberBins ) : AbstractMeasure( measureName ), mode( mode ), numberBins( numberBins ) {}


ComplexMeasure::~ComplexMeasure() {
  if ( F )
    free( F );
  }


void ComplexMeasure::step() {
  if (observedValueList.size()==0)
    return;
  int valNumber = 1;
  int binNumber=0;
  std::list<Discretisizer*>::iterator di = discretisizerList.begin();
  FOREACH( std::list<double*>, observedValueList, oValue ) {
    
    binNumber += (int) pow(( *di ) ->getBinNumber( *(*oValue)) ,valNumber++);
    di++;
  }
  actualStep++;
  //  std::cout << "computing step " << actualStep << " for bin=" << binNumber  << std::endl;
  switch ( mode ) {
      case ENT:
      if (actualStep<10){
        F[ binNumber ] ++;
        computeEntropy();
    }else {
    updateEntropy( binNumber );
      // now make update of F
      F[ binNumber ] ++;}
      break;
      case ENTSLOW:
      F[ binNumber ] ++;
      computeEntropy();
      break;
      default:
      break;
    }
  }


void ComplexMeasure::addObservable(double& observedValue,double minValue, double maxValue) {
  observedValueList.push_back(&observedValue);
  Discretisizer* dis = new Discretisizer(numberBins,minValue,maxValue,false);
  discretisizerList.push_back(dis);
  initF();
}


void ComplexMeasure::updateEntropy( int binNumber ) {
  // calculate dS
  double dS = 0;
  if ( F[ binNumber ] > 0 ) { // calculating log(0) is not smart ;)
    dS = ((double)( F[ binNumber ] + 1 )) * log((double) (F[ binNumber ] + 1) ) -
      ((double) F[ binNumber ] )* log((double) F[ binNumber ] );
  }// else {
    //dS = ;
    //}
  // update Entropy with old Entropy and dS
  if ( actualStep > 1 ) {
    value = - ( ((double)(actualStep-1)) * value + dS - log( actualStep-1) ) / ((double)(actualStep)) + log((double) (actualStep));
  } else
    value=-dS;
}


void ComplexMeasure::computeEntropy() {
  // calculate Entropy = - sum {from forall i in F} log F[i]
  double val = 0.0;
  for ( int i = 0; i < fSize;i++ ) {
    if ( F[ i ] > 0 ) {
      val += (((double)F[ i ])/((double)actualStep)) * log(((double) F[ i ]) /((double)actualStep));
      }
    }
  value = -val;
  }


void ComplexMeasure::initF() {
  // determine fSize
  fSize = (int) pow( numberBins, observedValueList.size() );
  // if ( F )
  //  free( F );
  F = ( int* ) malloc( sizeof( double ) * fSize );
  for ( int i = 0; i < fSize;i++ )
      F[ i ] =0;
  }

