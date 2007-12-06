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
*   Revision 1.3  2007/09/28 08:48:21  robot3
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
#ifndef _COMPLEX_MEASURE_H
#define _COMPLEX_MEASURE_H

#include "abstractmeasure.h"
#include <list>

/** measure modes of complex measures.
 */
enum ComplexMeasureMode {
  /// returns the entropy of the value, uses update formula, needs O(1)
  ENT,
  /// returns the entropy of the value, uses normal formula, needs O(n) or O(m*n)
  ENTSLOW
};

class Discretisizer;

class ComplexMeasure : public AbstractMeasure {

  public:

 /**
  * creates a new complex measure. the calculated things are such like
  * mutual information, entropy, joint entropy and so on.
  * it`s possible to add new ones, see above for the
  * ComplexMeasureModes.
  * Don"t forget! to add observed values! with the method @see addObservable
  * @param measureName the name of the measure, needed for PlotOptions and
  * HUDSM
  * @param mode the measure you like to have
  * @param numberBins in earlier versions named as intervalCount. For complex
  * measures the observedValue has to be discretisized, this does the
  * ComplexMeasure with the class Discretisizer for you.
  */
ComplexMeasure( char* measureName, ComplexMeasureMode mode, int numberBins );



    /**
     * adds a observed variable to the measure.
     * @param observedValue address of the observed value
     * @param minValue minimum value the observed value can become
     * @param maxValue maximum value the observed value can become 
     */
    virtual void addObservable( double& observedValue, double minValue, double maxValue );

    virtual ~ComplexMeasure();

    /**
     * defined by AbstractMeasure. This method is called from StatisticTools
     * for updating the measure in every simStep (ODE).
     */
    virtual void step();


  protected:
  std::list<double*> observedValueList; // stores the adresses of the observedValues
  std::list<Discretisizer*> discretisizerList; // stores the Discretisizer
  ComplexMeasureMode mode;
  int numberBins;
  int fSize; // size of F
  int *F; // stores the frequencies as a linear vector.

  
  
    // calculation methods

    /**
     * updates the entropy. uses update rule with O(1) costs
     * @param binNumbe1 the bin number
     */
    void updateEntropy( int binNumber);

    /**
     * computes the entropy. uses the normal rule with O(m*n*o) costs
     */
    void computeEntropy();

  
    /**
     * inits F, neccessary after each call of addObservable()
     * 
     */
    void initF();


  };

#endif
