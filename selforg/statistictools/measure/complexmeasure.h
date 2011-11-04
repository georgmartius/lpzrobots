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
#ifndef _COMPLEX_MEASURE_H
#define _COMPLEX_MEASURE_H

#include "abstractmeasure.h"
#include <list>

#include "sparsearray.h"

/** measure modes of complex measures.
 */
enum ComplexMeasureMode {
  /// returns the entropy of the value, uses update formula, needs O(1)
  ENT,
  /// returns the entropy of the value, uses normal formula, needs O(n) or O(m*n)
  ENTSLOW,
  /// returns the mutual information of two values, uses update formula, needs O(1)
  MI,
  /// returns the predictive information of two or more values
  PINF
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
ComplexMeasure( const char* measureName, ComplexMeasureMode mode, int numberBins );



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
  long fSize; // size of F
  int historySize; // size of binNumberHistory
//  int *F; // stores the frequencies as a linear vector
  int *binNumberHistory; // holds the binNumbers as an history, for predictive information 2 values are enough
  int historyIndex; // index of last stored value
  int *historyIndexList; // indexes of relevant stored values
  int historyIndexNumber; // number of indexes stored in historyIndexList
  int historyInterval; // interval between two different histoy indexes

  // new: use SparseArray backed by HashMap instead of normal array
  matrix::SparseArray<long, int> F;
    // calculation methods

      /**
     * calculates the Predictive Information
     */
    void calculatePInf();


    /**
     * updates the entropy. uses update rule with O(1) costs
     * @param binNumber the bin number
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
