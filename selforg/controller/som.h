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
#ifndef __SOM_H
#define __SOM_H

#include "abstractmodel.h"
#include "controller_misc.h"

#include <vector>

/** self-organised map class. Implementation follows normal vector-quantiser
 scheme.
The output of the network is  \f$exp(- |x-w_i|^2/rdfsize)\f$ for each neuron.
*/
class SOM : public AbstractModel {
public:
  typedef std::list< std::pair<int,double> > Neighbours;
  typedef std::vector<std::pair<matrix::Matrix,double> > Neighbourhood;

  SOM(const std::string& name="SOM",
      const std::string& revision = "$Id$");

  /** create a som
      @param dimensions number of dimensions of the neuron lattice
   */
  SOM(unsigned int dimensions, double sigma, double eps, double rbfsize,
      const std::string& name="SOM",
      const std::string& revision = "$Id$");
  virtual ~SOM(){};

  /** initialised som
      @param inputDim dimension of input vector
      @param outputDim number of outputneurons (must be a multiple of "dimensions" given at constructor)
      @param unit_map if zero then weights are randomly choosed, otherwise
             uniformly distributed in the inputspace of size (unit_map x unit_map x ...)
   */
  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0);

  virtual const matrix::Matrix process (const matrix::Matrix& input);

  /*  performs training. Nominal output is ignored.
      A zero-Matrix is returned.
      learnRateFactor can be given to modify eps for this learning step
      (process should be called before)
  */
  virtual const matrix::Matrix learn (const matrix::Matrix& input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1);

  virtual void damp(double damping) { return;}

  virtual unsigned int getInputDim() const { return weights[0].getM();}
  virtual unsigned int getOutputDim() const  { return weights.size();}


  virtual bool store(FILE* f) const;
  virtual bool restore(FILE* f);

  virtual void printWeights(FILE* f) const;

  const Neighbourhood& getNeighbourhood(){return neighbourhood;}

protected:

  /// activation function (rbf)
  static double activationfunction(void* rdfsize, double d);

  /// checks whether the given coordinate is within the lattice
  static bool validCoord(const matrix::Matrix& m, int size);
  /// converts index  to coordinates (size is the size of the space)
  static int coordToIndex(const matrix::Matrix& m, int size);
  /// converts coordinates to index (size is the size of the space)
  static matrix::Matrix indexToCoord(int index, int size, int dimensions);


  /// initialised neighbourhood
  void initNeighbourhood(double sigma);

  /** returns neighbourhood as a list of indices with weights
  */
  Neighbours getNeighbours(int winner);


public:
  double eps; ///< learning rate for weight update
private:

  std::vector<matrix::Matrix> weights;
  std::vector<matrix::Matrix> diffvectors; ///< temporary difference vectors
  matrix::Matrix distances; ///< vector of distances
  int dimensions; ///< number of dimensions of lattice
  double sigma; ///< neighbourhood size
  double rbfsize; ///< size of rbf function
  int size; ///< size of the lattice in each dimension


  /// list of vectors defining relative neighbourhood coordinates and weights
  Neighbourhood neighbourhood;


  bool initialised;
};


#endif
