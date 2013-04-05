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
#ifndef __NEURALGAS_H
#define __NEURALGAS_H

#include "abstractmodel.h"
#include "controller_misc.h"

#include <vector>

/** neural gas class. Implementation follows roughly Martinetz scheme.
    The output of the network is  \f$exp(- |x-w_i|^2/cellsize)\f$ for each neuron,
    where cellsize is distance to the second closest neigbour.
*/
class NeuralGas : public AbstractModel {
public:
  NeuralGas(const std::string& name = "NeuralGas", const std::string& revision = "$Id$");
  /** create a som
      @param lambda initial competetive constant for neighborhood learning
      @param eps initial  learning rate
      @param maxTime maximal time we expect the network to learn, if 0 no annealing is performed
   */
  NeuralGas(double lambda, double eps, int maxTime,
      const std::string& name="NeuralGas",
      const std::string& revision = "$Id$");
  virtual ~NeuralGas(){};

  /** initialised som
      @param inputDim dimension of input vector
      @param outputDim number of outputneurons
      @param unit_map interval for randomly choosen weights.
               if zero then (-1,1) is used otherwise (-unit_map, unit_map) (in all dimensions)
   */
  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0);

  virtual const matrix::Matrix process (const matrix::Matrix& input);

  /*  performs training. Nominal output is ignored.
      A zero matrix is returned.
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
  virtual void printCellsizes(FILE* f) const;

protected:
  /// updates the cell sizes
  void updateCellSizes();

  /// activation function (rbf)
  static double activationfunction(double rdfsize, double d);

public:
  double eps; ///< initial learning rate for weight update
private:

  std::vector<matrix::Matrix> weights;
  std::vector<matrix::Matrix> diffvectors; ///< temporary difference vectors
  matrix::Matrix distances; ///< vector of distances
  matrix::Matrix cellsizes; ///< vector of cell sizes
  double lambda; ///< initial neighbourhood size
  int maxTime;   ///< maximal time for annealing
  int t;         ///< time used for annealing

  bool initialised;
};


#endif
