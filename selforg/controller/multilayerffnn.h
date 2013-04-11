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
#ifndef __MULTILAYERFFNN_H
#define __MULTILAYERFFNN_H

#include <vector>

#include "feedforwardnn.h"
#include "invertablemodel.h"
#include "layer.h"


/// multi layer neural network with configurable activation functions
class MultiLayerFFNN : public FeedForwardNN {
public:

  /**
     @param eps learning rate
     @param layers Layer description (the input layer is not specified (always linear))
     @param useBypass if true, then a connection from input to output layer is included
     @param someInternalParams if true then only a few parameters are send to plotting
  */
  MultiLayerFFNN(double eps, const std::vector<Layer>& layers, bool useBypass=false,
                 bool someInternalParams=true);
  virtual ~MultiLayerFFNN(){ }

  /** initialisation of the network with the given number of input and output units.
      The dimensionality of the ouputlayer is automatically adjusted.
      @param unit_map defines the approximate response of the network
       after initialisation (if unit_map=1 the weights are unit matrices).
      @param randGen pointer to random generator, if 0 an new one is used
   */
  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0);

  /// passive processing of the input
  virtual const matrix::Matrix process (const matrix::Matrix& input);

  /** performs learning and returns the network output before learning
      (process should be called before) */
  virtual const matrix::Matrix learn (const matrix::Matrix& input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1);

  /** response matrix of neural network at given input

  \f[  J_ij = \frac{\partial y_i}{\partial x_j} \f]
  \f[  J = G_n' W_n G_{n-1}' W_{n-1} ... G_1' W_1 \f]
  with \f$W_n\f$ is the weight matrix of layer n and
  \f$ G'\f$ is a diagonal matrix with \f$ G'_ii = g'_i \f$ as values on the diagonal.
  ATTENTION: input is ignored! use process before!
  */
  virtual const matrix::Matrix response(const matrix::Matrix& input) const;

  /** calculates the input shift v to a given output shift xsi via pseudo inversion.

      \f[o+\xi = \psi(i+v)\f]

      The result is a vector of dimension inputdim.
      ATTENTION: input is ignored! use process before!
   */
  virtual const matrix::Matrix inversion(const matrix::Matrix& input, const matrix::Matrix& xsi) const;


  /// returns the number of input neurons
  virtual unsigned int getInputDim() const {
    return weights[0].getN();
  }
  /// returns the number of output neurons
  virtual unsigned int getOutputDim() const {
    return (weights.rbegin())->getM();
  }

  /// returns activation of the given layer. Layer 0 is the first hidden layer
  virtual const matrix::Matrix& getLayerOutput(unsigned int layer) const {
    assert(layer < layers.size());
    return ys[layer];
  }

  /// damps the weights and the biases by multiplying (1-damping)
  virtual void damp(double damping);

  // total number of layers (1 means no hidden units)
  virtual unsigned int getLayerNum() const {
    return layers.size();
  }

  /// layers 0 is the first hidden layer
  virtual const Layer& getLayer(unsigned int layer) const {
    assert(layer < layers.size());
    return layers[layer];
  }

  /// layers 0 is the first hidden layer
  virtual Layer& getLayer(unsigned int layer) {
    assert(layer < layers.size());
    return layers[layer];
  }

  /// weight matrix 0 connects input with the first hidden layer
  virtual const matrix::Matrix& getWeights(unsigned int to_layer) const {
    assert(to_layer < weights.size());
    return weights[to_layer];
  }

  /// weight matrix 0 connects input with the first hidden layer
  virtual matrix::Matrix& getWeights(unsigned int to_layer) {
    assert(to_layer < weights.size());
    return weights[to_layer];
  }

  /// Note: layers 0 is the first hidden layer
  virtual const matrix::Matrix& getBias(unsigned int of_layer) const {
    assert(of_layer < bias.size());
    return bias[of_layer];
  }

  /// Note: layers 0 is the first hidden layer
  virtual matrix::Matrix& getBias(unsigned int of_layer) {
    assert(of_layer < bias.size());
    return bias[of_layer];
  }

  /**************  STOREABLE **********************************/
  /// stores the layer binary into file stream
  bool store(FILE* f) const;
  /// restores the layer binary from file stream
  bool restore(FILE* f);


  /// writes the layer ASCII into file stream (not in the storable interface)
  bool write(FILE* f) const;

  /************** Inspectable **********************************/
  virtual iparamkeylist getInternalParamNames() const;
  virtual iparamvallist getInternalParams() const;
  virtual ilayerlist getStructuralLayers() const;
  virtual iconnectionlist getStructuralConnections() const;


  virtual void setSomeInternalParams(bool someInternalParams){
    assert(!initialised); this->someInternalParams = someInternalParams;
  }

public:
  double eps; ///< learning rate

  /**
  * sets the activation function (and derivative and inversion too) for ALL layers!
  * @param actfun the activation function to be used
  * @return the activation functions which where used until now
  */
  virtual std::vector<ActivationFunction> setActivationFunction(ActivationFunction actfun);

/**
  * sets the activation functions (and derivative and inversion too) for all layers.
  * @note: normally you call setActivationFunction() first and get a list of the used
  * activation functions, which are set back with this function
  * @param actfunList the list of actfuns to be used
  */
  virtual void setActivationFunctions(std::vector<ActivationFunction> actfunList);


protected:
  std::vector<Layer> layers;
  std::vector<matrix::Matrix> weights;
  std::vector<matrix::Matrix> bias;
  std::vector<matrix::Matrix> smallids; // small unit matrices for pseudoinversion
  bool useBypass;
  matrix::Matrix bypassWeights;
  bool someInternalParams;

  matrix::Matrix input;
  std::vector<matrix::Matrix> ys; // activations
  std::vector<matrix::Matrix> zs; // potentials

  double lambda;   // regularisation value for pseudoinverse
  bool initialised;
};

#endif
