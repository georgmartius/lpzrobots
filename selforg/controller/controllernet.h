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
#ifndef __CONTROLLERNET_H
#define __CONTROLLERNET_H

#include <vector>

#include "feedforwardnn.h"
#include "layer.h"

/** multi layer neural network with configurable activation functions
    and propagation and projection methods suitable for homeokinesis controller
 */
class ControllerNet : public Configurable {
public:

  /**
     @param layers Layer description (the input layer is not specified (always linear))
     @param useBypass if true, then a connection from input to output layer is included
  */
  ControllerNet(const std::vector<Layer>& layers, bool useBypass=false);
  virtual ~ControllerNet(){ }

  /** initialisation of the network with the given number of input and output units.
      The dimensionality of the ouputlayer is automatically adjusted.
      @param unit_map defines the approximate response of the network
       after initialisation (if unit_map=1 the weights are unit matrices).
      @param randGen pointer to random generator, if 0 an new one is used
   */
  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, double rand = 0.2, RandGen* randGen = 0);

  /** passive processing of the input.
      This has to be done before calling reponse, and the back/forward propagation/projection functions.
      The activations and the response matrix are stored internally.
   */
  virtual const matrix::Matrix process (const matrix::Matrix& input);

  /** like process just with the opportunity to overwrite the activation of
      a specific layer
      @param injections the input that is clamped at layer injectInLayer
      @param injectInLayer the injection is clamped at this layer
   */
  virtual const matrix::Matrix processX (const matrix::Matrix& input,
                                         const matrix::Matrix& injection,
                                         unsigned int injectInLayer);

  /// damps the weights and the biases by multiplying (1-damping)
  virtual void damp(double damping);

  /** response matrix of neural network (for current activation, see process)
  \f[  J_ij = \frac{\partial y_i}{\partial x_j} \f]
  \f[  J = G_n' W_n G_{n-1}' W_{n-1} ... G_1' W_1 \f]
  with \f$W_n\f$ is the weight matrix of layer n and
  \f$ G'\f$ is a diagonal matrix with \f$ G'_ii = g'_i \f$ as values on the diagonal.
  */
  virtual const matrix::Matrix& response() const;

  /** like response, just that only a range of layers is considered
      The Bypass is not considered here.
      @param from index of layer to start: -1 at input, 0 first hidden layer ...
      @param to index of layer to stop: -1: last layer, 0 first hidden layer ...
   */
  virtual matrix::Matrix responsePart(int from, int to) const;


  /** linear response matrix of neural network
  \f[  R = W_n W_{n-1} ... W_1 \f]
    with \f$W_n\f$ is the weight matrix of layer n.
  */
  virtual const matrix::Matrix& responseLinear() const;

  /** backpropagation of vector error through network.
      The storage for the intermediate values (errors, zetas) do not need to be given.
      The errors(layerwise) are at the output of the neurons
      (index 0 is at the input level, output of layer 0 has index 1 and so on)
      The zetas(layerwise) are the values inside the neurons that
      arise when backpropagating the error signal. (zeta[0] is at layer 0)
      @return errors[0] (result of backpropagation)
   */
  virtual const matrix::Matrix backpropagation(const matrix::Matrix& error,
                                               matrix::Matrices* errors = 0,
                                               matrix::Matrices* zetas = 0) const;

  /** like backpropagation but with special features: we can start from any layer
      and the bypass-discounting can be used (see disseration Georg Martius)
      WARNING: the errors and zetas above the `startWithLayer' are undefined
      @param startWithLayer the error is clamped at this layer and the processing starts there
        (-1: output layer)
      @see backpropagation
   */
  virtual const matrix::Matrix backpropagationX(const matrix::Matrix& error,
                                                matrix::Matrices* errors = 0,
                                                matrix::Matrices* zetas = 0,
                                                int startWithLayer = -1) const;


  /** backprojection of vector error through network.
      The storage for the intermediate values (errors, zetas) do not need to be given.
      The errors(layerwise) are at the output of the neurons
      (index 0 is at the input level, output of layer 0 has index 1 and so on)
      The zetas(layerwise) are the values inside the neurons that
      arise when backprojecting the error signal. (zeta[0] is at layer 0)
      @return errors[0] (result of backprojecting)
   */
  virtual const matrix::Matrix backprojection(const matrix::Matrix& error,
                                              matrix::Matrices* errors = 0,
                                              matrix::Matrices* zetas = 0) const;


  /** forwardpropagation of vector error through network.
      The storage for the intermediate values (errors, zetas) do not need to be given.
      The errors(layerwise) are at the output of the neurons
      (index 0 is at the input level = error, output of layer 0 has index 1 and so on)
      The zetas(layerwise) are the values inside the neurons that
      arise when forwardpropagate the error signal. (zeta[0] is at layer 0)
      @return errors[layernum] (result of forwardpropagation)
   */
  virtual const matrix::Matrix forwardpropagation(const matrix::Matrix& error,
                                               matrix::Matrices* errors = 0,
                                               matrix::Matrices* zetas = 0) const;

  /** forwardprojection of vector error through network.
      The storage for the intermediate values (errors, zetas) do not need to be given.
      The errors(layerwise) are at the output of the neurons
      (index 0 is at the input level = error, output of layer 0 has index 1 and so on)
      The zetas(layerwise) are the values inside the neurons that
      arise when forwardprojecting the error signal. (zeta[0] is at layer 0)
      @return errors[layernum] (result of forwardprojection)
   */
  virtual const matrix::Matrix forwardprojection(const matrix::Matrix& error,
                                               matrix::Matrices* errors = 0,
                                               matrix::Matrices* zetas = 0) const;

  /// returns the number of input neurons
  virtual unsigned int getInputDim() const {
    return weights[0].getN();
  }
  /// returns the number of output neurons
  virtual unsigned int getOutputDim() const {
    return (weights.rbegin())->getM();
  }

  /** returns activation of the given layer. Layer 0 is the first hidden layer.
      Negative values count from the end (-1 is the last layer)
   */
  virtual const matrix::Matrix& getLayerOutput(int layer) const {
    if(layer<0) layer = layers.size() + layer;
    assert(layer>=0 && layer < (int)layers.size());
    return y[layer];
  }

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

  /** weight matrix 0 connects input with the first hidden layer
      Negative values count from the end (-1 is the last layer)
  */
  virtual const matrix::Matrix& getWeights(int to_layer) const {
    if(to_layer<0) to_layer  = weights.size() - to_layer;
    assert(to_layer>=0 && to_layer < (int)weights.size());
    return weights[to_layer];
  }

  /** weight matrix 0 connects input with the first hidden layer
      Negative values count from the end (-1 is the last layer)
  */
  virtual matrix::Matrix& getWeights(int to_layer) {
    if(to_layer<0) to_layer  = weights.size() - to_layer;
    assert(to_layer>=0 && to_layer < (int)weights.size());
    return weights[to_layer];
  }

  virtual const matrix::Matrix& getByPass() const  {
    assert(useBypass);
    return bypassWeights;
  }

  virtual matrix::Matrix& getByPass() {
    assert(useBypass);
    return bypassWeights;
  }

  /** Note: layers 0 is the first hidden layer
      Negative values count from the end (-1 is the last layer)
  */
  virtual const matrix::Matrix& getBias(int of_layer) const {
    if(of_layer<0) of_layer  = bias.size() - of_layer;
    assert(of_layer>=0 && of_layer < (int)bias.size());
    return bias[of_layer];
  }

  /** Note: layers 0 is the first hidden layer
      Negative values count from the end (-1 is the last layer)
  */
  virtual matrix::Matrix& getBias(int of_layer) {
    if(of_layer<0) of_layer  = bias.size() - of_layer;
    assert(of_layer>=0 && of_layer < (int)bias.size());
    return bias[of_layer];
  }

  /**************  STOREABLE **********************************/
  /// stores the layer binary into file stream
  bool store(FILE* f) const;
  /// restores the layer binary from file stream
  bool restore(FILE* f);

  /// writes the layer ASCII into file stream (not in the storable interface)
  bool write(FILE* f) const;

protected:
  // actually calculate the jacobian and stores it in L, see response()
  virtual void calcResponseIntern();


protected:
  std::vector<Layer> layers;
  std::vector<matrix::Matrix> weights;
  std::vector<matrix::Matrix> bias;
  bool useBypass;
  matrix::Matrix bypassWeights;

  /*** storage variables ****/
  ///
  matrix::Matrix input;
  matrix::Matrices y; // activations
  matrix::Matrices z; // potentials
  matrix::Matrices gp; // g'

  matrix::Matrix L; // jacobian (or response) matrix
  matrix::Matrix R; // linearized jacobian matrix

  double lambda;   // regularisation value for pseudoinverse
  bool initialised;
};

#endif
