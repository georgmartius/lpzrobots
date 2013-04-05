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
#ifndef __ELMAN_H
#define __ELMAN_H

#include <vector>

#include "multilayerffnn.h"

/// updates for network
class NetUpdate {
public:
  NetUpdate(){}
  NetUpdate(int numweights, int numbias, int numothers)
    : weights(numweights), bias(numweights), other(numothers) {}
  std::vector<matrix::Matrix> weights;
  std::vector<matrix::Matrix> bias;
  std::vector<matrix::Matrix> other;
};

/** Multilayer Neural Network with context neurons (after Elman and Jordan)
Example of 2 hidden layer network with both, elman and jordan context units.
\pre{
+--<-----O O O
|        | | |
|        H H H
|        | | |
|        | | |
|        | | |
|        H H H ----->-----+ 1:1 fixed connections (time delayed)
|   >->-/| | |\-<-<       |
|  / / / | | | \ \ \      |
| J J J  I I I  E E E     |
+-^-^-^         ^-^-^--<--+
}
 */
class Elman : public MultiLayerFFNN {
public:
  /**
     @param eps learning rate
     @param layers Layer description (the input layer is not specified (always linear))
     @param lambda self-recurrent feedback strength of context neurons
  */
  Elman(double eps, const std::vector<Layer>& layers,
        bool useElman, bool useJordan=false, bool useBypass=false)
    : MultiLayerFFNN(eps,layers,useBypass), useElman(useElman), useJordan(useJordan) {

    initialised = false;
  }

  virtual ~Elman(){ }

  /// initialisation of the network with the given number of input and output units
  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0);

  /** passive processing of the input
      (this will be different for every input, since it is a recurrent network)
  */
  virtual const matrix::Matrix process (const matrix::Matrix& input);

  /// performs learning and returns the network output before learning
  virtual const matrix::Matrix learn (const matrix::Matrix& input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1);

  /** determines the weight and bias updates
   */
  virtual NetUpdate weightIncrement(const matrix::Matrix& xsi);

  /** like weightIncrement but with blocked backprop flow for some neurons.
      @param blockedlayer index of layer with blocked neurons
      @param blockfrom index of neuron in blockedlayer to start blocking
      @param blockto index of neuron in blockedlayer to end blocking (if -1 then to end)
      (not included)
   */
  virtual NetUpdate weightIncrementBlocked(const matrix::Matrix& xsi_,
                                           int blockedlayer,
                                           int blockfrom, int blockto);


  /** applies the weight increments to the weight (and bias) matrices
      with the learningrate and the learnRateFactor */
  virtual void updateWeights(const NetUpdate& updates);


  /* Is implemented in multilayerfnn
     virtual const matrix::Matrix response(const matrix::Matrix& input) const;
   */

  void damp(double damping);

  /**************  STOREABLE **********************************/
  /// stores the layer binary into file stream
  bool store(FILE* f) const;
  /// restores the layer binary from file stream
  bool restore(FILE* f);


  /************** CONFIGURABLE INTERFACE ************************/
  virtual paramkey getName() const {
    return std::string("elmanNN");
  }



  /************** Inspectable INTERFACE ************************/
  virtual iparamkeylist getInternalParamNames() const;
  virtual iparamvallist getInternalParams() const;
  virtual ilayerlist getStructuralLayers() const;
  virtual iconnectionlist getStructuralConnections() const;

protected:
  matrix::Matrix elmanWeights;
  matrix::Matrix elmanContext;
  matrix::Matrix jordanWeights;
  matrix::Matrix jordanContext;
  bool useElman;
  bool useJordan;

};

#endif
