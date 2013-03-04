/*****************************************************************************
 *             FIAS winter school, playful machine group                     *
 *                Supervisors: Ralf Der, Georg Martius                       *

 * Members: Fabien Benureau, Chrisantha Fernando, Quan Wang, Jimmy Baraglia  *
 *                   Echo State Network Header File                          *
 *                                                                           *
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
#ifndef __ESN_H
#define __ESN_H


#include <stdio.h>
#include <selforg/abstractmodel.h>
#include <selforg/matrix.h>


struct ESNConf {
  int numNeurons;         ///< number of neurons in the reservoir
  double connectionStrength; ///< strength of internal and input connections
  double connectionRatio; ///< ratio of internal connections w.r.t full connectivity
  double inRatio;         ///< ratio of input connections w.r.t full connectivity
  double outRatio;        ///< ratio of output connections w.r.t full connectivity
  double learningrate;
  /// switch on to get the internal weights and activations inspectabe
  bool   inspectInternals;
};


/**
 * class for robot control with sine, sawtooth and impuls
 *
 * period is the length of the period in steps and
 * phaseshift is the phase difference between channels given in Pi/2
 */
class ESN : public AbstractModel {
public:

  /**
     @param controlmask bitmask to select channels to control (default all)
     @param function controller function to use
   */
  ESN(const ESNConf& conf = getDefaultConf());

  static ESNConf getDefaultConf() {
    ESNConf c;
    c.numNeurons = 100;
    c.connectionStrength = 0.1;
    c.connectionRatio = 0.1;
    c.inRatio = 0.2;
    c.outRatio = 0.2;
    c.inspectInternals = false;
    return c;
  }

  /** initialisation of the network with the given number of input and output units
      @param inputDim length of input vector
      @param outputDim length of output vector
      @param unit_map if 0 the parametes are choosen randomly.
             Otherwise the model is initialised to represent a unit_map
	     with the given response strength.
      @param randGen pointer to random generator, if 0 an new one is used
   */
  virtual void init(unsigned int inputDim, unsigned  int outputDim,
		    double unit_map = 0.0, RandGen* randGen = 0);

  /** passive processing of the input
     (this function is not constant since a recurrent network
     for example might change internal states
  */
  virtual const matrix::Matrix process (const matrix::Matrix& input);

  /* performs learning and returns the network output before learning.
     Neural networks process the input before. (no need to call process before)
     \param learnRateFactor can be given to modify eps for this learning step.
  */
  virtual const matrix::Matrix learn (const matrix::Matrix& input,
				      const matrix::Matrix& nom_output,
				      double learnRateFactor = 1);

  /// damps the weights and the biases by multiplying (1-damping)
  virtual void damp(double damping);

  /// returns the number of input neurons
  virtual unsigned int getInputDim() const;
  /// returns the number of output neurons
  virtual unsigned int getOutputDim() const;

  virtual bool store(FILE*) const;

  virtual bool restore(FILE*);




protected:

  ESNConf conf;

  int nbInputs;
  int nbOutputs;
  matrix::Matrix inputWeights;
  matrix::Matrix outputWeights;
  matrix::Matrix ESNNeurons;
  matrix::Matrix ESNWeights;
  double error;

  //
};

#endif

