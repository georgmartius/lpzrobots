/*****************************************************************************
 *             FIAS winter school, playful machine group                     *
 *                Supervisors: Ralf Der, Georg Martius                       *

 * Members: Fabien Benureau, Chrisantha Fernando, Quan Wang, Jimmy Baraglia  *
 *                    Echo State Network c++ File                            *
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
 *                                                                           *
 *****************************************************************************/

#include "esn.h"
#include <selforg/controller_misc.h>
#include <selforg/matrixutils.h>

using namespace std;
using namespace matrix;

/**
 * ESN class constructor
 */
ESN::ESN(const ESNConf& _conf)
  : InvertableModel("ESN","0.1"), conf(_conf)
{

  addParameter("learningrate",&conf.learningrate,0,1,"learning rate");
  //nothing
  addInspectableMatrix("OutputWeights",&outputWeights,false,"output weights");
  addInspectableMatrix("OutputDirectWeights",&outputDirectWeights,false,"direct input to output weights");
  if(conf.inspectInternals){
    addInspectableMatrix("ESNState",&ESNState,false,"internal state");
    addInspectableMatrix("ESNWeights",&ESNWeights,false,"internal weights");
  }
  addInspectableValue("error",&error,"Learning error");
  error = 0;
  initialized = false;
}

void ESN::init(unsigned int inputDim, unsigned  int outputDim, double unit_map, RandGen* randGen)
{
  int nbInternalConnection;
  int nbInputConnectionPN;  // per neuron

  nbInputs = inputDim;
  nbOutputs = outputDim;

  nbInternalConnection = conf.numNeurons*conf.numNeurons*conf.connectionRatio;
  nbInputConnectionPN = conf.numNeurons*conf.inputRatio;

  inputWeights.set(conf.numNeurons, inputDim);
  outputWeights.set(outputDim, conf.numNeurons);
  outputDirectWeights.set(outputDim, inputDim);
  ESNState.set(conf.numNeurons, 1);
  ESNActivations.set(conf.numNeurons, 1);
  ESNWeights.set(conf.numNeurons, conf.numNeurons);
  for(int count1 = 0; count1 < nbInputs; count1++) {
    for(int count2 = 0; count2 < nbInputConnectionPN; count2++){
      int i = rand()%conf.numNeurons;
      inputWeights.val(i,count1) = random_minusone_to_one(0)*conf.inputStrength;
    }
  }
  // we may initialize the output weights with 0
  // for(int count1 = 0; count1 < nbOutputs; count1++)
  //         {
  //                 for(int count = 0; count < nbOutputConnectionPN; count++)
  //                         {
  //                                 int i = rand()%conf.numNeurons;
  //                                 outputWeights.val(count1,i) = random_minusone_to_one(0)*conf.connectionStrength;
  //                         }
  //         }

  outputDirectWeights = (outputDirectWeights^0) * unit_map;
  for(int count = 0; count < nbInternalConnection; count++) {
    int i = rand()%conf.numNeurons;
    int j = rand()%conf.numNeurons;
    ESNWeights.val(i,j) = random_minusone_to_one(0);
  }

  // calculate the eigenvalues
  Matrix real;
  Matrix img;
  assert(eigenValues(ESNWeights, real,img));
  Matrix abs = Matrix::map2(hypot, real, img); // calc absolute of complex number
  //  abs.toSort();
  // cout << (abs^T) << endl;
  ESNWeights *= conf.spectralRadius/max(abs);

  initialized = true;
}


const Matrix ESN::process (const Matrix& input)
{
  assert(initialized);
  ESNActivations = inputWeights*input+ESNWeights*ESNState;
  ESNState = ESNActivations.map(tanh);
  return outputWeights* ESNState + outputDirectWeights * input;
}

// double apply(double w ,double up){
//   if(w==0)
//     return 0;
//   else
//     return w+up;
// }

const Matrix ESN::learn (const Matrix& input, const Matrix& nom_output, double learnRateFactor)
{
  const Matrix& output = process(input);
  const Matrix& delta = nom_output - output;
  error = delta.norm_sqr();
  outputWeights       += (delta * (ESNState^T)) * (learnRateFactor * conf.learningrate);
  outputDirectWeights += (delta * (input^T)) * (learnRateFactor * conf.learningrate);
  // // if we want to force a sparse output connections we could restrict them here: (initialization matters!)
  // const Matrix& update = (delta * (ESNState^T)) * (learnRateFactor * conf.learningrate);
  // outputWeights.toMap2(apply,update);
  return output;
}

/*
  online learning from matlab toolbox
  case 'online'
  nSampleInput = length(trainInput);
  stateCollection = zeros(nSampleInput, trained_esn.nInternalUnits + trained_esn.nInputUnits);
  SInverse = 1 / trained_esn.RLS_delta * eye(trained_esn.nInternalUnits + trained_esn.nInputUnits) ;
  totalstate = zeros(trained_esn.nTotalUnits,1);
  internalState = zeros(trained_esn.nInternalUnits,1) ;
  error = zeros(nSampleInput , 1) ;
  weights = zeros(nSampleInput , 1) ;
  for iInput = 1 : nSampleInput
  if trained_esn.nInputUnits > 0
  in = [diag(trained_esn.inputScaling) * trainInput(iInput,:)' + esn.inputShift];  % in is column vector
  else in = [];
  end

  %write input into totalstate
  if esn.nInputUnits > 0
  totalstate(esn.nInternalUnits+1:esn.nInternalUnits+esn.nInputUnits) = in;
  end

  % update totalstate except at input positions

  % the internal state is computed based on the type of the network
  switch esn.type
  case 'plain_esn'
  typeSpecificArg = [];
  case 'leaky_esn'
  typeSpecificArg = [];
  case 'twi_esn'
  if  esn.nInputUnits == 0
  error('twi_esn cannot be used without input to ESN');
  end
  typeSpecificArg = esn.avDist;
  end
  internalState = feval(trained_esn.type , totalstate, trained_esn, typeSpecificArg ) ;
  netOut = feval(trained_esn.outputActivationFunction,trained_esn.outputWeights*[internalState;in]);
  totalstate = [internalState;in;netOut];
  state = [internalState;in] ;
  stateCollection(iInput, :) = state';
  phi = state' * SInverse ;
  %            u = SInverse * state ;
  %            k = 1 / (lambda + state'*u)*u ;
  k = phi'/(trained_esn.RLS_lambda + phi * state );
  e = trained_esn.teacherScaling * trainOutput(iInput,1) + trained_esn.teacherShift - netOut(1) ;
  % collect the error that will be plotted
  error(iInput , 1 ) = e*e ;
  % update the weights
  trained_esn.outputWeights(1,:) = trained_esn.outputWeights(1,:) + (k*e)' ;
  % collect the weights for plotting
  weights(iInput , 1) = sum(abs(trained_esn.outputWeights(1,:))) ;
  %            SInverse = 1 / lambda * (SInverse - k*(state' * SInverse)) ;
  SInverse = ( SInverse - k * phi ) / trained_esn.RLS_lambda ;
  end


*/

const Matrix ESN::response(const matrix::Matrix& _ignored) const {
  const Matrix& g_prime = ESNActivations.map(tanh_prime);
  return outputWeights * (inputWeights & g_prime) + outputDirectWeights;
}

const Matrix ESN::inversion(const matrix::Matrix& input, const matrix::Matrix& xsi) const {
  // todo
  return Matrix();
}


void ESN::damp(double damping)//Damp is Dumb
{
}

unsigned int ESN::getInputDim() const
{
  return inputWeights.getM();
}

unsigned int ESN::getOutputDim() const
{
  return outputWeights.getN();
}


bool ESN::store(FILE* f) const{
  // save matrix values

  inputWeights.store(f);
  outputWeights.store(f);
  outputDirectWeights.store(f);
  ESNWeights.store(f);
  ESNState.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the ESN values from a given file. */
bool ESN::restore(FILE* f){
  // save matrix values
  inputWeights.restore(f);
  outputWeights.restore(f);
  outputDirectWeights.restore(f);
  ESNWeights.restore(f);
  ESNState.restore(f);
  Configurable::parse(f);
  return true;
}
