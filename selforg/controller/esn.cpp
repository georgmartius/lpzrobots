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

using namespace std;
using namespace matrix;

  /**
   * ESN class constructor
  */
  ESN::ESN(const ESNConf& _conf)
    :AbstractModel("ESN","0.1"), conf(_conf)
  {

    addParameter("learningrate",&conf.learningrate,0,1,"learning rate");
    //nothing
    addInspectableMatrix("OutputWeights",&outputWeights,false,"output weights");
    if(conf.inspectInternals){
      addInspectableMatrix("ESNNeurons",&ESNNeurons,false,"internal state");
      addInspectableMatrix("ESNWeights",&ESNWeights,false,"internal weights");
    }
    addInspectableValue("error",&error,"Learning error");
    error = 0;
  }

  void ESN::init(unsigned int inputDim, unsigned  int outputDim, double unit_map, RandGen* randGen)
  {
	int nbInternalConnection;
	int nbInputConnectionPN;  // per neuron
	int nbOutputConnectionPN; // per neuron

	nbInputs = inputDim;
	nbOutputs = outputDim;

	nbInternalConnection = conf.numNeurons*conf.numNeurons*conf.connectionRatio;
	nbInputConnectionPN = conf.numNeurons*conf.inRatio;
	nbOutputConnectionPN = conf.numNeurons*conf.outRatio;

	inputWeights.set(conf.numNeurons,inputDim);
	outputWeights.set(outputDim,conf.numNeurons);
	ESNNeurons.set(conf.numNeurons,1);
	ESNWeights.set(conf.numNeurons,conf.numNeurons);
	for(int count1 = 0; count1 < nbInputs; count1++)
	{
		for(int count2 = 0; count2 < nbInputConnectionPN; count2++)
		{
			int i = rand()%conf.numNeurons;
			inputWeights.val(i,count1) = random_minusone_to_one(0)*conf.connectionStrength;
		}
	}

	for(int count1 = 0; count1 < nbOutputs; count1++)
	{
		for(int count = 0; count < nbOutputConnectionPN; count++)
		{
			int i = rand()%conf.numNeurons;
			outputWeights.val(count1,i) = random_minusone_to_one(0)*conf.connectionStrength;
		}
	}

	for(int count = 0; count < nbInternalConnection; count++)
	{
		int i = rand()%conf.numNeurons;
		int j = rand()%conf.numNeurons;
		ESNWeights.val(i,j) = random_minusone_to_one(0)*conf.connectionStrength;
	}

  }


  const Matrix ESN::process (const Matrix& input)
  {
	ESNNeurons = (inputWeights*input+ESNWeights*ESNNeurons).map(tanh);
	return outputWeights* ESNNeurons;
  }


  const Matrix ESN::learn (const Matrix& input, const Matrix& nom_output, double learnRateFactor)
  {
	const Matrix& output = process(input);
	const Matrix& delta = nom_output - output;
	error = delta.norm_sqr();
	outputWeights += (delta * (ESNNeurons^T)) * (learnRateFactor * conf.learningrate);
	/// if we want to force a sparse output connections we would need to
	/// restrict the outputWeights here.
	return output;
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
    outputWeights.store(f);
    ESNWeights.store(f);
    Configurable::print(f,0);
    return true;
  }

  /* loads the ESN values from a given file. */
  bool ESN::restore(FILE* f){
    // save matrix values
    inputWeights.restore(f);
    outputWeights.restore(f);
    outputWeights.restore(f);
    ESNWeights.restore(f);
    Configurable::parse(f);
    return true;
  }
