
#include "onelayerffnn.h"
#include "controller_misc.h"

using namespace matrix;

/// initialisation of the network with the given number of input and output units
void OneLayerFFNN::init(unsigned int inputDim, unsigned  int outputDim,
                        double unit_map, RandGen* randGen) {
  if(!randGen) randGen = new RandGen(); // this give a memory leak!
  weights.set(outputDim, inputDim);
  weights = (weights^0)*unit_map;
  weights+=weights.mapP(randGen, random_minusone_to_one) * (unit_map * 0.05 + 0.05);

  bias.set(outputDim, 1);
  initialised = true;
}

/// passive processing of the input
const Matrix OneLayerFFNN::process (const Matrix& input) {
  assert(initialised);
  return (weights * input + bias).map(actfun);
}
/// performs learning and returns the network output before learning
const Matrix OneLayerFFNN::learn (const Matrix& input,
                                  const Matrix& nom_output,
                                  double learnRateFactor) {
  assert(initialised);
  double epsilon = eps*learnRateFactor;
  const Matrix& z       = weights * input + bias;
  const Matrix& g_prime = z.map(dactfun);
  const Matrix& output  = z.map(actfun);
  const Matrix& xsi     = nom_output - output;

  weights += xsi.multrowwise(g_prime) * (input^T) * epsilon;
  bias    += xsi.multrowwise(g_prime) * epsilon * factor_bias;
  return output;
}


bool OneLayerFFNN::store(FILE* f) const {
        fprintf(f,"%g\n", eps);
        weights.store(f);
        bias.store(f);
        return true;
}

bool OneLayerFFNN::restore(FILE* f){
        char buffer[128];
        if(fscanf(f,"%s\n", buffer) != 1) return false;
        eps = atof(buffer);
        if(!weights.restore(f)) return false;
        if(!bias.restore(f)) return false;
        initialised = true;
        return true;
}
