#ifndef __ONELAYERFFNN_H
#define __ONELAYERFFNN_H

#include "feedforwardnn.h"
#include "randomgenerator.h"

/// simple one layer neural network with configurable activation function
class OneLayerFFNN : public FeedForwardNN {
public:
  /**
     Uses linear activation function
     @param eps learning rate
     @param factor_bias learning rate factor for bias learning
  */
  OneLayerFFNN(double eps,
               double factor_bias = 0.1,
               const std::string& name = "OneLayerFFN",
               const std::string& revision = "$Id: onelayerffnn.h,v 1.10 2011/05/30 13:52:54 martius Exp $")
    : FeedForwardNN(name, revision), eps(eps), factor_bias(factor_bias){
    actfun  = linear;
    dactfun = dlinear;
    addParameter("eps",&this->eps,0,1,"learning rate");
    addParameter("factor_bias",&this->factor_bias,0,2,"factor for learningrate of bias");
    initialised = false;
  }
  /**
     @param eps learning rate
     @param factor_bias learning rate factor for bias learning
     @param actfun callback activation function (see FeedForwardNN)
     @param dactfun callback for first derivative of the activation function
  */
  OneLayerFFNN(double eps,
               double factor_bias,
               ActivationFunction actfun,
               ActivationFunction dactfun,
               const std::string& name = "OneLayerFFN",
               const std::string& revision = "$Id: onelayerffnn.h,v 1.10 2011/05/30 13:52:54 martius Exp $")
    : FeedForwardNN(name, revision), eps(eps), factor_bias(factor_bias), actfun(actfun), dactfun(dactfun) {
    addParameter("eps",&this->eps,0,1,"learning rate");
    addParameter("factor_bias",&this->factor_bias,0,2,"factor for learningrate of bias");
    initialised = false;
  }
  virtual ~OneLayerFFNN(){ }

  /* initialisation of the network with the given number of input and output units
     @param unit_map defines the approximate response of the network
       after initialisation (if unit_map=1 the weights are unit matrices).
     @param randGen pointer to random generator, if 0 an new one is used
  */
  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0);

  virtual const matrix::Matrix process (const matrix::Matrix& input);

  virtual const matrix::Matrix learn (const matrix::Matrix& input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1);

  /// returns the number of input neurons
  virtual unsigned int getInputDim() const {
    return weights.getN();
  }
  /// returns the number of output neurons
  virtual unsigned int getOutputDim() const {
    return weights.getM();
  }

  virtual const matrix::Matrix& getWeights() const { return weights; }
  virtual const matrix::Matrix& getBias()    const { return bias; }

  /// damps the weights and the biases by multiplying (1-damping)
  virtual void damp(double damping){
    weights *= (1-damping);
    bias    *= (1-damping);
  }

  /**************  STOREABLE **********************************/
  /// stores the layer binary into file stream
  bool store(FILE* f) const;
  /// restores the layer binary from file stream
  bool restore(FILE* f);


private:
  double eps;
  double factor_bias;
  ActivationFunction actfun;  ///< callback activation function
  ActivationFunction dactfun; ///< first derivative of the activation function

private:
  bool initialised;
  matrix::Matrix weights;
  matrix::Matrix bias;
};

#endif
