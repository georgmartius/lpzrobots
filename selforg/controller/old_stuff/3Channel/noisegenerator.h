#ifndef __NOISEGENERATOR_H
#define __NOISEGENERATOR_H



#include <map>

template<int NUMBER_CHANNELS> class NoiseGenerator{
#define PI_NOISEGENERATOR  3.1415927

protected:
  double uniform_mean[NUMBER_CHANNELS];  // storage for adding colored uniformly distributed noise to values
  double normal_mean[NUMBER_CHANNELS];   // storage for adding colored normally distributed noise to values  

  double uniform_mean1channel;  // storage for generating uniformly distributed random numbers
  double normal_mean1channel;   // storage for generating normally distributed random numbers
  
  double tau_uniform; // smoothing paramter for uniformly distibuted random numbers
  double tau_normal;  // smoothing paramter for normally distibuted random numbers  

public:  
  NoiseGenerator(double tau_uniform=0.3, double tau_normal=0.3):tau_uniform(tau_uniform), tau_normal(tau_normal){

    // initialize random number generator
    srand(time(0));

    for (int i=0; i<NUMBER_CHANNELS; i++){
      uniform_mean[i]=0.0;
      normal_mean[i]=0.0;    
    }  
    uniform_mean1channel=0.0;
    normal_mean1channel=0.0;
  };

  //generate white (no averaging) uniformly distributed random number between "min" and "max"  
  double generateWhiteUniformlyDistributedRandomNumber(double min=-0.1, double max=0.1){
    return( (double(rand())/RAND_MAX)*(max-min)+min );
  };
  
  //generate colored (averaging) uniformly distributed random number between "min" and "max"  
  //! valid only for ONE random number, use addColoredUniformlyDistributedNoise(...) for 
  //! adding this kind of noise to several chanels
  double generateColoredUniformlyDistributedRandomNumber(double min=-0.1, double max=0.1){
    uniform_mean1channel+=tau_uniform*(generateWhiteUniformlyDistributedRandomNumber(min,  max) - uniform_mean1channel);
    return(uniform_mean1channel);
  };


  // generate white (no averaging) normally distributed random number with variance "variance" and mean "mean"  
  double generateWhiteNormallyDistributedRandomNumber(double variance=0.05, double mean=0.0){
    double x1=generateWhiteUniformlyDistributedRandomNumber(0, 1);
    double x2=generateWhiteUniformlyDistributedRandomNumber(0, 1);
    //return( (sqrt(-2*ln(x1)) *cos(2*PI_NOISEGENERATOR*x2))  * variance +mean) ; 
    return( (sqrt(-2*log(x1)) *cos(2*PI_NOISEGENERATOR*x2))  * variance +mean) ; 
  };
    
  // generate colored (averaging) normally distributed random number with variance "variance" and mean "mean"  
  //! valid only for ONE random number, use addColoredNormallyDistributedNoise(...) for 
  //! adding this kind of noise to several chanels
  double generateColoredNormallyDistributedRandomNumber(double variance=0.05, double mean=0.0){
    normal_mean1channel+=tau_normal*(generateWhiteNormallyDistributedRandomNumber(variance, mean) - normal_mean1channel);
    return(normal_mean1channel);
  };
  
  //add white (no averaging) uniformly distributed noise between "min" and "max" to the elements of "value"
  void addWhiteUniformlyDistributedNoise(double *value, double min=-0.1, double max=0.1){
    for(int i=0; i<NUMBER_CHANNELS; i++){
      value[i]+=generateWhiteUniformlyDistributedRandomNumber(max, min);
    }  
  };

  //add colored (averaging) uniformly distributed noise between "min" and "max" to the elements of "value"  
  void addColoredUniformlyDistributedNoise(double *value, double min=-0.1, double max=0.1){
    for(int i=0; i<NUMBER_CHANNELS; i++){
      uniform_mean[i]+=tau_uniform*(generateWhiteUniformlyDistributedRandomNumber(min,  max) - uniform_mean[i]);
      value[i]+=uniform_mean[i];
    }  
  };
  
  //add white (no averaging) normally distributed noise with variance "variance" and mean "mean" to the elements of "value"
  void addWhiteNormallyDistributedNoise(double *value, double variance=0.05, double mean=0.0){
    for(int i=0; i<NUMBER_CHANNELS; i++){
      value[i]+=generateWhiteNormallyDistributedRandomNumber(variance, mean);
    }
  };

  //add colored (averaging) normally distributed noise with variance "variance" and mean "mean" to the elements of "value"  
  void addColoredNormallyDistributedNoise(double *value, double variance=0.05, double mean=0.0){
    for (int i=0; i<NUMBER_CHANNELS; i++){
    normal_mean[i]+=tau_normal*(generateWhiteNormallyDistributedRandomNumber(variance, mean) - normal_mean[i]);
    value[i]+=normal_mean[i];
    }
  };



};
#endif
