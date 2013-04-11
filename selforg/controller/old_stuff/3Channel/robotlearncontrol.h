#include <cassert>
#include <cstdlib>
#include <iostream>


template <int NUMBER_CHANNELS, int BUFFER_SIZE=1> class RobotLearnControl{

protected:

  double A[NUMBER_CHANNELS][NUMBER_CHANNELS];    ///< model matrix
  double C[NUMBER_CHANNELS][NUMBER_CHANNELS];    ///< controller matrix
                                                 // Empfänger-Sender-Schweibweise
  double h[NUMBER_CHANNELS];       ///< bias vector
  double eps;        ///< learn rate
  double rho;        ///< regularisation
  double x_buffer[BUFFER_SIZE][NUMBER_CHANNELS]; ///< buffer for input values, x[t%buffersize]=actual value, x[(t-1+buffersize)%buffersize]=x(t-1)
  double y_buffer[BUFFER_SIZE][NUMBER_CHANNELS]; ///< buffer for output values, y[t%buffersize]=actual value(if already calculated!), y[(t-1+buffersize)%buffersize]=y(t-1)

  double delta;   ///< delta
  int    number_steps_of_delay;     ///< number of steps for delay
  int    number_steps_for_averaging;    ///< number of steps for averaging when calculating x_effective and y_effective
  int    t;       ///< number of steps, needed for ringbuffer x_buffer
  double m;  ///< factor between E and E_s
  double a_factor;  ///< additional factor for learning rate of model parameters


  /**
   * calculate inverse matrix Q_1 of matrix Q. only for NUMBER_CHANNELS<4 now!
   */
  virtual void inverseMatrix(double Q[NUMBER_CHANNELS][NUMBER_CHANNELS], double Q_1[NUMBER_CHANNELS][NUMBER_CHANNELS])
  {
    // Berechne Inverse von Q

    // only if NUMBER_CHANNELS<4
    assert(NUMBER_CHANNELS<4);
    if (NUMBER_CHANNELS==2){

      double det = Q[0][0] * Q[1][1] - Q[0][1] * Q[1][0];
      Q_1[0][0] = Q[1][1] / det;
      Q_1[1][1] = Q[0][0] / det;
      Q_1[0][1] = -Q[0][1] / det;
      Q_1[1][0] = -Q[1][0] / det;

    }


    if (NUMBER_CHANNELS==3){

      double  Q_adjoint[NUMBER_CHANNELS][NUMBER_CHANNELS]  ;
      double  detQ=0  ;

      //calculate the inverse of Q
      Q_adjoint[0][0]=Q[1][1]*Q[2][2]-Q[1][2]*Q[2][1] ;
      Q_adjoint[0][1]=(Q[1][2]*Q[2][0]-Q[1][0]*Q[2][2]) ;
      Q_adjoint[0][2]=Q[1][0]*Q[2][1]-Q[1][1]*Q[2][0] ;
      Q_adjoint[1][0]=(Q[2][1]*Q[0][2]-Q[0][1]*Q[2][2]) ;
      Q_adjoint[1][1]=Q[0][0]*Q[2][2]-Q[0][2]*Q[2][0] ;
      Q_adjoint[1][2]=(Q[0][1]*Q[2][0]-Q[0][0]*Q[2][1]) ;
      Q_adjoint[2][0]=Q[0][1]*Q[1][2]-Q[1][1]*Q[0][2] ;
      Q_adjoint[2][1]=(Q[1][0]*Q[0][2]-Q[0][0]*Q[1][2]) ;
      Q_adjoint[2][2]=Q[0][0]*Q[1][1]-Q[0][1]*Q[1][0] ;
      detQ=Q[0][0]*Q_adjoint[0][0]+Q[0][1]*Q_adjoint[0][1]+Q[0][2]*Q_adjoint[0][2] ;
      for(int i=0; i<NUMBER_CHANNELS; i++){
        for(int j=0; j<NUMBER_CHANNELS; j++) {
          Q_1[i][j]=(Q_adjoint[j][i])/detQ  ;
        }
      }
    }

  };


  /// calculate E
  virtual double calculateE(double *x_delay, double *y_delay)
  {
    double L[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double Q[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double Q_1[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double z[NUMBER_CHANNELS];
    double xsi[NUMBER_CHANNELS];


    // Calculate z based on the delayed inputs since the present input x is
    // produced by the outputs tau time steps before
    // which on their hand are y = K(x_D)
    // due to the delay in the feed back loop.

    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      z[i] = h[i];
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        z[i] += C[i][j] * x_delay[j];
      }
    }

    // Berechne Matrix L
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        L[i][j] = 0.0;
        for (int k = 0; k < NUMBER_CHANNELS; k++)
        {
          L[i][j] += A[i][k] * g_s(z[k]) * C[k][j];
        }
      }
    }

    // Berechne Q=LL^T
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        Q[i][j] = 0.0;
        for (int k = 0; k < NUMBER_CHANNELS; k++)
        {
          Q[i][j] += L[i][k] * L[j][k];
        }
        if (i == j)
          Q[i][j] += rho / NUMBER_CHANNELS; // Regularisation
      }
    }

    // Berechne Inverse von Q
    inverseMatrix(Q, Q_1);

    // Berechne xsi
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      xsi[i] = x_buffer[(t+BUFFER_SIZE)%BUFFER_SIZE][i];
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        xsi[i] -= A[i][j] * y_delay[j];  // using old y value -> no influence of changes (adding delta) in controller
        //xsi[i] -= A[i][j] * g(z[j]);     // using recalculating y -> influence of changes (adding delta) in controller
      }
    }
    double E = 0;
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        E += xsi[i] * Q_1[i][j] * xsi[j];
      }
    }

    double E_s=0;
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        E_s += (A[i][j]*g(z[j]) - x_buffer[(t+BUFFER_SIZE)%BUFFER_SIZE][i]) * (A[i][j]*g(z[j]) - x_buffer[(t+BUFFER_SIZE)%BUFFER_SIZE][i]);
      }
    }

    E=(1-m)*E+ m*E_s;
    return (E);

  };



  /// learn values h,C   //,A
  virtual void learn(double *x_delay, double *y_delay)
  {
    double A_update[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double C_update[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double h_update[NUMBER_CHANNELS];

    double E_0 = calculateE(x_delay, y_delay);

    // calculate updates for h,C,A
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      h[i] += delta;
      h_update[i] = -eps * (calculateE(x_delay, y_delay) - E_0) / delta;
      h[i] -= delta;
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        C[i][j] += delta;
        C_update[i][j] = -eps * (calculateE(x_delay, y_delay) - E_0) / delta;
        C[i][j] -= delta;
        //A[i][j] += delta;
        //A_update[i][j] = -eps * a_factor * (calculateE(x_delay, y_delay) - E_0) / delta;
        //A[i][j] -= delta;
      }
    }

    // apply updates to h,C,A
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      h[i] += squash(h_update[i]);
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        C[i][j] += squash(C_update[i][j]);
        //A[i][j] += squash(A_update[i][j]);
      }
    }
  };

  /// learn model parameter (matrix A) by gradient descent
  virtual void learnModel(double *x_actual, double *y_effective){
    /*         double z[N_output];
        for(int i=0; i<N_output; i++){
            z[i]=h[i];
            for(int j=0; j<N_input; j++) {
                z[i]+=C[i][j]*x_D[j];
            }
        }
      */
    double xsi[NUMBER_CHANNELS];
    // Berechne xsi
    for(int i=0; i<NUMBER_CHANNELS; i++){
      xsi[i]=x_actual[i];
      for(int j=0; j<NUMBER_CHANNELS; j++){
        xsi[i]-= A[i][j]*y_effective[j];
      }
    }

    for(int i=0; i<NUMBER_CHANNELS; i++){
      for (int j=0; j<NUMBER_CHANNELS; j++){
        A[i][j]+=squash( (a_factor*eps*0.2) *xsi[i] * y_effective[j]) ;
      }
    }
  };



  /// calculate delayed values
  virtual void calculateDelayedValues(double source[BUFFER_SIZE][NUMBER_CHANNELS], int number_steps_of_delay_, double *target)
  {
    // number_steps_of_delay must not be larger than BUFFER_SIZE
    assert (number_steps_of_delay_ < BUFFER_SIZE);

    // get delayed value from ring buffer

    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      target[i] = source[(t - number_steps_of_delay_ + BUFFER_SIZE) % BUFFER_SIZE][i];
    }
  };

  virtual void calculateSmoothValues(double source[BUFFER_SIZE][NUMBER_CHANNELS], int number_steps_for_averaging_, double *target)
  {
    // number_steps_for_averaging must not be larger than BUFFER_SIZE
    assert (number_steps_for_averaging_ <= BUFFER_SIZE);

    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      target[i] = 0.0;
      for (int k = 0; k < number_steps_for_averaging_; k++)
      {
        target[i] += source[(t - k + BUFFER_SIZE) % BUFFER_SIZE][i]/ (double) (number_steps_for_averaging);
      }
    }
  };



  /// calculate controller ouptus
  virtual void calculateControllerValues(double *x_smooth, double *y)
  {
    double z[NUMBER_CHANNELS];

    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      z[i] = h[i];
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        z[i] += C[i][j] * x_smooth[j];
      }
      y[i] = g(z[i]);
    }
  };


  // put new value in ring buffer
  virtual void putInBuffer(double buffer[BUFFER_SIZE][NUMBER_CHANNELS], double *values){
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      buffer[(t+BUFFER_SIZE)% BUFFER_SIZE][i] = values[i];
    }
  };





  /// neuron transfer function
  virtual double g(double z)
  {
    return tanh(z);
  };



  /// derivation of neuron transfer function
  virtual double g_s(double z)
  {
    return 1.0 - tanh(z) * tanh(z);
  };



  /// squashing function, to protect against to large weight updates
  virtual double squash(double z)
  {
    return 0.1 * tanh(10.0 * z);
  };




public:
  // when using RobotLearnControl_Gnu: remeber the default values in it's constuctor
  RobotLearnControl(double eps=0.7,double rho=0.0, int number_steps_of_delay_=3):
  eps(eps), rho(rho), number_steps_of_delay(number_steps_of_delay_), delta(0.01), number_steps_for_averaging(1), t(0), m(0.0), a_factor (1.0)
  {

    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      h[i] = 0.0;
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        if (i == j)
        {
          A[i][j] = 1.0;
          C[i][j] = 1.0;
        }
        else
        {
          A[i][j] = 0.0;
          C[i][j] = 0.0;
        }
      }
    }
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      for (int k = 0; k < BUFFER_SIZE; k++)
      {
        x_buffer[k][i] = 0;
        y_buffer[k][i] = 0;
      }
    }

    // print initial values
    std::cout<<"Constructor of RobotLearnControl:"<<std::endl;
    std::cout<<"init: epsilon="<<eps<<std::endl;
    std::cout<<"init: rho="<<rho<<std::endl;
    std::cout<<"init: number_steps_of_delay="<<number_steps_of_delay<<std::endl;
    std::cout<<"init: number_steps_for_averaging="<<number_steps_for_averaging<<std::endl;
    std::cout<<"init: delta="<<delta<<std::endl;
    std::cout<<"init: m (for calculation of E)="<<m<<std::endl;

    // initialize random number generator
    srand(time(0));
  };


  /// make step (calculate controller outputs and learn controller)
  virtual void makeStep(double *x_, double *y_)
  {
    double x_smooth[NUMBER_CHANNELS];
    double x_effective[NUMBER_CHANNELS];
    double y_effective[NUMBER_CHANNELS];
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      x_smooth[i] = 0.0;
      x_effective[i] = 0.0;
      y_effective[i] = 0.0;
    }


    // put new input value in ring buffer x_buffer
    putInBuffer(x_buffer, x_);

    // averaging over the last number_steps_for_averaging values of x_buffer
    calculateSmoothValues(x_buffer, number_steps_for_averaging, x_smooth);

    // calculate controller values based on smoothed input values
    calculateControllerValues(x_smooth, y_);

    // put new output value in ring buffer y_buffer
    putInBuffer(y_buffer, y_);

    // calculate effective input/output, which is (actual-number_steps_of_delay) element of buffer
    calculateDelayedValues(x_buffer, number_steps_of_delay, x_effective);
    calculateDelayedValues(y_buffer, number_steps_of_delay, y_effective);

    // learn controller with effective input/output
    learn(x_effective, y_effective);

    // learn model with actual input and effective output (that produced the actual input)
    learnModel(x_buffer[(t+BUFFER_SIZE)%BUFFER_SIZE], y_effective);

    // update step counter
    t++;
  };

  /// make step without learning (only calculate controller outputs)
  virtual void makeStepWithoutLearning(double *x_, double *y_)
  {
    double x_smooth[NUMBER_CHANNELS];
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      x_smooth[i] = 0.0;
    }


    // put new input value in ring buffer x_buffer
    putInBuffer(x_buffer, x_);

    // averaging over the last number_steps_for_averaging values of x_buffer
    calculateSmoothValues(x_buffer, number_steps_for_averaging, x_smooth);

    // calculate controller values based on smoothed input values
    calculateControllerValues(x_smooth, y_);

    // put new output value in ring buffer y_buffer
    putInBuffer(y_buffer, y_);

    // update step counter
    t++;
  };

  /// put x and y in all places in the x- and y-buffer
  virtual void putInWholeBuffers(double *x_, double *y_)
  {
    for (int tt=0; tt<BUFFER_SIZE; tt++){
      for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        x_buffer[tt][i] = x_[i];
      }
      for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        y_buffer[tt][i] = y_[i];
      }
    }

  };

  // put x and y at the actual position in the buffer
  virtual void putInBuffers(double *x_, double*y_)
  {
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      x_buffer[(t+BUFFER_SIZE)% BUFFER_SIZE][i] = x_[i];
    }
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      y_buffer[(t+BUFFER_SIZE)% BUFFER_SIZE][i] = y_[i];
    }
  };

  virtual void increaseStepCounter()
  {
    t++;
  };

  virtual void setEps(double x)
  {
    eps = x;
  };
  virtual double getEps()
  {
    return (eps);
  };

  virtual void setRho(double x)
  {
    rho = x;
  };
  virtual double getRho()
  {
    return (rho);
  };

  virtual void setDelta(double x)
  {
    delta = x;
  };
  virtual double getDelta()
  {
    return (delta);
  };

  virtual void setNumberStepsForAveraging(int x)
  {
    number_steps_for_averaging = x;
  };
  virtual int getNumberStepsForAveraging()
  {
    return (number_steps_for_averaging);
  };

  virtual void setNumberStepsOfDelay(int x)
  {
    number_steps_of_delay = x;
  };
  virtual int getNumberStepsOfDelay()
  {
    return (number_steps_of_delay);
  };


  virtual void setM(double x)
  {
    m = x;
  };
  virtual double getM()
  {
    return (m);
  };

  virtual void setAFactor(double x)
  {
    a_factor = x;
  };
  virtual double getAFactor()
  {
    return (a_factor);
  };

};









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
