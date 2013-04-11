#include <cassert>


template <int NUMBER_CHANNELS, int BUFFER_SIZE=2> class RobotLearnControl{

protected:
  double epsilon_it ;
  int  number_it   ;
  double L[NUMBER_CHANNELS][NUMBER_CHANNELS];
  double detC ;
  double detA ;
  double  detL  ;
  double A[NUMBER_CHANNELS][NUMBER_CHANNELS];    ///< model matrix
  double C[NUMBER_CHANNELS][NUMBER_CHANNELS];    ///< controller matrix
  double h[NUMBER_CHANNELS];       ///< bias vector
  double eps;        ///< learn rate
  double rho;        ///< regularisation
  double x_buffer[BUFFER_SIZE][NUMBER_CHANNELS]; ///< buffer for input values, x[t%buffersize]=actual value, x[(t-1+buffersize)%buffersize]=x(t-1)
  double y_buffer[BUFFER_SIZE][NUMBER_CHANNELS]; ///< buffer for output values, y[t%buffersize]=actual value(if already calculated!), y[(t-1+buffersize)%buffersize]=y(t-1)

  double delta;   ///< delta
  int    number_steps_of_delay;     ///< number of steps for delay
  int    number_steps_for_averaging;    ///< number of steps for averaging when calculating x_effective
  int    t;       ///< number of steps, needed for ringbuffer x_buffer
  double m;  ///< factor between E and E_s
  double factor_a  ;

    /*double Determinant(double L[3][3] )
     {
      double  det_L=0.0  ;
     det_L=L[0][0]*L[1][1]*L[2][2]-L[0][2]*L[1][1]*L[2][0]+L[0][1]*L[1][2]*L[2][0]
     -L[0][0]*L[1][2]*L[2][1]+L[0][2]*L[1][0]*L[2][1]-L[0][1]*L[1][0]*L[2][2] ;
      return det_L  ;
     }*/







  /**
   * calculate inverse matrix Q_1 of matrix Q. only for NUMBER_CHANNELS<4 now!
   */
 /* virtual void inverseMatrix(double Q[NUMBER_CHANNELS][NUMBER_CHANNELS], double Q_1[NUMBER_CHANNELS][NUMBER_CHANNELS])
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

  };*/



  /// calculate E
  /*virtual double calculateE(double *x_delay, double *y_delay)
  {
    //double L[NUMBER_CHANNELS][NUMBER_CHANNELS];
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
        //xsi[i] -= A[i][j] * y_delay[j];
        xsi[i] -= A[i][j] * g(z[j]);
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

  };*/



  /// learn values h,C,A
  /*virtual void learn(double *x_delay, double *y_delay)
  {
    //double A_update[NUMBER_CHANNELS][NUMBER_CHANNELS];
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
        //A_update[i][j] = -eps * (calculateE(x_delay, y_delay) - E_0) / delta;
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

    };*/





     // Berechne xsi

   /*virtual void xsi_calculate( double x_buffer,double  y_delay,double *xsi ){

    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      xsi[i] = x_buffer[(t+BUFFER_SIZE)%BUFFER_SIZE][i];
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        xsi[i] -= A[i][j] * y_delay[j];

      }
    }

   };*/


   virtual void iteration(double *colmn,double dommy[NUMBER_CHANNELS][NUMBER_CHANNELS],double *improvment){
     double sum[NUMBER_CHANNELS]  ;
     double norm=0.0 ;


     for (int k= 0; k< NUMBER_CHANNELS; k++)
         {
           norm+=colmn[k]*colmn[k]  ;
           }
         norm=sqrt(norm)   ;
   for(int t = 0; t <number_it ; t++)
    {

     //initialization
      if(t==0)
        {

         for (int i = 0; i < NUMBER_CHANNELS; i++)
         {
           improvment[i]=0.0 ;

         }
        }


        for (int k= 0; k< NUMBER_CHANNELS; k++)
         {
             sum[k]=colmn[k]/norm ;

             for (int l= 0; l<NUMBER_CHANNELS; l++)
              {
                sum[k]-=dommy[k][l]*improvment[l]  ;
                }
           }


          for (int j = 0; j< NUMBER_CHANNELS; j++){
            for (int i = 0; i < NUMBER_CHANNELS; i++){
              improvment[j]+=epsilon_it*dommy[i][j]*sum[i]       ;
            }
          }

      }//endof-t-loop


      for (int j = 0; j< NUMBER_CHANNELS; j++){
         improvment[j]*=norm  ;
        }


    };

   virtual  double calculateE(double *x_delay,double *h,double *y_delay, double *eita_sup){
     double eita[NUMBER_CHANNELS]  ;
     double eita_zero[NUMBER_CHANNELS]  ;
     double xsi[NUMBER_CHANNELS]  ;
     double  shift_value[NUMBER_CHANNELS]  ;
     //double  xsi_norm[NUMBER_CHANNELS]  ;
     double  sum=0.0  ;
     for (int i = 0; i < NUMBER_CHANNELS; i++)
      {
        eita[i]=0.0  ;
        shift_value[i]=0.0 ;
        }

    // Calculate z based on the delayed inputs since the present input x is
    // produced by the outputs tau time steps before
    // which on their hand are y = K(x_D)
    // due to the delay in the feed back loop.
    double z[NUMBER_CHANNELS];

    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      z[i] = h[i];
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        z[i] += C[i][j] * x_delay[j];
      }
    }



    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      xsi[i] = x_buffer[(t+BUFFER_SIZE)%BUFFER_SIZE][i];
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
//        xsi[i] -= A[i][j] * y_delay[j];
        xsi[i] -= A[i][j] * g(z[j]);
      }
    }
    /*for (int i = 0; i < NUMBER_CHANNELS; i++){
      sum+=xsi[i]  ;
      }
    for (int i = 0; i < NUMBER_CHANNELS; i++){
      xsi_norm[i]=xsi[i]/sum  ;  ;
      }*/



/*
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      z[i] = h[i];
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        z[i] += C[i][j] *x_buffer[(t-1+BUFFER_SIZE)%BUFFER_SIZE][j];
      }
      //y[i] = g(z[i]);
    }
*/


    iteration(xsi,A,eita_zero)  ;
    for (int i = 0; i < NUMBER_CHANNELS; i++)
     {
       eita[i]=(1/(g_s(z[i])))*eita_zero[i]  ;
       }

    iteration(eita,C,shift_value) ;
    double E=0.0  ;
    for (int i=0;i<NUMBER_CHANNELS;i++)
       {
         E+=shift_value[i]*shift_value[i]   ;
         }
    for (int i = 0; i < NUMBER_CHANNELS; i++)
     {
       eita_sup[i]=eita[i]  ;
       }


    // Berechnung des z mit aktuellem Sensorwert
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      z[i] = h[i];
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        z[i] += C[i][j] *x_buffer[(t+BUFFER_SIZE)%BUFFER_SIZE][j];
      }
      //y[i] = g(z[i]);
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

     return E  ;

   };

  /// learn values h,C,A
  virtual void learn(double *x_delay, double *y_delay)
  {
    //double A_update[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double C_update[NUMBER_CHANNELS][NUMBER_CHANNELS];
    double h_update[NUMBER_CHANNELS];
    double eita[NUMBER_CHANNELS]  ;
    //eita[]=0

    double E_0 = calculateE(x_delay,h, y_delay, eita);


    // calculate updates for h,C,A
    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
        h[i] += delta;
       h_update[i] = -eps * (calculateE(x_delay,h ,y_delay, eita) - E_0) / delta;
        //h_update[i] = -2*eps *eita[i]*eita[i]*g(y_delay[i]);
      h[i] -= delta;
      for (int j = 0; j < NUMBER_CHANNELS; j++)
      {
        C[i][j] += delta;
        C_update[i][j] = -eps * (calculateE(x_delay,h, y_delay,eita) - E_0) / delta;
        C[i][j] -= delta;
        //A[i][j] += delta;
        //A_update[i][j] = -eps * (calculateE(x_delay, y_delay,eita) - E_0) / delta;
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


  virtual void learnmodel(double y_delay[NUMBER_CHANNELS],double x_buffer[BUFFER_SIZE][NUMBER_CHANNELS]){
          double z[NUMBER_CHANNELS];
          double A_update[NUMBER_CHANNELS][NUMBER_CHANNELS];

         double xsi[NUMBER_CHANNELS];
   // Berechne xsi
      for(int i=0; i<NUMBER_CHANNELS; i++){
       xsi[i]=x_buffer[(t+BUFFER_SIZE)%BUFFER_SIZE][i];
       for (int j = 0; j < NUMBER_CHANNELS; j++)
        {

         xsi[i] -= A[i][j] * y_delay[j];

        }
      }


       for (int i=0;i<NUMBER_CHANNELS;i++)
       {
           for (int j=0; j<NUMBER_CHANNELS; j++){

             A_update[i][j]=eps*factor_a*xsi[i]*y_delay[j] ;
              A[i][j]+=squash(A_update[i][j])  ;

           }
       }
   };





  /// calculate delayed values
  virtual void calculateDelayedValues(double source[NUMBER_CHANNELS][NUMBER_CHANNELS], int number_steps_of_delay_, double *target)
  {
    // number_steps_of_delay must not be larger than BUFFER_SIZE
    assert (number_steps_of_delay_ < BUFFER_SIZE);

    // get delayed value from ring buffer

    for (int i = 0; i < NUMBER_CHANNELS; i++)
    {
      target[i] = source[(t - number_steps_of_delay_ + BUFFER_SIZE) % BUFFER_SIZE][i];
    }

  };

  virtual void calculateSmoothValues(double source[NUMBER_CHANNELS][NUMBER_CHANNELS], int number_steps_for_averaging_, double *target)
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



  ///
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
  RobotLearnControl(double eps=0.7,double rho=0.0, int number_steps_of_delay_=1):
  eps(eps), rho(rho), number_steps_of_delay(number_steps_of_delay_), delta(0.01),
  number_steps_for_averaging(1),factor_a(0.1),epsilon_it(0.1),number_it(100), t(0), m(0.0)
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
    std::cout<<"init: NUMBER_CHANNELS="<<NUMBER_CHANNELS<<std::endl;
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
   //detL=Determinant(L)  ;
   //detC=Determinant(C)  ;

   //detA=Determinant(A)  ;



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
    learnmodel(y_effective,x_buffer);

    // update step counter
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
   virtual void setFactor_a(double x)
  {
    factor_a = x;
  };

   virtual void setNumber_it(int x)
  {
    number_it= x;
  };

  virtual int getNumber_it()
  {
    return (number_it);
  };

   virtual void setEps_it(double x)
  {
    epsilon_it= x;
  };

   virtual double getEps_it()
  {
    return (epsilon_it);
  };

  virtual double getFactor_a()
  {
    return (factor_a);
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
