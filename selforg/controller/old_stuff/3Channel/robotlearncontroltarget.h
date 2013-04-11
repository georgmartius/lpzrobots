#include <cassert>
#include <cstdlib>
#include <iostream>

#include "robotlearncontrol.h"


template <int NUMBER_CHANNELS, int BUFFER_SIZE=1> class RobotLearnControlTarget :public RobotLearnControl<NUMBER_CHANNELS, BUFFER_SIZE>{


  protected:
  double eps_target; // learning rate for target learning

  virtual void learnTarget(double *x, double *y){
    double E[NUMBER_CHANNELS];

    E[0]=y[1]-y[0];   // Motorkommandos sollen den gleichen Wert haben
    E[1]=y[0]-y[1];   // -> Roboter fährt geradeaus
    //funktioniert, Roboter fährt gut geradeaus

    //    E[0]=0-y[0];   // y[0] soll 0 sein, y[1] irgendeinen Wert
    //    E[1]=0;        // -> Roboter dreht um eines der Räder
    //funktioniert, auch wenn y[1] nicht 0 wird, sondern bis +/-0.2 pendelt ergibt sich die Drehung um ein Rad


    //E[0]=-y[1]-y[0];   // y[0] soll -y[1] sein
    //E[1]=0;        // -> Roboter dreht auf der Stelle
    //funktioniert, Roboter dreht schön auf der Stelle


    for (int i=0; i<NUMBER_CHANNELS; i++){
      for (int j=0; j<NUMBER_CHANNELS; j++){
        C[i][j]+=RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::squash(eps_target*E[i]*x[j] /*- 0.1*eps_target*C[i][j]*/);
      }
      h[i]+=RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::squash(eps_target*E[i] /*- 0.1*eps_target*h[i]*/);
    }
  };

  virtual void learnLastOutput(double *x, double *y_teach){
    double E[NUMBER_CHANNELS];
    double y[NUMBER_CHANNELS],z[NUMBER_CHANNELS];

    for (int i=0; i<NUMBER_CHANNELS; i++){
      for (int j=0; j<NUMBER_CHANNELS; j++){
        z[i]=C[i][j]*x[j];
      }
      z[i]=h[i];
      y[i]=RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::g(z[i]);
      E[i]=y_teach[i]-y[i];
    }

    for (int i=0; i<NUMBER_CHANNELS; i++){
      for (int j=0; j<NUMBER_CHANNELS; j++){
        C[i][j]+=RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::squash(eps_target*E[i]*x[j] /*- 0.1*eps_target*C[i][j]*/);
      }
      h[i]+=RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::squash(eps_target*E[i] /*- 0.1*eps_target*h[i]*/);
    }
  };



  public:
  RobotLearnControlTarget():
  RobotLearnControl<NUMBER_CHANNELS, BUFFER_SIZE> ()
  {
    eps_target=0.1;
  }


  virtual void setC(int i, int j ,double val){
    C[i][j]=val;
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
    RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::putInBuffer(x_buffer, x_);

    // averaging over the last number_steps_for_averaging values of x_buffer
    RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::calculateSmoothValues(x_buffer, number_steps_for_averaging, x_smooth);

    // calculate controller values based on smoothed input values
    RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::calculateControllerValues(x_smooth, y_);

    // put new output value in ring buffer y_buffer
    RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::putInBuffer(y_buffer, y_);

    // calculate effective input/output, which is (actual-number_steps_of_delay) element of buffer
    RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::calculateDelayedValues(x_buffer, number_steps_of_delay, x_effective);
    RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::calculateDelayedValues(y_buffer, number_steps_of_delay, y_effective);

    // learn controller with effective input/output
    RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::learn(x_effective, y_effective);

    // learn model with actual input and effective output (that produced the actual input)
    RobotLearnControl<NUMBER_CHANNELS,BUFFER_SIZE>::learnModel(x_buffer[(t+BUFFER_SIZE)%BUFFER_SIZE], y_effective);

    // learn target for controller with effective input/output
        learnTarget(x_effective, y_effective);

    // learn last output with (input+random value) as input
        //
        // sollte einmal begonnene Geradeausfahrt stabilisieren,
        // weil wenn Roboter geradeausfährt ist kein Lernsignal für target-Lernen da und
        // c's werden von Homeokinese verändert, weg vom geradeaus fahren
        //
        // Ergebnis: diagonal c's werden vergrößert und Verhalten wird superstabil, keine Reaktion mehr auf Kollision mit Wand
        // => weglassen
        //
     double x_rand[2];
     for(int i=0; i<NUMBER_CHANNELS; i++){  // noise with pos. sign
       x_rand[i]=x_effective[i]+(double(rand())/RAND_MAX)*0.05;
     }
     learnLastOutput(x_rand, y_effective);

     for(int i=0; i<NUMBER_CHANNELS; i++){  // noise with neg. sign
       x_rand[i]=x_effective[i]-(double(rand())/RAND_MAX)*0.05;
     }
     learnLastOutput(x_rand, y_effective);

    // update step counter
    t++;
  };


  /*  /// make step (calculate controller outputs and learn controller)
  virtual void makeStep(double *x_, double *y_)
  {

     RobotLearnControl<N,buffer_size>::makeStep(x_, y_);

    // learn target for controller with effective input/output
    learnTarget(x_effective, y_effective);

    };
  */

  virtual void setEpsTarget(double x)
  {
    eps_target = x;
  };
  virtual double getEpsTarget()
  {
    return (eps_target);
  };

};



