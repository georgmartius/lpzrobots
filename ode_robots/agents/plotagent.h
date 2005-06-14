#ifndef __PLOTAGENT_H
#define __PLOTAGENT_H

#include "agent.h"

/// Abstract glue-object between controller and robot. 
//   Adds gnuplot functionality 
class PlotAgent : public Agent {
public:
  PlotAgent();
  /// constructs the object with the given controller and robot
  PlotAgent(AbstractController* controller, AbstractRobot* robot) 
    : Agent (controller, robot) {}

  PlotAgent~() { 
    CloseGui();
  } 
  
  virtual void init(AbstractController* controller, AbstractRobot* robot);

protected:
  /**
   * plotting sensorvalues x, motorvalues y, matrix A, matrix C and vector h
   * @param x actual sensorvalues (used for generation of motorcommand in actual timestep)
   * @param y actual motorcommand (generated in the actual timestep)
   */
  virtual void plot(const sensor* x, int sensornumber, const motor* y, int motornumber);

  static void OpenGui();
  static void CloseGui():

private:
  FILE* pipe;
  int numberInternalParameters;
};


#endif


