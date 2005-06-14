#ifndef __ONE2ONEAGENT_H
#define __ONE2ONEAGENT_H

#include "plotagent.h"

/// Glue-object between controller and robot which implements a 1 to 1 mapping 
class One2oneAgent : public PlotAgent {
public:
  One2oneAgent() : PlotAgent() {
    sensors=0; motors=0;
  }
  /// constructs the object with the given controller and robot
  One2oneAgent(AbstractController* controller, AbstractRobot* robot) 
    : PlotAgent (controller, robot) {}

  One2oneAgent~(){
    if(sensors) free(sensors);
    if(motors) free(motors);
  }

  virtual void step();
private:
  int sensornumber;
  int motornumber;
  sensor *sensors;
  motor *motors;
}
 
};


#endif
