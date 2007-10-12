#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <iterator>

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/dercontroller.h>
#include <selforg/one2onewiring.h>

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

using namespace std;

bool stop=0;
double noise=0.1;
double sleep_=1000;

typedef enum Mode {INERTIA, OPENEND, SINEINPUT};


/** This robot emulates different systems based on the mode
    parameter. 
    This is usually some kind of short-circuit with inertia, 
     additional inputs/outputs ...
    */
class MyRobot : public AbstractRobot {
public:
  MyRobot(const string& name, Mode mode, int dimension = 2)
    : AbstractRobot(name, "$Id$"),
      mode(mode) {
    t=0;
    switch(mode){
    case INERTIA:
      motornumber  = dimension;
      sensornumber = dimension;      
      break;
    case OPENEND:
      motornumber  = dimension+1;
      sensornumber = dimension;
      break;
    case SINEINPUT:
      motornumber  = dimension;
      sensornumber = dimension+1;
      break;
    }
    x = new double[sensornumber];
    y = new double[motornumber];
    addParameterDef("tau", &tau, 100);
    addParameterDef("inertia", &inertia, 0.0);

    addParameterDef("sleep",   &sleep_,   1000); // actually a global parameter 
    addParameter("noise", &noise);               // actually a global parameter 
  }

  ~MyRobot(){
    if(x) delete[] x;
    if(y) delete[] y;
  }

  // robot interface

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] 
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber){
    assert(sensornumber >= this->sensornumber);
    memcpy(sensors, x, sizeof(sensor) * this->sensornumber);
    return this->sensornumber;
  }

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber){
    assert(motornumber >= this->motornumber);
    memcpy(y, motors, sizeof(motor) * this->motornumber);
    switch(mode){
    case INERTIA: 
    case OPENEND: doInertia(); break;
    case SINEINPUT : doSineInput(); break;
    }   
    t++;    
  }

  /** returns number of sensors */
  virtual int getSensorNumber(){ return sensornumber; }

  /** returns number of motors */
  virtual int getMotorNumber() { return motornumber; }

  virtual Position getPosition()     const {return Position(0,0,0);}
  virtual Position getSpeed()        const {return Position(0,0,0);}
  virtual Position getAngularSpeed() const {return Position(0,0,0);}
  virtual matrix::Matrix getOrientation() const {
    matrix::Matrix m(3,3);
    m.toId();
    return m; 
  };

  // different Systems:
  
  /// system with inertia  (also used for OpenEnd because we ignore
  /// additional motors
  void doInertia(){
    for(int i=0; i<sensornumber; i++){
      x[i] = x[i]*inertia + y[i]*(1-inertia); 
    }
  }

  // system with one additional input, that is a sine wave (tau is
  // period). the rest motors have inertia as above
  void doSineInput(){
    for(int i=0; i<motornumber; i++){
      x[i] = x[i]*inertia + y[i]*(1-inertia); 
    }
    x[sensornumber-1]=sin(t/tau);    
  }

private:
  int motornumber;
  int sensornumber;

  Mode mode;
  double* x;
  double* y;

  paramval tau;
  paramval inertia;
  int t;

}; 


void printRobot(MyRobot* robot){
  char line[81];
  memset(line,'_', sizeof(char)*80);
  line[80]=0;
  sensor s[20];
  int len = robot->getSensors(s,20);
  for(int i=0; i<len; i++){
    double x = s[i];
    line[int((x+1.0)/2.0*80.0)]='0'+ i;
  }
  
  printf("\033[1G%s",line);
  fflush(stdout);
  
}

// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}

int main(int argc, char** argv){
  list<PlotOption> plotoptions;

  if(contains(argv,argc,"-g")!=0) plotoptions.push_back(PlotOption(GuiLogger,Controller,10));
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g] [-f]\n",argv[0]);
    printf("\t-g\tstart guilogger\n\t-f\twrite logfile\n\t-h\tdisplay this help\n");
    exit(0);
  }
  // TODO add mode
  
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");

  Mode mode  = INERTIA;
  int index = contains(argv,argc,"-m");
  char* modestr = "inertia";
  if(index >0 && argc>index) {
    modestr = argv[index];
    if(strcasecmp(modestr,"inertia")==0) mode = INERTIA;
    else if(strcasecmp(modestr,"openend")==0) mode = OPENEND;
    else if(strcasecmp(modestr,"sineinput")==0) mode = SINEINPUT;
    else modestr="inertia";
  }

  GlobalData globaldata;
  MyRobot* robot;
  Agent* agent;
  initializeConsole();
  
  
//   InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
//   cc.useSD=true;
//   AbstractController* controller = new InvertMotorNStep(cc);
  AbstractController* controller = new InvertMotorSpace(10,1.0);
//  AbstractController* controller = new DerController();
//  AbstractController* controller = new InvertNChannelController(10,1.0);
  controller->setParam("s4delay",   1.0);
  controller->setParam("s4avg",     1.0);  
  controller->setParam("adaptrate", 0.0);  
  controller->setParam("factorB",   0.01);  
  
  robot         = new MyRobot(string("Robot_") + string(modestr), mode);
  agent         = new Agent(plotoptions);
  AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));  
  agent->init(controller, robot, wiring);
  // if you like, you can keep track of the robot with the following line. 
  //  this assumes that you robot returns its position, speed and orientation. 
  // agent->setTrackOptions(TrackRobot(true,false,false, false,"systemtest"));
  
  globaldata.agents.push_back(agent);
  globaldata.configs.push_back(robot);
  globaldata.configs.push_back(controller);
 
  
  showParams(globaldata.configs);
  
  cmd_handler_init();
  long int t=0;
  while(!stop){

    agent->step(noise,t);

    if(control_c_pressed()){      
      if(!handleConsole(globaldata)){
        stop=1;
      }
      cmd_end_input();
    }
    if(t%10==0){
      if(sleep_) usleep((int)(10*sleep_));    
      if(sleep_ > 100 || t%100==0) 
	printRobot(robot);    
    }
    t++;
  };
  delete agent;
  closeConsole();
  fprintf(stderr,"terminating\n");
  // should clean up but what costs the world
  return 0;
}
