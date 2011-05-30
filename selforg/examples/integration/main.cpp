#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <iterator>
using namespace std;

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
#include <selforg/invertmotorspace.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include "cmdline.h"

bool stop=0;

class MyRobot : public AbstractRobot {
public:
  MyRobot()
    : AbstractRobot("MyRobot", "$Id$") {
    myparam=0;
    motornumber  = 3;
    sensornumber = 3;
    x = new double[sensornumber];
    y = new double[motornumber];
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
    assert(sensornumber == this->sensornumber);
    memcpy(sensors, x, sizeof(sensor) * sensornumber);
    return sensornumber;
  }

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber){
    assert(motornumber == this->motornumber);
    memcpy(y, motors, sizeof(motor) * motornumber);

    // motor values are now stored in y, sensor values are expected to be stored in x

    // perform robot action here 

    //  this is just a senceless dummy robot
    for(int i=0; i<sensornumber; i++){
      x[i] = sin(myparam) * y[i%motornumber];
    }
    myparam+=0.01;
   
  }

  /** returns number of sensors */
  virtual int getSensorNumber(){ return sensornumber; }

  /** returns number of motors */
  virtual int getMotorNumber() { return motornumber; }

  /** returns position of the object
      @return vector of position (x,y,z) */
  virtual Position getPosition() const {return Position(0,0,0);}

  /** returns linear speed vector of the object
      @return vector  (vx,vy,vz)
   */
  virtual Position getSpeed() const {return Position(0,0,0);}

  /** returns linear speed vector of the object
      @return vector  (vx,vy,vz)
   */
  virtual Position getAngularSpeed() const {return Position(0,0,0);}

  /** returns the orientation of the object
      @return 3x3 rotation matrix
   */
  virtual matrix::Matrix getOrientation() const {
    matrix::Matrix m(3,3);
    m.toId();
    return m; 
  };

  virtual paramval getParam(const paramkey& key, bool traverseChildren) const{
    if(key == "myparam") return myparam; 
    else return Configurable::getParam(key);
  }

  virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren){
    cerr << "huhu";
    if(key == "myparam") myparam = val; 
    else return Configurable::setParam(key, val); 
    return true;
  }

  virtual paramlist getParamList() const {
    paramlist list;
    list += pair<paramkey, paramval> (string("myparam"), myparam);
    return list;
  };

private:
  int motornumber;
  int sensornumber;

  double* x;
  double* y;
  
  paramval myparam;
}; 


void onTermination(){
  stop=1;
}

// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}

int main(int argc, char** argv){
  ConfigList configs;
  list<PlotOption> plotoptions;

  AbstractController* controller = new InvertMotorSpace(10);
  controller->setParam("s4delay",2.0);
  controller->setParam("s4avg",2.0);

  if(contains(argv,argc,"-g")!=0) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g] [-f]\n",argv[0]);
    printf("\t-g\tstart guilogger\n\t-f\twrite logfile\n\t-h\tdisplay this help\n");
    exit(0);
  }
  
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");
  
  AbstractRobot* robot   = new MyRobot();
  Agent* agent           = new Agent(plotoptions);
  AbstractWiring* wiring  = new One2OneWiring(new ColorUniformNoise(0.1));  
  agent->init(controller, robot, wiring);
  // if you like, you can keep track of the robot with the following line. 
  //  this assumes that you robot returns its position, speed and orientation. 
  // agent->setTrackOptions(TrackRobot(true,false,false, false,"interagetiontest"));
 
  configs.push_back(robot);
  configs.push_back(controller);
  showParams(configs);

  cmd_handler_init();
  //while(!stop){
  for(int n=0; n<20; n++) { 
    usleep(1000);
    agent->step(0.1);
    if(control_c_pressed()){
      cmd_begin_input();
      changeParams(configs, onTermination);
      cmd_end_input();
    }
    
  };  


  fprintf(stderr,"terminating\n");
  delete agent;
  delete controller;
  delete wiring;
  delete robot;

  return 0;
}
