#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <iterator>

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
// #include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>
#include <selforg/feedbackwiring.h>

#include "cmdline.h"

using namespace std;

bool stop=0;

class MyRobot : public AbstractRobot {
public:
  MyRobot(const string& name, int dof)
    : AbstractRobot(name, "$Id$") {
    prevRobot = 0;
    myparam=0;
    degrees  = dof;
    x = new double[degrees];
    y = new double[degrees];
    memset(x,0,sizeof(double) * degrees);
    memset(y,0,sizeof(double) * degrees);
    t = 0.01;
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
    assert(sensornumber == this->degrees);
    assert(prevRobot);
    assert(prevRobot->degrees == sensornumber);
    memcpy(sensors, prevRobot->y, sizeof(sensor) * sensornumber);
    memcpy(x, sensors, sizeof(sensor) * sensornumber);   
    return sensornumber;
  }

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber){
    assert(motornumber == this->degrees);
    memcpy(y, motors, sizeof(motor) * motornumber);   
  }

  /** returns number of sensors */
  virtual int getSensorNumber(){ return degrees; }

  /** returns number of motors */
  virtual int getMotorNumber() { return degrees; }

  /** returns position of the object
      @return vector of position (x,y,z) */
  virtual Position getPosition() const { return Position(x[0],0,0); }

  /** returns linear speed vector of the object
      @return vector  (vx,vy,vz)
   */
  virtual Position getSpeed() const {return Position();}


  virtual Position getAngularSpeed() const {return Position();}

  /** returns the orientation of the object
      @return 3x3 rotation matrix
   */
  virtual matrix::Matrix getOrientation() const {
    matrix::Matrix m(3,3);
    m.toId();
    return m; 
  };

  virtual void setPrevRobot(MyRobot* otherRobot){
    if(otherRobot!=this)
      prevRobot=otherRobot;
  }

  virtual paramval getParam(const paramkey& key) const{
    if(key == "myparam") return myparam; 
    else return Configurable::getParam(key);
  }

  virtual bool setParam(const paramkey& key, paramval val){
    if(key == "myparam") myparam = val; 
    else return Configurable::setParam(key, val); 
    return true;
  }

  virtual paramlist getParamList() const {
    paramlist list;
    list += pair<paramkey, paramval> (string("myparam"), myparam);
    return list;
  }

private:
  int degrees;

  double* x;
  double* y;

  double t; // stepsize
  MyRobot* prevRobot; // robot to receive sensors from (its motors)
  
  paramval myparam;
  
}; 


void onTermination(){
  stop=1;
}

void printRobots(list<MyRobot*> robots){
  char line[81];
  memset(line,'_', sizeof(char)*80);
  line[80]=0;
  int k=0;
  FOREACH(list<MyRobot*>, robots, i) {    
    double x = (*i)->getPosition().x;
    line[int((x+1)/2.0*80.0)]='0'+ k;
    k++;
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
  ConfigList configs;
  list<PlotOption> plotoptions;
  int dof=1;
  int index;

  if(contains(argv,argc,"-g")!=0) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g] [-f]\n",argv[0]);
    printf("\t-g\tstart guilogger\n\t-f\twrite logfile\n\t-h\tdisplay this help\n");
    exit(0);
  } 
  if((index=contains(argv,argc,"-d"))!=0){
    if(index<argc) dof=atoi(argv[index]);
  }
    
      
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");

  list<MyRobot*> robots;
  list<Agent*> agents;
  
  for(int i=0; i<2; i++){
    AbstractController* controller = new InvertMotorNStep();
    //AbstractController* controller = new SineController();
    controller->setParam("s4delay",1.0);
    controller->setParam("s4avg",2.0);  
    controller->setParam("adaptrate",0.0);  
    controller->setParam("factorB",0.01);  
  
    MyRobot* robot         = new MyRobot("Robot" + itos(i), dof);
    //    Agent* agent           = new Agent(i==0 ? plotoptions : list<PlotOption>());
    Agent* agent           = new Agent(i<2 ? plotoptions : list<PlotOption>());
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));  
    //    AbstractWiring* wiring = new FeedbackWiring(new ColorUniformNoise(0.1),
    //						FeedbackWiring::All,0.7);  
    agent->init(controller, robot, wiring);
    // if you like, you can keep track of the robot with the following line. 
    //  this assumes that you robot returns its position, speed and orientation. 
    // agent->setTrackOptions(TrackRobot(true,false,false, false,"interagetiontest"));
  
    configs.push_back(robot);
    configs.push_back(controller);
    robots.push_back(robot);
    agents.push_back(agent);
  }
  
  showParams(configs);
  
  // connect robots to each other
  FOREACH (list<MyRobot*>, robots, i){
    list<MyRobot*>::iterator j = i;
    j++;
    if(j == robots.end())
      robots.front()->setPrevRobot(*i);
    else
      (*j)->setPrevRobot(*i);    
  }

  cmd_handler_init();
  while(!stop){
    usleep(1000);
    FOREACH (list<Agent*>, agents, i){
      (*i)->step(0.05);
    }
    if(control_c_pressed()){
      cmd_begin_input();
      changeParams(configs, onTermination);
      cmd_end_input();
    }
    printRobots(robots);        
  };
  
  fprintf(stderr,"terminating\n");
  // should clean up but what costs the world
  return 0;
}
