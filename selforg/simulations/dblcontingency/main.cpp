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
#include <selforg/one2onewiring.h>

#include "cmdline.h"

using namespace std;

#define FOREACH(colltype, coll, it) for( colltype::iterator it = (coll).begin(); it!= (coll).end(); it++ )

bool stop=0;

class MyRobot : public AbstractRobot {
public:
  MyRobot(const string& name, const Position& initial_pos, double _mass = 1.0)
    : AbstractRobot(name, "$Id$") {
    myparam=0;
    motornumber  = 1;
    sensornumber = 2;
    x = new double[sensornumber];
    y = new double[motornumber];
    pos = initial_pos;
    speed = Position(0,0,0);
    mass  = _mass;
    t = 0.01;
    mu = 0.7;
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
    /*  simple discrete simulation
        a = F*m - \mu v_0 // friction approximation
        v = a*t + v0
        x = v*t + x0
    */
    Position a(y[0],0,0);
    a     = a*mass - speed*mu;
    speed = a*t + speed;
    pos   = speed*t + pos;

    // environment is cyclic
    if(pos.x>1) pos.x-=2;
    if(pos.x<-1) pos.x+=2;
    
//     //  position sensor
//     for(int i=0; i<sensornumber; i++){
//       x[i] = pos.toArray()[i];
//     }
    int len=0;

    //  speed sensor
    for(int i=0; i<1; i++){
      x[len] = speed.toArray()[i];
      len++;
      if(len>=sensornumber) return;
    }
    //  other agents sensor
    for(list<const MyRobot*>::iterator i = otherRobots.begin();
        i!= otherRobots.end(); i++){
      Position opos = (*i)->getPosition();
      double dist = fabs(pos.x-opos.x);
      x[len] = dist<1 ? dist : 2-dist; // measure always shortest distance
      len++;
      if(len>=sensornumber) return;
    }
   
  }

  /** returns number of sensors */
  virtual int getSensorNumber(){ return sensornumber; }

  /** returns number of motors */
  virtual int getMotorNumber() { return motornumber; }

  /** returns position of the object
      @return vector of position (x,y,z) */
  virtual Position getPosition() const {return pos;}

  /** returns linear speed vector of the object
      @return vector  (vx,vy,vz)
   */
  virtual Position getSpeed() const {return speed;}

  /** returns the orientation of the object
      @return 3x3 rotation matrix
   */
  virtual matrix::Matrix getOrientation() const {
    matrix::Matrix m(3,3);
    m.toId();
    return m; 
  };

  virtual void addOtherRobot(const MyRobot* otherRobot){
    if(otherRobot!=this)
      otherRobots.push_back(otherRobot);
  }

  virtual paramval getParam(const paramkey& key) const{
    if(key == "myparam") return myparam; 
    else return Configurable::getParam(key);
  }

  virtual bool setParam(const paramkey& key, paramval val){
    cerr << "huhu";
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
  int motornumber;
  int sensornumber;

  double* x;
  double* y;

  double t; // stepsize
  double mu; // friction
  double mass;
  Position pos;
  Position speed;

  list<const MyRobot*> otherRobots;
  
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
    line[int((x+1)/2*80)]='0'+ k;
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

  if(contains(argv,argc,"-g")!=0) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g] [-f]\n",argv[0]);
    printf("\t-g\tstart guilogger\n\t-f\twrite logfile\n\t-h\tdisplay this help\n");
    exit(0);
  }
  
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");

  list<MyRobot*> robots;
  list<Agent*> agents;
  
  for(int i=0; i<3; i++){
    AbstractController* controller = new InvertMotorNStep();
    controller->setParam("s4delay",1.0);
    controller->setParam("s4avg",2.0);  
    controller->setParam("adaptrate",0.0);  
    controller->setParam("factorB",0.01);  
  
    MyRobot* robot         = new MyRobot("Robot" + itos(i), Position(0,0,0));
    Agent* agent           = new Agent(i==0 ? plotoptions : list<PlotOption>());
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));  
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
    FOREACH (list<MyRobot*>, robots, k){
      (*i)->addOtherRobot(*k);
    }    
  }

  cmd_handler_init();
  while(!stop){
    usleep(1000);
    FOREACH (list<Agent*>, agents, i){
      (*i)->step(0.1);
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
