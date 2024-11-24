#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <iterator>

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
#include <selforg/pimax.h>
#include <selforg/one2onewiring.h>

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

using namespace std;

bool stop=0;

/**
   The robot lives on a 1-D space follows discrete physics, with mass and friction.
   The sensor values are the own speed and the distance to the other robots

*/
class MyRobot : public AbstractRobot {
public:
  MyRobot(const string& name, const Position& initial_pos, double _mass = 1.0)
    : AbstractRobot(name, "$Id$") {
    motornumber  = 1;
    sensornumber = 2;
    x = new double[sensornumber];
    y = new double[motornumber];
    pos = initial_pos;
    speed = Position(0,0,0);
    t = 0.01;
    // This is how to add configurable parameters
    addParameterDef("mu",&mu, 0.7, 0, 1, "friction");
    addParameterDef("mass",&mass, _mass, 0, 100, "mass of the robot");
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
    //  sense other agents (distance)
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


  virtual Position getAngularSpeed() const {return speed;}

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
};


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
  GlobalData globaldata;
  initializeConsole();

  list<PlotOption> plotoptions;

  if(contains(argv,argc,"-g")!=0) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-s")!=0) plotoptions.push_back(PlotOption(SoundMan));
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g] [-f]\n",argv[0]);
    printf("\t-g\tstart guilogger\n\t-f\twrite logfile\n\t-h\tdisplay this help\n");
    exit(0);
  }

  printf("\nPress Ctrl-c to invoke parameter input shell\n");

  list<MyRobot*> robots;

  for(int i=0; i<2; i++){
    AbstractController* controller = new PiMax();
    // controller->setParam("s4delay",1.0);
    // controller->setParam("s4avg",2.0);
    // controller->setParam("adaptrate",0.0);
    // controller->setParam("factorB",0.01);

    MyRobot* robot         = new MyRobot("Robot" + itos(i), Position(0,0,0));
    Agent* agent           = new Agent(i==0 ? plotoptions : list<PlotOption>());
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent->init(controller, robot, wiring);
    // if you like, you can keep track of the robot use the following line.
    agent->setTrackOptions(TrackRobot(true,false,false, false,"mutual"));

    globaldata.configs.push_back(robot);
    globaldata.configs.push_back(controller);
    robots.push_back(robot);
    globaldata.agents.push_back(agent);
  }

  showParams(globaldata.configs);

  // connect robots to each other
  FOREACH (list<MyRobot*>, robots, i){
    FOREACH (list<MyRobot*>, robots, k){
      (*i)->addOtherRobot(*k);
    }
  }

  cmd_handler_init();
  while(!stop){
    usleep(1000);
    FOREACH (vector<Agent*>, globaldata.agents, i){
      (*i)->step(0.1);
    }
    if(control_c_pressed()){
      if(!handleConsole(globaldata)){
        stop=1;
      }
      cmd_end_input();
    }
    printRobots(robots);
  };

  FOREACH (vector<Agent*>, globaldata.agents, i){
    delete (*i);
  }
  closeConsole();
  fprintf(stderr,"terminating\n");
  // should clean up but what costs the world
  return 0;
}
