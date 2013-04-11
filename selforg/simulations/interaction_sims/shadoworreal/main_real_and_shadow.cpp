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
#include "console.h"
#include "globaldata.h"

/*
This experiment is inspired by Eziquiel DiPaolo, but he was
not the inventor, but e used it with robotic agents as well.
The experiment was originally conduced with babies and their
mothers. The baby does not react on a replay of the mother.
Here we have two agents which can move a long a one-dimensional
space. They can touch the other when they are at the some place or
their place consides with a shadow of the other. In this case the
perception is not mutual.
*/

using namespace std;

bool stop=0;
double sleep_=10000;
double shadowdist = 0.6;

double toEnv(double pos){
  // environment is cyclic
  if(pos>1) pos-=2;
  if(pos<-1) pos+=2;
  return pos;
}
Position toEnv(Position pos){
  pos.x = toEnv(pos.x);
  return pos;
}

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
    addParameterDef("mass", &mass, _mass);
    addParameterDef("mu", &mu, 1.5);
    addParameterDef("range", &range, 0.3);
    addParameterDef("sensorscale", &sensorscale, 0.5);

    addParameter("sleep", &sleep_); //  global param. a bit stupid  to do here, but who cares
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
        a = F/m - \mu v_0/m // friction approximation
        v = a*t + v0
        x = v*t + x0
    */
    Position a(y[0],0,0);
    a     = (a - speed*mu)*(1/mass);
    speed = a*t + speed;
    pos   = speed*t + pos;

    // environment is cyclic
    pos = toEnv(pos);

    int len=0;
    //  speed sensor (proprioception)
    for(int i=0; i<1; i++){
      x[len] = speed.toArray()[i];
      len++;
      if(len>=sensornumber) return;
    }
    whatDoIFeel=0;
    int k=1;
    //  other agents sensor (tactile or whatever)
    FOREACHC(list<const MyRobot*>, otherRobots, i){
      Position opos   = (*i)->getPosition();
      // shadow of the other in the cyclic environment
      Position shadow = toEnv(opos + Position(shadowdist,0,0));

      double rdist = toEnv(pos.x-opos.x);
      double sdist = toEnv(pos.x-shadow.x);
      // feel shortest object (there should be only on in range anyway)
      double dist = fabs(rdist) < fabs(sdist) ? rdist : sdist;
      if(dist < -range) x[len]=0;
      else if(dist > range) x[len]=0;
      else {
        whatDoIFeel = k        * (fabs(rdist) < fabs(sdist) ? 1 : -1);
        //        x[len] = sensorscale * tactilezigzag(dist,range);
        x[len] = sensorscale * tactileupdown(dist,range);
        // x[len] = sensorscale * tactilebump(dist,range);
        // x[len] = sensorscale * tactilesteepsides(dist,range);
      }
      len++;
      k++;
      if(len>sensornumber){
        fprintf(stderr,"something is wrong with the sensornumber\n");
      }
    }
  }

  virtual int getSensorNumber(){ return sensornumber; }
  virtual int getMotorNumber() { return motornumber; }
  virtual Position getPosition() const {return pos;}
  virtual Position getSpeed() const {return speed;}
  virtual Position getAngularSpeed() const {return Position(x[0],x[1],whatDoIFeel);}
  virtual matrix::Matrix getOrientation() const {
    matrix::Matrix m(3,3); m.toId();  return m;
  };

  virtual void addOtherRobot(const MyRobot* otherRobot){
    if(otherRobot!=this)
      otherRobots.push_back(otherRobot);
  }

  // different tactile sensors
  double tactilezigzag(double dist, double range){
    if(dist < 0) return -1+fabs((-range/2) - dist)/(range/2);
    else return 1-fabs((range/2) - dist)/(range/2);
  }
  double tactileupdown(double dist, double range){
    // this is a  0-down-0-up-0 curve.
    // gnuplot> scale = 0.5
    // gnuplot> plot exp(1)*(scale*x)*(exp(-abs(scale*x)))
    double scale = 6/range;
    return exp(1) * scale*dist * exp(-abs(scale*dist));
  }
  double tactilebump(double dist, double range){
    // this is a bump at 0
    double x = dist/range;
    return 1-x*x;
  }
  double tactilesteepsides(double dist, double range){
    return dist/range;
  }

private:
  int motornumber;
  int sensornumber;

  double* x;
  double* y;

  double t; // stepsize
  double mu; // friction
  double mass; // mass of the robot, that determines the inertia
  paramval range;// range of tactile sensor (-range..range) measured
               // from current position.
  paramval sensorscale;// factor for tactile sensor
  Position pos;
  Position speed;
  list<const MyRobot*> otherRobots;
public:
  int whatDoIFeel; // 0: nothing, x>0 agent x; x<0 shadow of agent x
};


int coord(double x){ return int((x+1.0)/2*80);}

void printRobots(const list<MyRobot*>& robots){
  char line[81];
  char color[80];
  memset(line,'_', sizeof(char)*80);
  line[80]=0;
  memset(color,0, sizeof(char)*80);
  // first their sensor range
  int k=0;
  FOREACHC(list<MyRobot*>, robots, i) {
    double x = (*i)->getPosition().x;
    int start = coord(x-(*i)->getParam("range"));
    int end = coord(x+(*i)->getParam("range"));
    for(int i=start; i<end; i++){
      color[(i+80)%80] |= 1<<k;
    }
    k++;
  }
  k=0;
  // robots itself
  FOREACHC(list<MyRobot*>, robots, i) {
    double x = (*i)->getPosition().x;
    line[coord(x)]='A'+ k;
    x = toEnv(x+shadowdist);
    line[coord(x)]='a'+ k;
    k++;
  }

  printf("\033[1G");
  for(int i=0; i<80; i++){
    printf("\033[%im%c",color[i]==0 ? 0 : 100+color[i],line[i]);
  }
  printf("\033[0m");
  fflush(stdout);

}

void reinforce(Agent* a){
  MyRobot* r = (MyRobot*)a->getRobot();
  InvertMotorNStep* c = dynamic_cast<InvertMotorNStep*>(a->getController());
  if(c)
    c->setReinforcement(2*(r->whatDoIFeel != 0));
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

  int index = contains(argv,argc,"-g");
  if(index >0 && argc>index) {
    plotoptions.push_back(PlotOption(GuiLogger,Controller,atoi(argv[index])));
  }
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g N] [-f] \n",argv[0]);
    printf("\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n");
    printf("\t-h\tdisplay this help\n");
    exit(0);
  }

  list<MyRobot*> robots;

  for(int i=0; i<2; i++){
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.useSD=true;
    AbstractController* controller = new InvertMotorNStep(cc);
    controller->setParam("s4delay",1.0);
    controller->setParam("s4avg",1.0);
    controller->setParam("adaptrate",0.0);
    controller->setParam("factorB",0.1);
    controller->setParam("epsC",0.1);
    controller->setParam("epsA",0.1);

    MyRobot* robot         = new MyRobot("Robot" + itos(i), Position(i*0.3,0,0),0.1);
    Agent* agent           = new Agent(i==0 ? plotoptions : list<PlotOption>());
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent->init(controller, robot, wiring);
    // if you like, you can keep track of the robot with the following line.
    //  this assumes that you robot returns its position, speed and orientation.
    //    if(i==0)
    agent->setTrackOptions(TrackRobot(true,true,false, false,"updown_reinf2_SD",10));

    globaldata.configs.push_back(robot);
    globaldata.configs.push_back(controller);
    robots.push_back(robot);
    globaldata.agents.push_back(agent);
  }

  showParams(globaldata.configs);
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");
  printf("The output of the program is as follows:\n");
  printf(" Capital letters are the Agents, like A,B,C. Lower case are their shadows.\n");
  printf(" You probably want to use the guilogger with e.g.: -g 10\n");

  // connect robots to each other
  FOREACH (list<MyRobot*>, robots, i){
    FOREACH (list<MyRobot*>, robots, k){
      if(*i != *k)
        (*i)->addOtherRobot(*k);
    }
  }

  cmd_handler_init();
  long int t=0;
  while(!stop){
    FOREACH(AgentList, globaldata.agents, i){
      (*i)->step(0.1,t/100.0);
      reinforce(*i);
    }
    if(control_c_pressed()){
      if(!handleConsole(globaldata)){
        stop=1;
      }
      cmd_end_input();
    }
    int drawinterval = 100000;
    if(sleep_){
      drawinterval = (int)(1000000.0/(25*sleep_));
      if(sleep_ < 5000 && (t%10)==0)
        usleep((int)(10*sleep_));
      else usleep((int)(sleep_));
    }
    if(t%drawinterval==0)
      printRobots(robots);
    t++;
  };

  FOREACH(AgentList, globaldata.agents, i){
    delete (*i);
  }
  closeConsole();
  fprintf(stderr,"terminating\n");
  // should clean up but what costs the world
  return 0;
}
