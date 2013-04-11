#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <iterator>
#include <algorithm>

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
// #include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/one2onewiring.h>

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

/*
Multiple robots in a 2D cyclic environment.
Flocking simulation:
 - near robots teach each other
*/

using namespace std;
using namespace matrix;

const bool shadow = true;
const int SIZEX=110;
const int SIZEY=42;

bool stop=0;
bool reset=true;
double realtimefactor=10;
double noise = 0.1;
GlobalData globaldata;

double toEnv(double pos){
  // environment is cyclic
  if(pos>1) pos-=2;
  if(pos<-1) pos+=2;
  return pos;
}
Position toEnv(Position pos){
  pos.x = toEnv(pos.x);
  pos.y = toEnv(pos.y);
  return pos;
}

typedef pair<double, Matrix> Neighbor;

bool neighborcmp(const Neighbor& n1, const Neighbor& n2){
  return n1.first < n2.first;
}

class MyRobot : public AbstractRobot {
public:
  MyRobot(const string& name, const Position& initial_pos, AbstractController* controller, double _mass = 1.0)
    : AbstractRobot(name, "$Id$") {
    this->controller=dynamic_cast<InvertMotorNStep*>(controller);

    motornumber  = 2;
    sensornumber = 2;
    x = new double[sensornumber];
    y = new double[motornumber];
    pos = initial_pos;
    speed = Position(0,0,0);
    t = 0.01;
    addParameterDef("mass", &mass, _mass);
    addParameterDef("mu", &mu, 1.5);
    addParameterDef("range", &range, 0.15);
    addParameterDef("num_vision", &num_vision,0);

    addParameter("realtimefactor", &realtimefactor); //global param. a bit stupid to do here, but who cares
    addParameter("noise", &noise); // global param
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

    // motor values are now stored in y,
    //  sensor values are expected to be stored in x

    // perform robot action here
    /*  simple discrete simulation
        a = F/m - \mu v_0/m // friction approximation
        v = a*t + v0
        x = v*t + x0
    */
    Position a(y[0],y[1],0);
    a     = (a - speed*mu)*(1/mass);
    speed = a*t + speed;
    pos   = speed*t + pos;

    // environment is cyclic
    pos = toEnv(pos);

    int len=0;
    //  speed sensor (proprioception)
    for(int i=0; i<2; i++){
      x[len] = speed.toArray()[i];
      len++;
      //      if(len>=sensornumber) return;
    }

    teaching2();

  }


  virtual void teaching1(){
    //  other agents motors
    whatDoISee=0;
    //    double weights=0.0;
    Matrix neighboraction = this->getAction();
    neighboraction.toZero();
    FOREACHC(vector<Agent*>, globaldata.agents, i){
      MyRobot* r = (MyRobot*)(*i)->getRobot();
      if(r==this) continue;
      Position opos   = r->getPosition();
      Position diff = toEnv(pos-opos);
      double dist = diff.length();
      // only measure close by robot

      if(dist<range){
         neighboraction += r->getAction();
//        neighboraction += r->getAction()*(1-sqr(dist/range));
//        weights+=(1-sqr(dist/range));
         whatDoISee++;
      }
    }
    if(controller && whatDoISee>0)
      neighboraction *= 1.0/whatDoISee;
      //    neighboraction *= 1.0/weights;
      controller->setMotorTeachingSignal(neighboraction.unsafeGetData(),
                                         neighboraction.size());
  }

  virtual void teaching2(){
    //  other agents motors
    whatDoISee=0;
    vector<Neighbor > neighbors;
    int k=1;
    FOREACHC(vector<Agent*>, globaldata.agents, i){
      MyRobot* r = (MyRobot*)(*i)->getRobot();
      if(r==this) continue;
      Position opos   = r->getPosition();
      Position diff = toEnv(pos-opos);
      double dist = diff.length();
      // only measure close by robot
      if(dist<range){
        neighbors.push_back(pair<double, Matrix>(dist,r->getAction()));
         whatDoISee++;
      }
      k++;
    }
    if(controller && whatDoISee && num_vision>0){
      std::sort(neighbors.begin(), neighbors.end(),neighborcmp);
      Matrix neighboraction;
      int n=0;
      FOREACHC(vector<Neighbor>, neighbors, i){
        if(neighboraction.isNulltimesNull())
          neighboraction=i->second;
        else
          neighboraction+=i->second;
        n++;
        if(n>=num_vision) break;
      }
      neighboraction*=1.0/n;
      controller->setMotorTeachingSignal(neighboraction.unsafeGetData(),
                                         neighboraction.size());
    }
  }


  virtual int getSensorNumber(){ return sensornumber; }
  virtual int getMotorNumber() { return motornumber; }
  virtual Position getPosition() const {return pos;}
  virtual Position getSpeed() const {return speed;}
  /** we return not the angular velocity but the taktile senors and
      whom we feel */
  virtual Position getAngularSpeed() const {return Position(x[2],x[3],whatDoISee);}
  virtual matrix::Matrix getOrientation() const {
    matrix::Matrix m(3,3); m.toId();  return m;
  };

  Matrix getAction() {
    return Matrix(motornumber,1,y);
  }

private:
  int motornumber;
  int sensornumber;

  double* x;
  double* y;

  InvertMotorNStep* controller;

  double t; // stepsize
  double mu; // friction
  double mass; // mass of the robot, that determines the inertia
  paramval range;// range of tactile sensor (-range..range) measured
               // from current position.
  paramval num_vision; // number of other agents that influence this robot
  Position pos;
  Position speed;
public:
  int whatDoISee; // 0: nothing, x>0: x agents;
};


int coordx(double x){ return int((x+1.0)/2*SIZEX);}
int coordy(double y){ return int((y+1.0)/2*SIZEY);}

void printRobots(){
  char field[SIZEX*SIZEY];
  char color[SIZEX*SIZEY];
  memset(field,'_', sizeof(char)*SIZEX*SIZEY);
  memset(color,0, sizeof(char)*SIZEX*SIZEY);
  // first their sensor range
  int k=2;
  FOREACHC(vector<Agent*>, globaldata.agents, i) {
    MyRobot* r = (MyRobot*) (*i)->getRobot();
    double x = r->getPosition().x;
    double y = r->getPosition().y;
    int xstart = coordx(x-r->getParam("range"));
    int xend = coordx(x+r->getParam("range"));
    int ystart = coordy(y-r->getParam("range"));
    int yend = coordy(y+r->getParam("range"));
    for(int i=xstart; i<xend; i++){
      for(int j=ystart; j<yend; j++){
        int index = ((i+SIZEX)%SIZEX)+((j+SIZEY)%SIZEY)*SIZEX;
        if(color[index] < 2) color[index]++;
      }
    }
    color[coordx(x) + coordy(y)*SIZEX]--;
    k++;
  }
  k=0;
  // robots itself
  FOREACHC(vector<Agent*>, globaldata.agents, i) {
    MyRobot* r = (MyRobot*) (*i)->getRobot();
    int x = coordx(r->getPosition().x);
    int y = coordy(r->getPosition().y);
    field[x + y*SIZEX]='A'+ k;
    int index = x + y*SIZEX;
    if(color[index] < 5) color[index]++;
    k++;
  }

  if(reset){
    printf("\n");
    reset=false;
  }else{
    printf("\033[1G");
    printf("\033[%iA",SIZEY);
  }
  for(int j=0; j<SIZEY; j++){
    for(int i=0; i<SIZEX; i++){
      printf("\033[%im%c",(color[i + j*SIZEX]==0) ? 0 : 100+color[i + j*SIZEX],
             field[i + j*SIZEX]);
    }
    printf("\033[0m\n");
  }
  fflush(stdout);

}

void reinforce(Agent* a){
//   MyRobot* r = (MyRobot*)a->getRobot();
//   InvertMotorNStep* c = dynamic_cast<InvertMotorNStep*>(a->getController());
//   if(c)
//     c->setReinforcement(r->getParam("reinf")*(r->whatDoISee != 0));
}


// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}

int main(int argc, char** argv){
  initializeConsole();
  list<PlotOption> plotoptions;
  int numagents=2;

  int index = contains(argv,argc,"-g");
  if(index >0 && argc>index) {
    plotoptions.push_back(PlotOption(GuiLogger,atoi(argv[index])));
  }
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  index = contains(argv,argc,"-a");
  if(index>0 && argc>index) {
    numagents=atoi(argv[index]);
  }
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-a N] [-g N] [-f] \n",argv[0]);
    printf("\t-a N\tnumber of agents\n");
    printf("\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n");
    printf("\t-h\tdisplay this help\n");
    exit(0);
  }

  for(int i=0; i<numagents; i++){
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    //    cc.useSD=true;
    AbstractController* controller = new InvertMotorNStep(cc);
    controller->setParam("s4delay",4.0);
    controller->setParam("s4avg",1.0);
    controller->setParam("adaptrate",0.0);
    controller->setParam("factorB",0.1);
    controller->setParam("epsC",0.1);
    controller->setParam("epsA",0.1);
    controller->setParam("teacher",0.05);

    MyRobot* robot         =
      new MyRobot("Robot" + itos(i), Position(random_minusone_to_one(0),
                                              random_minusone_to_one(0),0),
                  controller,0.1);
    Agent* agent           = new Agent(i==0 ? plotoptions : list<PlotOption>());
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent->init(controller, robot, wiring);
    // if you like, you can keep track of the robot with the following line.
    //  this assumes that you robot returns its position, speed and orientation.
    if(i==0) agent->setTrackOptions(TrackRobot(true,true,false, false,"updown_SD",10));

    globaldata.configs.push_back(robot);
    globaldata.configs.push_back(controller);
    globaldata.agents.push_back(agent);
  }

  //  showParams(globaldata.configs);
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");
  printf("The output of the program is as follows:\n");
  printf(" Letters are the agents, like A,B,C. Red is the visual field.\n");
  printf(" Green is the overlapping visual field and other.\n");
  printf(" Yellow,Blue,Cyan means 2,3,4 or more agents at the same place.\n");
  printf(" You probably want to use the guilogger with e.g.: -g 10\n");

  cmd_handler_init();
  long int t=0;
  while(!stop){
    FOREACH(AgentList, globaldata.agents, i){
      (*i)->step(noise,t/100.0);
      reinforce(*i);
    }
    if(control_c_pressed() || t==8000){
      if(!handleConsole(globaldata)){
        stop=1;
      }
      reset=true;
      cmd_end_input();
    }
    int drawinterval = 10000;
    if(realtimefactor){
      drawinterval = int(6*realtimefactor);
    }
    if(t%drawinterval==0){
      printRobots();
      usleep(60000);
    }
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
