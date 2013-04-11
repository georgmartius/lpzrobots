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
Multiple robots in a 2D cyclic environment.
Different sensor configurations:
 - The robots can touch the other when they are close to each other.
 - or relative position sensor
*/

using namespace std;

const bool shadow = true;
const int SIZEX=80;
const int SIZEY=35;

bool stop=0;
bool reset=true;
double realtimefactor=10;
double shadowdist = 0.6;
double noise = 0.1;

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


class MyRobot : public AbstractRobot {
public:
  MyRobot(const string& name, const Position& initial_pos, double _mass = 1.0)
    : AbstractRobot(name, "$Id$") {
    motornumber  = 2;
    sensornumber = 4;
    x = new double[sensornumber];
    y = new double[motornumber];
    pos = initial_pos;
    speed = Position(0,0,0);
    t = 0.01;
    addParameterDef("mass", &mass, _mass);
    addParameterDef("mu", &mu, 1.5);
    addParameterDef("range", &range, 0.5);
    addParameterDef("sensorscale", &sensorscale, 0.5);
    addParameterDef("reinf", &reinf,0);

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

    // motor values are now stored in y, sensor values are expected to be stored in x

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
      if(len>=sensornumber) return;
    }

    if(len+2>sensornumber){
      fprintf(stderr,"something is wrong with the sensornumber\n");
    }
    whatDoIFeel=0;
    //  other agents sensor (tactile or whatever)
    int k=1;
    double mindist = 10.0;
    int feelrobot=1;
    FOREACHC(list<const MyRobot*>, otherRobots, i){
      Position opos   = (*i)->getPosition();
      Position diff = toEnv(pos-opos);
      double dist = diff.length();
      if(shadow){
        // shadow of the other in the cyclic environment
        Position shadow = toEnv(opos + Position(shadowdist,shadowdist,0));
        Position sdiff = toEnv(pos-shadow);
        double sdist = sdiff.length();
        if(fabs(sdist) < fabs(dist)){
          feelrobot=-1;
          diff = sdiff;
          dist = sdist;
        }
      }
      // only measure the closest robot
      if(dist<mindist){mindist = dist;}
      else continue;
      if(     min(diff.x,diff.y) < -range) x[len]=0;
      else if(max(diff.x,diff.y) > range) x[len]=0;
      else {
        whatDoIFeel = k * feelrobot;
        //        x[len] = sensorscale * tactilezigzag(dist,range);
        x[len]   = sensorscale * tactileupdown(diff.x,range);
        x[len+1] = sensorscale * tactileupdown(diff.y,range);
        // x[len] = sensorscale * tactilebump(dist,range);
        // x[len] = sensorscale * tactilesteepsides(dist,range);
      }
      k++;
    }
    len+=2;
  }

  virtual int getSensorNumber(){ return sensornumber; }
  virtual int getMotorNumber() { return motornumber; }
  virtual Position getPosition() const {return pos;}
  virtual Position getSpeed() const {return speed;}
  /** we return not the angular velocity but the taktile senors and
      whom we feel */
  virtual Position getAngularSpeed() const {return Position(x[2],x[3],whatDoIFeel);}
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
    double scale = 5/range;
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
  paramval reinf;      // strength of reinforcement
  Position pos;
  Position speed;
  list<const MyRobot*> otherRobots;
public:
  int whatDoIFeel; // 0: nothing, x>0 agent x; x<0 shadow of agent x
};


int coordx(double x){ return int((x+1.0)/2*SIZEX);}
int coordy(double y){ return int((y+1.0)/2*SIZEY);}

void printRobots(const list<MyRobot*>& robots){
  char field[SIZEX*SIZEY];
  char color[SIZEX*SIZEY];
  memset(field,'_', sizeof(char)*SIZEX*SIZEY);
  memset(color,0, sizeof(char)*SIZEX*SIZEY);
  // first their sensor range
  int k=0;
  FOREACHC(list<MyRobot*>, robots, i) {
    double x = (*i)->getPosition().x;
    double y = (*i)->getPosition().y;
    int xstart = coordx(x-(*i)->getParam("range"));
    int xend = coordx(x+(*i)->getParam("range"));
    int ystart = coordy(y-(*i)->getParam("range"));
    int yend = coordy(y+(*i)->getParam("range"));
    for(int i=xstart; i<xend; i++){
      for(int j=ystart; j<yend; j++){
        color[((i+SIZEX)%SIZEX)+((j+SIZEY)%SIZEY)*SIZEX] |= 1<<k;
      }
    }
    k++;
  }
  k=0;
  // robots itself
  FOREACHC(list<MyRobot*>, robots, i) {
    int x = coordx((*i)->getPosition().x);
    int y = coordy((*i)->getPosition().y);
    field[x + y*SIZEX]='A'+ k;
    if(shadow){
      x = coordx(toEnv((*i)->getPosition().x+shadowdist));
      y = coordy(toEnv((*i)->getPosition().y+shadowdist));
      field[x + y*SIZEX]='a'+ k;
    }
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
  MyRobot* r = (MyRobot*)a->getRobot();
  InvertMotorNStep* c = dynamic_cast<InvertMotorNStep*>(a->getController());
  if(c)
    c->setReinforcement(r->getParam("reinf")*(r->whatDoIFeel != 0));
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
    plotoptions.push_back(PlotOption(GuiLogger,atoi(argv[index])));
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
    //    cc.useSD=true;
    AbstractController* controller = new InvertMotorNStep(cc);
    controller->setParam("s4delay",4.0);
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
    if(i==0) agent->setTrackOptions(TrackRobot(true,true,false, false,"updown_SD",10));

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
      (*i)->step(noise,t/100.0);
      reinforce(*i);
    }
    if(control_c_pressed()){
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
      printRobots(robots);
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
