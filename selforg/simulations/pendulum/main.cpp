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
#include <selforg/matrix.h>

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

/*
Pendulum

m*l \frac{d^2\phi}{dt^2}= - m*g \sin(\phi) - \mu \frac{d\phi}{dt} + f(t)
where \phi is the displacement angle from the vertical (resting position)
\mu is the friction and f(t) is the additional force at time t

we get 2 ODE in units of \sqrt{l/g}:
\frac{d\phi}{dt}   = \omega
\frac{d\omega}{dt} = - \sin(\phi) - \mu \omega + f(t)

We will use a 4th order Runge-Kutta
*/

using namespace std;
using namespace matrix;

const int SIZEX=80;
const int SIZEY=35;

bool stop=0;
bool reset=true;
double realtimefactor=10;
double noise = 0.1;


class Pendulum : public AbstractRobot {
public:
  Pendulum(const string& name, double dt = 0.01)
    : AbstractRobot(name, "$Id$") {
    motornumber  = 1;
    sensornumber = 1;
    x = new double[sensornumber];
    y = new double[motornumber];

    state.set(2,1);
    state.val(0,0) = 0.0; // position
    state.val(1,0) = 0.0; // speed
    this->dt=dt;

    addParameterDef("mu", &mu, 0.01);
    addParameterDef("sensorscale", &sensorscale, 0.5);
    addParameterDef("maxForce", &maxForce, 0.1);
    addParameterDef("reinf", &reinf,0);

    addParameter("realtimefactor", &realtimefactor); //global param. a bit stupid to do here, but who cares
    addParameter("noise", &noise); // global param
  }

  ~Pendulum(){
    if(x) delete[] x;
    if(y) delete[] y;
  }

  // set of 2 ODEs for pendulum
  //  F(param, {\phi,\omega}) = {d\phi/dt, d\omega/dt}
  static Matrix System(const void* p, Matrix s){
    const Pendulum* pendl = (Pendulum*) p;
    Matrix result(2,1);
    double phi = s.val(0,0);
    double omega = s.val(1,0);
    // d\phi/dt = \omega
    result.val(0,0) = omega;
    // d\omega/dt = force(x)
    result.val(1,0) = -sin(phi) - pendl->mu*omega + pendl->maxForce*pendl->y[0];
    return result;
  }
  

  // Runge Kutta of forth order for systems of ODEs
  Matrix RungeKutta4(const void* p, Matrix s, Matrix (*F)(const void*, Matrix), double dt ){
    Matrix k1 = F(p, s) * dt;
    Matrix k2 = F(p, s + (k1 * 0.5)) * dt;
    Matrix k3 = F(p, s + (k2 * 0.5)) * dt; 
    Matrix k4 = F(p, s + k3) * dt;
    return  s + (k1 + k2*2.0 + k3*2.0 + k4)*(1.0/6.0);
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
    state = RungeKutta4(this, state, System, dt);
    while(state.val(0,0)>M_PI) state.val(0,0)-=2*M_PI;
    while(state.val(0,0)<-M_PI) state.val(0,0)+=2*M_PI;
    x[0] = state.val(0,0);    
  }  

  virtual int getSensorNumber(){ return sensornumber; }
  virtual int getMotorNumber() { return motornumber; }
  virtual Position getPosition() const {return Position(state.val(0,0),0,0);}
  virtual Position getSpeed() const {return Position(state.val(1,0),0,0);}
  virtual Position getAngularSpeed() const {return Position(0,0,0);}
  virtual matrix::Matrix getOrientation() const {
    matrix::Matrix m(3,3); m.toId();  return m; 
  };


private:
  int motornumber;
  int sensornumber;

  double* x;
  double* y;

  double dt; // stepsize
  double mu; // friction

  Matrix state; // phi and omega

  paramval sensorscale;// factor for tactile sensor 
  paramval maxForce;   // factor for Force
  paramval reinf;      // strength of reinforcement
}; 


int coord(double phi,double len){ 
  double x = sin(phi)*len;
  double y = cos(phi)*len;
  return int((x+1.0)/2*SIZEX) + int((y+1.0)/2*SIZEY)*SIZEX;  
}

void printRobots(Pendulum* robot){
  char field[SIZEX*SIZEY];
  char color[SIZEX*SIZEY];
  memset(field,' ', sizeof(char)*SIZEX*SIZEY);
  memset(color,0, sizeof(char)*SIZEX*SIZEY);
  double phi = robot->getPosition().x;
  double omega = robot->getSpeed().x;
  for(double l=0; l<1; l+=0.03){
    field[coord(phi,l)]='X';
    color[coord(phi,l)]=1;
  }
  field[coord(phi,0)]='O';
  color[coord(phi,0)]=2;
  
  if(reset){
    printf("\n");
    reset=false;
  }else{
    printf("\033[1G");
    printf("\033[%iA",SIZEY+1);
  }
  for(int j=0; j<SIZEY; j++){
    for(int i=0; i<SIZEX; i++){
      printf("\033[%im%c",(color[i + j*SIZEX]==0) ? 0 : 100+color[i + j*SIZEX], 
	     field[i + j*SIZEX]);
    }
    printf("\033[0m\n");    
  }  
  printf("Phi: %lf\tOmega: %lf\n", phi, omega);
  fflush(stdout);
  
}

void reinforce(Agent* a){
//   MyRobot* r = (MyRobot*)a->getRobot();  
//   InvertMotorNStep* c = dynamic_cast<InvertMotorNStep*>(a->getController());
//   if(c)
//     c->setReinforcement(r->getParam("reinf")*(r->whatDoIFeel != 0));
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

  Pendulum* pendulum;;
  
  InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
  //    cc.useSD=true;
  AbstractController* controller = new InvertMotorNStep(cc);
  controller->setParam("s4delay",4.0);
  controller->setParam("s4avg",1.0);  
  controller->setParam("adaptrate",0.0);  
  controller->setParam("factorB",0.1);  
  controller->setParam("epsC",0.1);  
  controller->setParam("epsA",0.1);  
  
  pendulum               = new Pendulum("Pendulum");
  Agent* agent           = new Agent(plotoptions);
  AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));  
  agent->init(controller, pendulum, wiring);
  // if you like, you can keep track of the robot with the following line. 
  //  this assumes that you robot returns its position, speed and orientation. 
  //  if(i==0) agent->setTrackOptions(TrackRobot(true,true,false, false,"updown_SD",10));
    
  globaldata.configs.push_back(pendulum);
  globaldata.configs.push_back(controller);
  globaldata.agents.push_back(agent);
  
  showParams(globaldata.configs);
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");
  printf(" You probably want to use the guilogger with e.g.: -g 10\n");
  
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
      printRobots(pendulum);
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
