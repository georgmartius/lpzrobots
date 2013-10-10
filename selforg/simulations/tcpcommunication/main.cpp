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
#include <selforg/one2onewiring.h>
#include <selforg/sinecontroller.h>
#include <selforg/motorbabbler.h>
//#include <selforg/semox.h>
//#include <selforg/crossmotorcoupling.h>
//#include <selforg/universalcontroller.h>
#include "tcpcontroller.h"


#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

using namespace std;

bool stop=0;
double noise=.1;
double realtimefactor=1;

enum Mode {NORMAL, SWING, EXTRASINE};


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
    buffersize=20;
    switch(mode){
    case NORMAL:
    case SWING:
      motornumber  = dimension;
      sensornumber = dimension;
      break;
    case EXTRASINE:
      motornumber  = dimension;
      sensornumber = dimension+1;
      addParameterDef("tau",          &tau,        100.0);
      break;
    }
    x = new double[sensornumber];
    y = new double[motornumber];
    x_buffer = new double*[buffersize];
    for (unsigned int k = 0; k < buffersize; k++) {
      x_buffer[k] = new double[sensornumber];
      memset(x_buffer[k],0,sizeof(double)*sensornumber);
    }
    v = new double[motornumber];
    memset(x,0,sizeof(double)*motornumber);
    memset(y,0,sizeof(double)*motornumber);
    memset(v,0,sizeof(double)*motornumber);

    x[0]=1;
    //0: no interia, 1: only inertia, no control, a good value is 0.9
    addParameterDef("inertia",      &inertia,      0.0);
    // if not 0 then the last sensor is set to 0 (only noise remains)
    //  this usually kills the controller after some time, because
    //  the C value for the input goes to infinity and eventually
    //  strong interaction between the other channels occur
    addParameterDef("disablesensor",&disablesensor,0.0);
    // 0 no correlation, 1 very strong correlation of subsequent sensors
    addParameterDef("correlation",  &correlation,  0.0);
    // 1: normal delay, >1 additional delay in the loop, use s4del of
    // controllers in this situaltion
    addParameterDef("delay",        &delay,        1.0);

    // parameter that defines the speed of the simulation
    addParameter("realtimefactor",&realtimefactor); // actually a global parameter
    // noise level
    addParameter("noise", &noise);               // actually a global parameter
  }

  ~MyRobot(){
    if(x) delete[] x;
    if(y) delete[] y;
    if(x_buffer) {
      for (unsigned int k = 0; k < buffersize; k++)
        if(x_buffer[k]) delete [] x_buffer[k];
      delete[] x_buffer;
    }
  }

  // robot interface

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1]
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber){
    assert(sensornumber >= this->sensornumber);
    int d = max(1,int(delay));
    double* x_cur = x_buffer[(t-d + buffersize)%buffersize];
    memcpy(sensors, x_cur, sizeof(sensor) * this->sensornumber);
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
    case SWING: swing(); break;
    case NORMAL:
    default: normal(); break;
    }
    if(mode == EXTRASINE){
      extraSineInput();
    }
    // copy the sensor values into the buffer
    memcpy(x_buffer[t%buffersize],x,sizeof(sensor)*sensornumber);
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

  // additional input, that is a sine wave (tau is
  // period)
  void extraSineInput(){
    x[sensornumber-1]=sin(t/tau);
  }

  // system with damping, correlation of subsequent channels (cyclic),
  // and delay (the delay is actually done by getSensors()
  void normal(){
    for(int i=0; i<motornumber; i++){
      x[i] = x[i]*inertia + y[i]*(1-inertia) + correlation*x[(i+1)%motornumber];
    }
    if(disablesensor!=0.0)
      x[motornumber-1]= 0;
  }


  // system that is swingable, and possible correlation of subsequent channels (cyclic),
  // and delay (the delay is actually done by getSensors()
  void swing(){
    //    double* x_tm1 = x_buffer[(t-1+buffersize)%buffersize];
    for(int i=0; i<motornumber; i++){
      // x_{t+1} = f(x) + x_t - x_{t-1} +  correlation
      // x_{t+1} = f(x) - (1-1/m)* x_t - x_{t-1}
      cout <<      x[i] << endl;
      //      x[i] = tanh(x[i] - (1-1/(inertia+0.5))*x[i] - x_tm1[i]);
    }
    if(disablesensor!=0.0)
      x[motornumber-1]= 0;
  }



private:
  int motornumber;
  int sensornumber;
  unsigned int buffersize;

  Mode mode;
  double* x;
  double* v; // velocities
  double* y;
  double** x_buffer;

  paramval tau;
  paramval inertia;
  paramval correlation;
  paramval disablesensor;
  paramval delay;
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
    x=clip(x,-1.0,1.0);
    line[int((x+1.0)/2.0*80.0)%80]='0'+ i;
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
  Mode mode  = NORMAL;
  const char* modestr = "normal";
  int dim = 2;

  int index = contains(argv,argc,"-g");
  if(index >0 && argc>index) {
    plotoptions.push_back(PlotOption(GuiLogger, atoi(argv[index])));
  }
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-n")!=0) plotoptions.push_back(PlotOption(NeuronViz));
  index = contains(argv,argc,"-m");
  if(index >0 && argc>index) {
    modestr = argv[index];
    if(strcasecmp(modestr,"normal")==0) mode = NORMAL;
    else if(strcasecmp(modestr,"swing")==0) mode = SWING;
    else if(strcasecmp(modestr,"extrasine")==0) mode = EXTRASINE;
    else { fprintf(stderr, "Unknown mode! See help -h\n"); exit(1);}
  }
  index = contains(argv,argc,"-d");
  if(index >0 && argc>index)
    dim=atoi(argv[index]);
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g N] [-f] [-n] [-m MODE] [d DIM]\n",argv[0]);
    printf("\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n");
    printf("\t-n\tstart neuronviz\n");
    printf("\t-m MODE\t system properties: normal (def), swing, extrasine\n");
    printf("\t-d DIM\t dimensionality, default 2\n");
    printf("\t-h\tdisplay this help\n");
    exit(0);
  }

  GlobalData globaldata;
  MyRobot* robot;
  Agent* agent;
  initializeConsole();

// Different controllers
//   SeMoXConf cc = SeMoX::getDefaultConf();
//   cc.modelExt=false;
//   //cc.useS=true;
//   cc.someInternalParams=true;
//   AbstractController* controller = new SeMoX (cc);

//     SeMoX* semox = new SeMoX (cc);
//     CrossMotorCoupling* controller = new CrossMotorCoupling( semox, semox);
//     std::list<int> perm;
//     perm += 1;
//     perm += 0;
//     CMC cmc = controller->getPermutationCMC(perm);
//     controller->setCMC(cmc);

//   UniversalControllerConf cc = UniversalController::getDefaultConf();
//   vector<Layer> layers;
//   layers.push_back(Layer(5,0.5,FeedForwardNN::tanh)); // hidden layer
//   layers.push_back(Layer(0,1,FeedForwardNN::tanhr)); // motor layer
//   // size of output layer is automatically set
//   layers.push_back(Layer(1,0,FeedForwardNN::linear));

//   Elman* e = new Elman(1,layers,false, false, false);
//   cc.net = e;
//   cc.init = 1;
//   cc.someInternalParams=false;
//   AbstractController* controller = new UniversalController(cc);
//   controller->setParam("epsDyn",     0);


//  AbstractController* controller = new InvertMotorSpace(10,1.2);

//  AbstractController* controller = new InvertNChannelController(10,false);

  //AbstractController* controller = new SineController();
  //AbstractController* controller = new Homeokinesis();

  //AbstractController* controller = new MotorBabbler();
  AbstractController* controller = new TcpController("Test", 4000);


  controller->setParam("epsC",     0.1);

  robot         = new MyRobot(string("Robot_") + string(modestr), mode, dim);
  agent         = new Agent(plotoptions);
  AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.2),
                                             AbstractWiring::Controller | AbstractWiring::Noise);
  // AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(),true);
  agent->init(controller, robot, wiring);
  // if you like, you can keep track of the robot with the following line.
  //  this assumes that you robot returns its position, speed and orientation.
  // agent->setTrackOptions(TrackRobot(true,false,false, false,"systemtest"));

  globaldata.agents.push_back(agent);
  globaldata.configs.push_back(robot);
  globaldata.configs.push_back(controller);


  showParams(globaldata.configs);
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");
  printf("The output of the program is more fun then useful ;-).\n");
  printf(" The number are the sensors and the position there value.\n");
  printf(" You probably want to use the guilogger with e.g.: -g 10\n");

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
    int drawinterval = 10000;
    if(realtimefactor!=0){
      drawinterval = max(1,int(6*realtimefactor));
    }
    if(t%drawinterval==0){
      printRobot(robot);
      usleep(60000);
    }
    t++;
  };
  delete agent;
  closeConsole();
  fprintf(stderr,"terminating\n");
  // should clean up but what costs the world
  return 0;
}
