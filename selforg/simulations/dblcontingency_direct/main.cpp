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
#include "console.h"
#include "globaldata.h"

using namespace std;

// some evil global variables!
bool stop=0;
list<PlotOption> plotoptions;
int dof=1;
int numRobots=2;


/**
   Robot that is directly coupled to another robot.
   Sensor values are the motor actions of the connected robot.
   Note: this connection does not need to be mutual, it can also be
    via several other robots (each robot only cares about its sensor values)
*/
class MyRobot : public AbstractRobot {
public:
  MyRobot(const string& name, int dof)
    : AbstractRobot(name, "$Id$") {
    prevRobot = 0;
    // This is how to add configurable parameters
    addParameterDef("myparam", &myparam,0, 0, 100, "my parameter description");
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
  virtual Position getPosition() const {
    return Position(x[0], degrees > 1 ? x[1]: 0, degrees > 2 ? x[2]: 0); }

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

private:
  int degrees;

  double* x;
  double* y;

  double t; // stepsize
  MyRobot* prevRobot; // robot to receive sensors from (its motors)

  paramval myparam;

};


void printRobots(vector<Agent*> robots){
  char line[81];
  memset(line,'_', sizeof(char)*80);
  line[80]=0;
  int k=0;
  FOREACH(vector<Agent*>, robots, i) {
    double x = (*i)->getRobot()->getPosition().x;
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

// this is our simulation. We can have configurable variables here, which is handy
class MySim : public Configurable {
public:
  MySim(){
    addParameterDef("noise", &noise, 0.05, 0, 1, "strength of additive noise");
    addParameterDef("wait",  &wait,  20, 0, 1000, "wait in ms");
  }

  void run(GlobalData& globaldata){
    printf("\nPress Ctrl-c to invoke parameter input shell\n");
    // add the simulation to the configuration list, so that we can change the parameters
    globaldata.configs.push_back(this);

    for(int i=0; i<numRobots; i++){
      //      AbstractController* controller = new Homeokinesis();
      //AbstractController* controller = new SineController();
      AbstractController* controller = new InvertMotorNStep();
      controller->setParam("s4delay",1.0);
      controller->setParam("s4avg",2.0);
      controller->setParam("factorB",0.1);

      MyRobot* robot         = new MyRobot("Robot" + itos(i), dof);
      Agent* agent           = new Agent(i<2 ? plotoptions : list<PlotOption>());
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      //    AbstractWiring* wiring = new FeedbackWiring(new ColorUniformNoise(0.1),
      //                                                FeedbackWiring::All,0.7);
      agent->init(controller, robot, wiring);
      // if you like, you can keep track of the robot use the following line.
      // agent->setTrackOptions(TrackRobot(true,false,false, false,"mutual"));

      globaldata.configs.push_back(robot);
      globaldata.configs.push_back(controller);
      globaldata.agents.push_back(agent);
    }

    showParams(globaldata.configs);

    // connect robots to each other (cyclic)
    FOREACH (vector<Agent*>, globaldata.agents, i){
      vector<Agent*>::iterator j = i;
      j++;
      if(j == globaldata.agents.end())
        j=globaldata.agents.begin();
      // the dynamic casts are required to convert the AbstactRobot* to MyRobot*
      MyRobot* r1 = dynamic_cast<MyRobot*>((*j)->getRobot());
      MyRobot* r2 = dynamic_cast<MyRobot*>((*i)->getRobot());
      assert(r1!=0 && r2!=0); // make sure the conversion succeeded
      r1->setPrevRobot(r2);
    }

    initializeConsole();
    cmd_handler_init();
    while(!stop){
      usleep(wait*1000);
      FOREACH (vector<Agent*>, globaldata.agents, i){
        (*i)->step(noise);
      }
      if(control_c_pressed()){
        if(!handleConsole(globaldata)){
          stop=1;
        }
        cmd_end_input();
      }
      printRobots(globaldata.agents);
    };

    FOREACH (vector<Agent*>, globaldata.agents, i){
      delete (*i);
    }
    closeConsole();
    fprintf(stderr,"terminating\n");
    // should clean up but what costs the world
  }

  double noise;
  int wait;
};


int main(int argc, char** argv){
  GlobalData globaldata;
  int index;

  if(contains(argv,argc,"-g")!=0) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if((index=contains(argv,argc,"-d"))!=0){
    if(index<argc) dof=atoi(argv[index]);
  }
  if((index=contains(argv,argc,"-n"))!=0){
    if(index<argc) numRobots=atoi(argv[index]);
  }
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-d DOF] [-n NUM] [-g] [-f]\n",argv[0]);
    printf("\t-d DOF\tDegrees of freedom (Def: 1)\n\t-n NUM\tNumber of robots (Def: 2)\n");
           printf("\t-g\tstart guilogger\n\t-f\twrite logfile\n\t-h\tdisplay this help\n");
    exit(0);
  }
  MySim sim;
  sim.run(globaldata);
  return 0;
}
