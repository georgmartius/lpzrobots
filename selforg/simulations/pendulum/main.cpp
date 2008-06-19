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
#include <selforg/replaycontroller.h>
#include <selforg/matrix.h>

#include "multisat.h"

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"
#include "pendulum.h"

using namespace std;
using namespace matrix;

bool stop=false;

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
  char* replayFile=0;
  bool useMultiAgent=false;

  int index = contains(argv,argc,"-g");
  if(index >0 && argc>index) {
    plotoptions.push_back(PlotOption(GuiLogger,Controller,atoi(argv[index])));
  }
  index = contains(argv,argc,"-f");
  if(index >0 && argc>index) {
    plotoptions.push_back(PlotOption(File,Controller,atoi(argv[index])));
  }
  index = contains(argv,argc,"-replay");
  if(index >0 && argc>index) {
    replayFile = argv[index];
  }
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g N] [-f N] [-replay]\n",argv[0]);
    printf("\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n");
    printf("\t-replay\tuse replay filuilogger with interval N\n\t-f\twrite logfile\n");
    printf("\t-h\tdisplay this help\n");
    exit(0);
  }

  Pendulum* pendulum;;
  
  AbstractController* controller;
  if(replayFile)
    controller = new ReplayController(replayFile,true);
  else{
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.useS=true;
    controller = new InvertMotorNStep(cc);
    // controller = new SineController();
  }
  controller->setParam("s4delay",1);
  controller->setParam("s4avg",1.0);  
  controller->setParam("adaptrate",0.0);  
  controller->setParam("factorB",0.0);  
  controller->setParam("epsC",0.1);  
  controller->setParam("epsA",0.05);  
  controller->setParam("dampS",0.001);  
  controller->setParam("steps",1);  
  controller->setParam("logaE",3);  
  controller->setParam("sinerate",100);  


  pendulum               = new Pendulum("Pendulum", &globaldata, 20);
  Agent* agent           = new Agent(plotoptions);
  AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise());  

  MultiSat* multisat;
  if(useMultiAgent){
    MultiSatConf msc = MultiSat::getDefaultConf();
    msc.controller = controller;
    msc.numContext = 2;
    msc.numHidden = 2;
    msc.numSats   = 2; 
    msc.penalty   = 10.0; 
    msc.eps0      = 0.05;
    msc.deltaMin  = 1/100.0;
    msc.tauE1     = 50;
    msc.tauE2     = 200;
    msc.tauC     = 500;
    msc.tauW     = 2000;
    
    msc.useDerive=false;
    msc.useY=false;
    multisat = new MultiSat(msc);
    agent->addInspectable(controller);
    agent->init(multisat, pendulum, wiring);
    globaldata.configs.push_back(multisat);
  }else 
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
      (*i)->step(globaldata.noise,t/100.0);
      reinforce(*i);
    }
    if(control_c_pressed()){      
      if(!handleConsole(globaldata)){
        stop=true;
      }
      pendulum->setBlankScreen();
      cmd_end_input();
    }
    int drawinterval = 10000;
    if(globaldata.realtimefactor){
      drawinterval = int(6*globaldata.realtimefactor);
    }
    if(t%drawinterval==0){
      pendulum->print();
      usleep(60000);
    }    
    t++;    
  };

  if(useMultiAgent) multisat->storeSats("pendulum_1h");
  FOREACH(AgentList, globaldata.agents, i){
    delete (*i);
  }
  closeConsole();
  fprintf(stderr,"terminating\n");
  // should clean up but what costs the world
  return 0;
}
