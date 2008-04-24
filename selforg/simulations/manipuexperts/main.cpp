#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <iterator>

#include <selforg/agent.h>
#include <selforg/position.h>
#include <selforg/stl_adds.h>

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

#include "multiexpertpair.h"
#include <selforg/printInternals.h>

/*
Test of multiexpert system with manipulandum data
*/

using namespace std;
using namespace matrix;


bool stop=0;
bool reset=true;
double realtimefactor=1;

int stepsize=2;
list<PlotOption> plotoptions;
GlobalData globaldata;
list<const Inspectable*> inspectables;


class Sim: public Configurable, public Inspectable {
public:
  Sim(const string& name)
    : Configurable(name, "$Id$") {
    addParameter("realtimefactor", &realtimefactor); 
    pred_error=0;
  }

  ~Sim(){
    delete mep;    
  }


  AbstractModel* init(const char* filename){
    FILE *
    f=fopen(filename, "r");
    if(!f) {
      fprintf(stderr, "cannot open file %s\n", filename);
      exit(1);
    }  
    
    //  vector<Matrix> speeds;
    double time;
    int frame;
    while(!feof(f)){
      Matrix m(3,1);
      fscanf(f,"%lf%i%lf%lf%lf",&time,&frame,&m.val(0,0),&m.val(1,0),&m.val(2,0));
      pos.push_back(m);
    }    
    data_size= pos.size();
    fclose(f);
    //   for(int i=0; i<data_size-1; i++){
    //     speeds.push_back(pos[i+1]-pos[i]);
    //   }
    //   data_size= speeds.size();
  
  
    MultiExpertPairConf pc = MultiExpertPair::getDefaultConf();
    pc.numSats=8;
    pc.numHidden=6;
    pc.lambda_w=0.05;
    pc.eps0=0.1;
    pc.tauE1=5;
    pc.tauW=300;    
    mep  = new MultiExpertPair(pc);
    mep->init(3*3,3);

    t= 4*stepsize;

    return mep;
  }

  int step(){
    // learn
    Matrix input = getData(pos,t,-1).above(getData(pos,t,-2).above(getData(pos,t,-3)));
    Matrix output = getData(pos,t,0);
    mep->learn(input,output);
    // interative predict the future
    int horizont=3;
    list<Matrix> pred;
    predict(&pred, mep, pos, t, horizont);
    // calculate error
    pred_error= calcError(pred, pos, t);
    if((t%5000) == 0) printf("Time %i\n",t);
    if(t>data_size-(horizont+1)*stepsize)
      t=4*stepsize;
    return t++;    
  }

  /// calculates Speed at time t+d(stepsize)
  Matrix getData(const vector<Matrix>& pos, int t, int d){
    return pos[t+(d*stepsize)] - pos[t+((d-1)*stepsize)];

  }

  // predicts the future 
  void predict(list<Matrix>* pred, AbstractModel* modell, const vector<Matrix>& pos, int t, int steps){
    Matrix b[3]={getData(pos,t,-2), getData(pos,t,-1), getData(pos,t,0)};
    for(int i=0; i< steps; i++){
      Matrix input = b[(i+2)%3].above(b[(i+1)%3]).above(b[(i+0)%3]);
      b[i%3]= modell->process(input);    
      pred->push_back(b[i%3]);  
    }
  }

  double calcError(const list<Matrix>& pred, const vector<Matrix>& pos, int t){
    int i=0;
    double error = 0;
    FOREACHC( list<Matrix>, pred, p){
      i++;
      error += ((*p) - getData(pos,t,i)).multTM().val(0,0); 
      // calc length of speed difference
    }
    return error;
  }

  virtual iparamkeylist getInternalParamNames() const{
    list<iparamkey> keylist;  
    keylist += storeVectorFieldNames(pos[0], "pos");
    keylist += string("pred_error");
    return keylist; 
  }
  virtual iparamvallist getInternalParams() const{
    list<iparamval> l;
    l += pos[t].convertToList();
    l += (double)pred_error;
    return l;
  }

public:
  vector<Matrix> pos;
  MultiExpertPair * mep;  
  double pred_error;
  int data_size;  
  int t;
};


// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}

void openPlotOptions(list<PlotOption>& plotoptions , list<const Inspectable*>& inspectables){
  signal(SIGPIPE,SIG_IGN);
  FOREACH(list<PlotOption>, plotoptions, p){
    p->open();
    if(p->pipe){
      // print start
      time_t t = time(0);
      fprintf(p->pipe,"# Start %s", ctime(&t));    
      // print interval
      fprintf(p->pipe, "# Recording every %dth dataset\n", p->interval);
      printInternalParameterNames(p->pipe, 0, 0, inspectables);
    }
  }
}

void plot(list<PlotOption>& plotoptions, list<const Inspectable*>& inspectables, int t){
  FOREACH(list<PlotOption>, plotoptions, i){
    if( ((*i).pipe) && (t % (*i).interval == 0) ){      
      printInternalParameters((*i).pipe, t, 0, 0, 0, 0, inspectables);
      if(t% ((*i).interval * 10)) fflush((*i).pipe);
    }
  }
}

int main(int argc, char** argv){
  initializeConsole();
  char* filename = "input.dat";
  int index = contains(argv,argc,"-g");
  if(index >0 && argc>index) {
    plotoptions.push_back(PlotOption(GuiLogger,Controller,atoi(argv[index])));
  }
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g N] [-f] [-file FILE] \n",argv[0]);
    printf("\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n");
    printf("\t-file FILE\tread FILE\n");
    printf("\t-h\tdisplay this help\n");
    exit(0);
  }
  index = contains(argv,argc,"-file");
  if(index >0 && argc>index) {
    filename = strdup(argv[index]);
  }

  Sim sim("ManipuTest");
 
  globaldata.configs.push_back(&sim);
  inspectables.push_back(&sim);
  AbstractModel* m = sim.init(filename);
  globaldata.configs.push_back(m);  
  inspectables.push_back(m);

  showParams(globaldata.configs);
  openPlotOptions(plotoptions, inspectables);
  
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");
  printf(" You probably want to use the guilogger with e.g.: -g 10\n");
  
  cmd_handler_init();
  while(!stop){
    int t = sim.step();
    plot(plotoptions, inspectables,t);
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
      usleep(60000);
    }    
  };

  closeConsole();  
  fprintf(stderr,"terminating\n");
  FOREACH(list<PlotOption>, plotoptions, i){
    i->close();
  }
  return 0;
}
