#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <iterator>

#include <selforg/agent.h>
#include <selforg/position.h>
#include <selforg/controller_misc.h>
#include <selforg/stl_adds.h>

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

// #include "multiexpertpair.h"
#include "multiexpertsubopt.h"
#include <selforg/printInternals.h>

/*
Test of multiexpert system with manipulandum data
*/

using namespace std;
using namespace matrix;


bool stop=0;
bool reset=true;
double realtimefactor=1;

int stepsize=1; // time intervals for learning and prediction
int horizont=5; // how many step are predicted
list<PlotOption> plotoptions;
GlobalData globaldata;
list<const Inspectable*> inspectables;


class Sim: public Configurable, public Inspectable {
public:
  Sim(const string& name)
    : Configurable(name, "$Id$")
      , predpos(horizont) {
    horizontCopy=horizont;
    stepsizeCopy=stepsize;
    addParameter("realtimefactor", &realtimefactor);
    addParameter("stepsize", &stepsizeCopy);// actually not to change, only for documentation
    addParameter("horizont", &horizontCopy);// actually not to change, only for documentation
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

    double time;
    int frame;
    while(!feof(f)){
      Matrix m(3,1);
      fscanf(f,"%lf%i%lf%lf%lf",&time,&frame,&m.val(0,0),&m.val(1,0),&m.val(2,0));
      pos.push_back(m);
    }
    data_size= pos.size();
    fclose(f);

//     MultiExpertPairConf pc = MultiExpertPair::getDefaultConf();
//     pc.numSats=20;
//     pc.numHidden=6; // 12
//     pc.lambda_w=0.1;
//     pc.eps0=1; //0.1
//     pc.tauE1=2;
//     pc.tauW=300; // 300
//     mep  = new MultiExpertPair(pc);
//     mep->init(3*3,3);

    MultiExpertSuboptConf pc = MultiExpertSubopt::getDefaultConf();
    pc.numSats=6;
    pc.numHidden=6; // 12
    pc.eps0=0.01; //0.1
    pc.tauE1=5;
    pc.tauE2=50;
    pc.tauF=10000;
    pc.satMemory=500;
    pc.satTrainPast=5;
    pc.lambda_comp=2;
    mep  = new MultiExpertSubopt(pc);
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
    list<Matrix> pred;
    predict(&pred, mep, pos, t, horizont);
    // calculate error
    pred_error= calcError(pred, pos, t);
    // calc predicted positions
    calcPredPos(&predpos, pred,pos,t);
    if((t%5000) == 0) printf("Time %i\n",t);
    if(t>data_size-(horizont+1)*stepsize)
      t=4*stepsize;
    return t++;
  }

  Matrix getData(const vector<Matrix>& pos, int t, int d) const {
    return getDataAcc(pos,t,d);
  }

  /// calculates Speed at time t+d(stepsize)
  Matrix getDataSpeed(const vector<Matrix>& pos, int t, int d) const {
    if(t+(d*stepsize)<0) t=-(d*stepsize);
    if(t+((d-1)*stepsize)<0) t=-((d-1)*stepsize);
    return pos[t+(d*stepsize)] - pos[t+((d-1)*stepsize)];
  }

  // caculates Acceleration at time t+d(stepsize)
  Matrix getDataAcc(const vector<Matrix>& pos, int t, int d) const {
    if(t+(d*stepsize)<0) t=-(d*stepsize);
    if(t+((d-2)*stepsize)<0) t=-((d-2)*stepsize);
    return pos[t+(d*stepsize)] - pos[t+((d-1)*stepsize)]*2 + pos[t+((d-2)*stepsize)];
  }


  // predicts the future
  void predict(list<Matrix>* pred, AbstractModel* modell, const vector<Matrix>& pos, int t, int steps){
    Matrix b[3]={getData(pos,t,-2), getData(pos,t,-1), getData(pos,t,0)};
    for(int i=0; i< steps; i++){
      Matrix input = b[(i+2)%3].above(b[(i+1)%3]).above(b[(i+0)%3]);
      b[i%3]= modell->process(input);
      // b[i%3]= input.rows(0,2);
      pred->push_back(b[i%3]);
    }
  }

  void calcPredPos(vector<Matrix>* pp, const list<Matrix>&pred,
                   const vector<Matrix>& pos, int t){
    calcPredPosFromAcc(pp,pred,pos,t);
  }
  void calcPredPosFromSpeed(vector<Matrix>* pp, const list<Matrix>&pred,
                   const vector<Matrix>& pos, int t){
    assert(pp->size() == pred.size() );
    Matrix point=pos[t];
    int i=0;
    FOREACHC(list<Matrix>, pred, p){
      point += *p;
      (*pp)[i]=point;
      i++;
    }
  }

  void calcPredPosFromAcc(vector<Matrix>* pp, const list<Matrix>&pred,
                   const vector<Matrix>& pos, int t){
    assert(pp->size() == pred.size() );
    Matrix point = pos[t];
    Matrix speed = getDataSpeed(pos,t,0);
    int i=0;
    FOREACHC(list<Matrix>, pred, p){
      speed += *p;
      point += speed;
      (*pp)[i]=point;
      i++;
    }
  }

  double calcError(const list<Matrix>& pred, const vector<Matrix>& pos, int t){
    int i=0;
    double error = 0;
    double size=0;
    FOREACHC( list<Matrix>, pred, p){
      i++;
      error += ((*p) - getData(pos,t,i)).multTM().val(0,0);
      size  += getData(pos,t,i).map(fabs).elementSum();
      // calc length of speed difference
    }
    return error/size;
  }

  virtual iparamkeylist getInternalParamNames() const{
    list<iparamkey> keylist;
    keylist += storeVectorFieldNames(pos[0], "pos");
    keylist += storeVectorFieldNames(pos[0], "data");
    for(unsigned int i=0; i < predpos.size(); i++){
      keylist += storeVectorFieldNames(pos[0], "pred" + itos(i));
    }
    keylist += string("pred_error");
    return keylist;
  }
  virtual iparamvallist getInternalParams() const{
    list<iparamval> l;
    l += pos[max(t-1,100)].convertToList();
    l += getData(pos,max(t-1,100),0).convertToList();
    FOREACHC(vector<Matrix>, predpos, i)
      l += i->convertToList();
    l += (double)pred_error;
    return l;
  }

public:
  vector<Matrix> pos;   // true positions
  vector<Matrix> predpos; // predicted positions
  AbstractModel * mep;
  double pred_error;
  int data_size;
  int t;

  double stepsizeCopy;
  double horizontCopy;
};


// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}

void openPlotOptions(list<PlotOption>& plotoptions , list<const Inspectable*>& inspectables, const vector<Configurable*>& configureables){
  signal(SIGPIPE,SIG_IGN);
  FOREACH(list<PlotOption>, plotoptions, p){
    p->open();
    if(p->pipe){
      // print start
      time_t t = time(0);
      fprintf(p->pipe,"# Start %s", ctime(&t));
      // print interval
      fprintf(p->pipe, "# Recording every %dth dataset\n", p->interval);
      // print all configureables
      FOREACHC (vector<Configurable*>, configureables,i){
        (*i)->print(p->pipe, "# ");
      }

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
  openPlotOptions(plotoptions, inspectables, globaldata.configs);

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
