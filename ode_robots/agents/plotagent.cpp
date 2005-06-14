#include "plotagent.h"

Agent::PlotAgent(): Agent() { 
  pipe=0;
  numberInternalParameters=0;
}

void PlotAgent::OpenGui(){
    pipe=popen("guilogger > /dev/null 2>/dev/null","w");
}

void PlotAgent::CloseGui(){
    pclose(pipe);
}

bool PlotAgent::init(AbstractController* controller, AbstractRobot* robot){
  if(!Agent::init(controller, robot)) return false;  
  if(!OpenGui()) return false;
  ///// send channel names to GUI
  // first the sensor and motor channels
  int s = robot->getSensorNumber();
  int m = robot->getMotorNumber();
  fprintf(pipe,"#C");
  for(int i = 0; i < s; i++){
    fprintf(pipe, " x[%i]", i);
  }
  for(int i = 0; i < m; i++){
    fprintf(pipe, " y[%i]", i);
  }
  // then the internal parameters
  paramkey keys;
  numberInternalParameters = controller->getInternalParamNames(keys);
  for(int i = 0; i < numberInternalParameters; i++){
    fprintf(pipe, " %s", keys[i]);
  }
  fprintf(pipe,"\n"); // terminate line
  fflush(pipe);
  return true;
}
void PlotAgent::plot(const sensor* x, int sensornumber, const motor* y, int motornumber){
  if(!controller || !pipe || !x || !y) return;
  if(sensornumber!=robot->getSensorNumber()) {
    fprintf(stderr, "%s:%i: Given sensor number does not match the one from robot!\n", 
	    __FILE__, __LINE__);
  }
  if(motornumber!=robot->getMotorNumber()) {
    fprintf(stderr, "%s:%i: Given motor number does not match the one from robot!\n", 
	    __FILE__, __LINE__);
  }
  for(int i = 0; i < sensornumber; i++){
    fprintf(pipe, " %f", x[i]);
  }
  for(int i = 0; i < motornumber; i++){
    fprintf(pipe, " %f", y[i]);
  }
  // internal parameters ( we allocate one place more to be able to realise when the number raises)
  paramval* vals = (paramval*)malloc(sizeof(paramval) * numberInternalParameters+1);
  int len = controller->getInternalParams(vals, numberInternalParameters+1);
  if(len!= numberInternalParameters) {
    fprintf(stderr, "%s:%i: Weird! Internal parameter number seems to have changed!\n", 
	    __FILE__, __LINE__);
  }
  for(int i=0; i<len; i++){
    fprintf(pipe, " %f", vals[i]);
  }

  fprintf(pipe,"\n"); // terminate line
  fflush(pipe);
  free(paramval);
};



/// converts integer to string
std::string PlotAgent::intToStr(int value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
};
