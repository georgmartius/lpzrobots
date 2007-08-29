/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2007-08-29 15:32:52  martius
 *   reinforcement learning with 4 wheeled
 *
 *
 *
 ***************************************************************************/

#include "multireinforce.h"

using namespace matrix;
using namespace std;


Sat::Sat(MultiLayerFFNN* _net, double _eps){
  net=_net;
  eps=_eps;
  lifetime=0;
}


MultiReinforce::MultiReinforce( const MultiReinforceConf& _conf) 
  : AbstractController("MultiReinforce", "$Id: "), buffersize(_conf.buffersize), conf(_conf)
{  
  assert(conf.qlearning);
  managementInterval=100;
  manualControl=false;
  addParameter("tauE",&conf.tauE1);
  addParameter("tauH",&conf.tauH);
  addParameter("tauI",&conf.tauI);
  initialised = false;  
};


MultiReinforce::~MultiReinforce()
{
  if(x_buffer && y_buffer && xp_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] xp_buffer;
    delete[] x_context_buffer;
  }
  FOREACH(vector<Sat>, sats, s){
    if(s->net) delete s->net;
  }
  if(conf.qlearning) delete conf.qlearning;
}


void MultiReinforce::init(int sensornumber, int motornumber){

  number_motors  = motornumber;
  number_sensors = sensornumber;  
  int number_real_sensors = number_sensors - conf.numContext;

  x_buffer = new Matrix[buffersize];
  xp_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  x_context_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_real_sensors,1);
    xp_buffer[k].set(2*number_real_sensors,1);
    y_buffer[k].set(number_motors,1);
    x_context_buffer[k].set(conf.numContext,1);
  }

  conf.numSats = conf.satFiles.size();
  restoreSats(conf.satFiles);
  
  satErrors.set(conf.numSats, 1);
  satAvgErrors.set(conf.numSats, 1);
  statesbins.set(getStateNumber(),1);

  
  conf.qlearning->init(getStateNumber(), conf.numSats);

  action=0;
  state=0;
  phase=0;
  reward=0;
  phasecnt=0;
  t=0;
  initialised = true;
}

// put new value in ring buffer
void MultiReinforce::putInBuffer(matrix::Matrix* buffer, const matrix::Matrix& vec, int delay){
  buffer[(t-delay)%buffersize] = vec;
}

/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void MultiReinforce::step(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{
  bool phasechanged=false;
  //  int oldstate = state;
  phasecnt++;
  fillSensorBuffer(x_, number_sensors);  
  if(t>buffersize) {   
    statesbins*=(1-1/conf.tauE1);
    statesbins.val(calcState(),0)+=1.0;
    int newstate = argmax(statesbins);
    if (newstate != state) { 
      statesbins.val(newstate,0)+=conf.tauH;//hystersis
      phase++;
      phasecnt=0;
    phasechanged=true;
      state=newstate;
    }
    // reward is collected over the interval tauH and averaged
    reward = (1.0- 1.0/conf.tauH)*reward + (1.0/conf.tauH)*calcReinforcement();

    switch(phase){
    case 0: // do nothing (but counting)
      if(phasecnt > conf.tauI){
	phase++;
	phasecnt=0;
      }
      break;
    case 1: // update q table
      if(phasecnt%(int)conf.tauH)
	conf.qlearning->learn(state,action,reward,1);
      if(phasecnt > conf.tauI){ // enable exploration
	phase++;
      }
      break;
    case 2: // select a new action
      if(!manualControl)
	action = conf.qlearning->select(state);
    default:
      phase=0;
      break;
    }

    if(t%managementInterval==0){
      management();
    }

    /// sat network control
    const Matrix& x_t   = x_buffer[t%buffersize];
    if(conf.useDerive){
      const Matrix& xp_t  = xp_buffer[t%buffersize];
      satInput   = x_t.above(xp_t);
    } else {
      const Matrix& x_tm1 = x_buffer[(t-1)%buffersize];
      satInput   = x_t.above(x_tm1);
    }
    if(conf.useY){
      const Matrix& y_tm1 = y_buffer[(t-1)%buffersize];
      satInput.toAbove(y_tm1);
    }

    const Matrix& out = sats[action].net->process(satInput);
    const Matrix& y_sat = out.rows(x_t.getM(), out.getM()-1);
    y_sat.convertToBuffer(y_, number_motors); // store the values into y_ array
  }else{
    memset(y_,0,sizeof(motor)*number_motors);
  }
  fillMotorBuffer(y_, number_motors); // store the plain c-array "_y" into the y buffer
  
  // update step counter
  t++;
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void MultiReinforce::stepNoLearning(const sensor* x, int number_sensors, motor*  y, int number_motors )
{
  fillSensorBuffer(x, number_sensors);
  memset(y,0,sizeof(motor)*number_motors); // fixme
  fillMotorBuffer(y, number_motors);
  // update step counter
  t++;
};


void MultiReinforce::fillSensorBuffer(const sensor* x_, int number_sensors)
{
  assert((unsigned)number_sensors == this->number_sensors);
  Matrix x(number_sensors-conf.numContext, 1, x_);
  Matrix x_c(conf.numContext, 1, x_+number_sensors-conf.numContext);
  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);  
  if(conf.useDerive){
    const Matrix& xp = calcDerivatives(x_buffer,0);
    putInBuffer(xp_buffer, xp);    
  }
  putInBuffer(x_context_buffer, x_c);  
}

void MultiReinforce::fillMotorBuffer(const motor* y_, int number_motors)
{
  assert((unsigned)number_motors == this->number_motors);
  Matrix y(number_motors,1,y_);
  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);  
}

void MultiReinforce::setManualControl(bool mControl, int action_){  
  if(mControl)
    action=clip(action_,0,conf.numSats-1);
  manualControl=mControl;
}
  
Matrix MultiReinforce::calcDerivatives(const matrix::Matrix* buffer,int delay){  
  int t1 = t+buffersize;
  const Matrix& xt    = buffer[(t1-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t1-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t1-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

void MultiReinforce::management(){
  conf.qlearning->collectedReward*=(1-managementInterval/(10000.0));
}


Configurable::paramval MultiReinforce::getParam(const paramkey& key) const{
  if (key=="mancontrol") return (double)manualControl;
  else if (key=="action") return (double)action;
  else if (key=="interval") return (double)conf.reinforce_interval;
  else return AbstractController::getParam(key);
}


bool MultiReinforce::setParam(const paramkey& key, paramval val){
  if(key=="mancontrol") {
    setManualControl(val!=0);
    return true;
  }else
  if(key=="action") {
    setManualControl(manualControl, (int)val);
    return true;
  }else   
  if(key=="interval") {
    conf.reinforce_interval = max((int)val,1);
    return true;
  }else   
  return AbstractController::setParam(key, val);
}

Configurable::paramlist MultiReinforce::getParamList() const{
  paramlist keylist = AbstractController::getParamList();
  keylist += pair<paramkey, paramval>("mancontrol",(double)manualControl);
  keylist += pair<paramkey, paramval>("action",(double)action);
  keylist += pair<paramkey, paramval>("interval",(double)conf.reinforce_interval);
  return keylist;
} 


bool MultiReinforce::store(FILE* f) const {  
  fprintf(f,"%i\n", conf.numSats);
  fprintf(f,"%i\n", conf.numContext);

  // save matrix values
  satErrors.store(f);
  satAvgErrors.store(f);

  // store sats
  FOREACHC(vector<Sat>, sats, s){
    s->net->store(f);
  }
 
  // save config and controller
  Configurable::print(f,0);
  conf.qlearning->store(f);
  return true;
}

bool MultiReinforce::restore(FILE* f){
  if(!initialised)
    init(2,2);

  char buffer[128];
  if(fscanf(f,"%s\n", buffer) != 1) return false;	
  conf.numSats = atoi(buffer);
 // we need to use fgets in order to avoid spurious effects with following matrix (binary)
  if((fgets(buffer,128, f))==NULL) return false;
  conf.numContext = atoi(buffer);

  // restore matrix values
  satErrors.restore(f);
  satAvgErrors.restore(f);

  // clean sats array
  sats.clear();
  // restore sats
  for(int i=0; i < conf.numSats; i++){ 
    MultiLayerFFNN* n = new MultiLayerFFNN(0,vector<Layer>());
    n->restore(f);
    sats.push_back(Sat(n,n->eps));
  }
 
  // save config and controller
  Configurable::parse(f);
  conf.qlearning->restore(f);
  t=0; // set time to zero to ensure proper filling of buffers  
  action=0;
  state=0;
  phase=0;
  reward=0;
  return true;
}

void MultiReinforce::restoreSats(const list<string>& files){
  assert(conf.numSats == (signed)files.size());
  assert(conf.numSats);
  sats.clear();
  vector<Layer> l;
  FILE* file;
  FOREACHC(list<string>, files,f){
    file = fopen(f->c_str(),"rb");
    if(!file) {
      cerr << "MultiReinforce::restoreSats: " << f->c_str();
      perror(0);
      exit(1);
    }  
    MultiLayerFFNN* net = new MultiLayerFFNN(1.0,l,false);
    net->restore(file);    
    fclose(file);
    sats.push_back(Sat (net,1));
  } 
}


list<string> MultiReinforce::createFileList(const char* filestem, int n){
  list<string> fs;
  for(int i=0; i< n; i++){
    char fname[256];
    sprintf(fname,"%s_%02i.net", filestem, i);
    fs.push_back(string(fname));    
  }
  return fs;
}

list<Inspectable::iparamkey> MultiReinforce::getInternalParamNames() const {
  list<iparamkey> keylist;
  
  keylist += storeVectorFieldNames(x_context_buffer[0], "XC");
  keylist += storeVectorFieldNames(satErrors, "errs");
  keylist += storeVectorFieldNames(satAvgErrors, "avgerrs");
  keylist += storeVectorFieldNames(statesbins, "statesbins");
  keylist += string("action");
  keylist += string("state");
  keylist += string("phase");
  keylist += string("reward");
  keylist += string("coll_rew");
  return keylist; 
}

list<Inspectable::iparamval> MultiReinforce::getInternalParams() const {
  list<iparamval> l;
  l += x_context_buffer[t%buffersize].convertToList();
  l += satErrors.convertToList();
  l += satAvgErrors.convertToList();
  l += statesbins.convertToList();
  l += (double)action;
  l += (double)state;
  l += (double)phase;
  l += (double)reward;
  l += conf.qlearning->collectedReward;
  return l;
}

list<Inspectable::ILayer> MultiReinforce::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
//   l+=ILayer("x","", number_sensors, 0, "Sensors");
//   l+=ILayer("y","H", number_motors, 1, "Motors");
//   l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> MultiReinforce::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
//   l+=IConnection("C", "x", "y");
//   l+=IConnection("A", "y", "xP");
//   if(conf.useS) l+=IConnection("S", "x", "xP"); // this is not quite true! it is x' x'' -> xp
  return l;
}
