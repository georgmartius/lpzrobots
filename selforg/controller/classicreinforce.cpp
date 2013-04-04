/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/

#include "classicreinforce.h"
#include <selforg/controller_misc.h>

using namespace matrix;
using namespace std;



ClassicReinforce::ClassicReinforce( const ClassicReinforceConf& _conf)
  : AbstractController("ClassicReinforce", "$Id: "), buffersize(_conf.buffersize), conf(_conf)
{
  assert(conf.qlearning);
  managementInterval=100;
  initialised = false;

  addParameterDef("mancontrol",&manualControl, false, "Manual control used, see action");
  addParameterDef("action",&action, 0, "action to be used in manual control");
  addParameter("interval",&conf.reinforce_interval, "interval between reinforcement steps");
};


ClassicReinforce::~ClassicReinforce()
{
  if(x_buffer && y_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] x_context_buffer;
  }
  if(conf.qlearning) delete conf.qlearning;
}


void ClassicReinforce::init(int sensornumber, int motornumber, RandGen* randGen){
  //if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  number_motors  = motornumber;
  number_sensors = sensornumber;
  int number_real_sensors = number_sensors - conf.numContext;

  x_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  x_context_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_real_sensors,1);
    y_buffer[k].set(number_motors,1);
    x_context_buffer[k].set(conf.numContext,1);
  }
  assert(conf.qlearning && "Please set qlearning in controller configuration");
  conf.qlearning->init(getStateNumber(), getActionNumber());

  action=0;
  oldaction=0;
  state=0;
  reward=0;
  oldreward=0;
  t=0;
  initialised = true;
}

// put new value in ring buffer
void ClassicReinforce::putInBuffer(matrix::Matrix* buffer, const matrix::Matrix& vec, int delay){
  buffer[(t-delay)%buffersize] = vec;
}


/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void ClassicReinforce::step(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{
  double slidingtime=min(4.0,(double)conf.reinforce_interval/2);
  fillSensorBuffer(x_, number_sensors);
  if(t>buffersize) {
    if(t%managementInterval==0){
      management();
    }
    reward += calcReinforcement() / (double)conf.reinforce_interval;
    if((t%conf.reinforce_interval)==0){
      conf.qlearning->learn(state,action,reward,1);
      state = calcState();
      oldreward=reward;
      reward=0;
      if(!manualControl){// select a new action
        oldaction = action;
        action = conf.qlearning->select(state);
        //action = conf.qlearning->select_sample(state);
        //newaction = conf.qlearning->select_keepold(state);
      }
    }

    const Matrix& y = calcMotor(action);
    assert(((signed)y.getM())==number_motors);
    int ts = t%conf.reinforce_interval;
    if(ts<slidingtime && action != oldaction){
      // mixture of old and new actions
      const Matrix& y_o = calcMotor(oldaction);
      // store the values into y_ array
      (y_o*(1-(ts/slidingtime)) +
       y*(ts/slidingtime)).convertToBuffer(y_, number_motors);
    }else{
      y.convertToBuffer(y_, number_motors); // store the values into y_ array
    }
  }else{
    memset(y_,0,sizeof(motor)*number_motors);
  }
  fillMotorBuffer(y_, number_motors); // store the plain c-array "_y" into the y buffer

  // update step counter
  t++;
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void ClassicReinforce::stepNoLearning(const sensor* x, int number_sensors, motor*  y, int number_motors )
{
  fillSensorBuffer(x, number_sensors);
  memset(y,0,sizeof(motor)*number_motors); // fixme
  fillMotorBuffer(y, number_motors);
  // update step counter
  t++;
};


void ClassicReinforce::fillSensorBuffer(const sensor* x_, int number_sensors)
{
  assert((unsigned)number_sensors == this->number_sensors);
  Matrix x(number_sensors-conf.numContext, 1, x_);
  Matrix x_c(conf.numContext, 1, x_+number_sensors-conf.numContext);
  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);
  putInBuffer(x_context_buffer, x_c);
}

void ClassicReinforce::fillMotorBuffer(const motor* y_, int number_motors)
{
  assert((unsigned)number_motors == this->number_motors);
  Matrix y(number_motors,1,y_);
  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);
}

void ClassicReinforce::setManualControl(bool mControl, int action_){
  if(mControl){
    action = clip(action_,0,getActionNumber()-1);
    oldaction=action;
  }
  manualControl=mControl;
}


void ClassicReinforce::management(){
}


void ClassicReinforce::notifyOnChange(const paramkey& key){
  if(key=="mancontrol") {
    setManualControl(manualControl);
  }else
  if(key=="action") {
    setManualControl(manualControl, action);
  }else
  if(key=="interval") {
    conf.reinforce_interval = max(conf.reinforce_interval,1);
  }
}


bool ClassicReinforce::store(FILE* f) const {
  fprintf(f,"%i\n", conf.numContext);

  // save config and controller
  Configurable::print(f,0);
  conf.qlearning->store(f);
  return true;
}

bool ClassicReinforce::restore(FILE* f){
  if(!initialised)
    init(2,2);

  char buffer[128];
 // we need to use fgets in order to avoid spurious effects with following matrix (binary)
  if((fgets(buffer,128, f))==NULL) return false;
  conf.numContext = atoi(buffer);

  // save config and controller
  Configurable::parse(f);
  conf.qlearning->restore(f);
  t=0; // set time to zero to ensure proper filling of buffers
  action=0;
  oldaction=0;
  state=0;
  reward=0;
  return true;
}


list<Inspectable::iparamkey> ClassicReinforce::getInternalParamNames() const {
  list<iparamkey> keylist;

  keylist += storeVectorFieldNames(x_context_buffer[0], "XC");
  keylist += string("action");
  keylist += string("state");
  keylist += string("reward");
  keylist += string("coll_rew");
  return keylist;
}

list<Inspectable::iparamval> ClassicReinforce::getInternalParams() const {
  list<iparamval> l;
  l += x_context_buffer[t%buffersize].convertToList();
  l += (double)action;
  l += (double)state;
  l += (double)oldreward;
  l += conf.qlearning->getCollectedReward();
  return l;
}

list<Inspectable::ILayer> ClassicReinforce::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  return l;
}

list<Inspectable::IConnection> ClassicReinforce::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  return l;
}
