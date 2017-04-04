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

#include "randomdyn.h"

using namespace matrix;
using namespace std;

RandomDyn::RandomDyn(const RandomDynConf& conf_)
  : AbstractController("RandomDyn", "0.2"),
    conf(conf_), t(0)
{

  addParameterDef("sigmaC", &sigmaC, 0.001, 0,5, "standard dev. for diagonal elements update");
  addParameterDef("sigmah", &sigmah, 0.001, 0,5, "standard dev. for bias update");
  addParameterDef("damping",   &damping,     0.001, 0,0.1,
                  "forgetting term for controller to initial values");

  addInspectableMatrix("C", &C, conf.someInternalParams, "controller matrix");
  addInspectableMatrix("h", &h, conf.someInternalParams, "controller bias");
};

RandomDyn::~RandomDyn(){
  if(conf.noiseGenC) delete conf.noiseGenC;
  if(conf.noiseGenh) delete conf.noiseGenh;
  if(randGenC) delete randGenC;
  if(randGenh) delete randGenh;
}

void RandomDyn::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  randGenC=new RandGen();
  randGenC->init(randGen->rand()*100000.0);
  randGenh=new RandGen();
  randGenh->init(randGen->rand()*100000.0);

  if(!conf.noiseGenC){
    conf.noiseGenC = new ColorUniformNoise(0.005);
  }
  if(!conf.noiseGenh){
    conf.noiseGenh = new ColorUniformNoise(0.01);
  }
  conf.noiseGenC->init(sensornumber*motornumber,randGenC);
  conf.noiseGenh->init(motornumber,randGenh);

  number_sensors= sensornumber;
  number_motors = motornumber;
  C.set(number_motors, number_sensors);
  h.set(number_motors, 1);

  C_native.set(number_motors, number_sensors);
  C.toId(); // set a to identity matrix;
  C*=conf.initFeedbackStrength;

  // if motor babbling is used then this is overwritten
  C_native.toId();
  C_native*=conf.initFeedbackStrength;

}

matrix::Matrix RandomDyn::getC(){
  return C;
}

void RandomDyn::setC(const matrix::Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}

matrix::Matrix RandomDyn::geth(){
  return h;
}

void RandomDyn::seth(const matrix::Matrix& _h){
  assert(h.getM() == _h.getM() && h.getN() == _h.getN());
  h=_h;
}

// performs one step (includes learning). Calculates motor commands from sensor inputs.
void RandomDyn::step(const sensor* s_, int number_sensors,
                       motor* a_, int number_motors){
  stepNoLearning(s_, number_sensors, a_, number_motors);
  t--; // stepNoLearning increases the time by one - undo here

  update();

  // update step counter
  t++;
};


// performs one step without learning. Calulates motor commands from sensor inputs.
void RandomDyn::stepNoLearning(const sensor* s_, int number_sensors,
                                 motor* a_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors
         && (unsigned)number_motors <= this->number_motors);

  s.set(number_sensors,1,s_); // store sensor values

  // calculate controller values based on current input values (smoothed)
  Matrix a =   (C*s + h).map(g);

  // convert a to motor*
  a.convertToBuffer(a_, number_motors);

  // update step counter
  t++;
};


void RandomDyn::motorBabblingStep(const sensor* s_, int number_sensors,
                            const motor* a_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors
         && (unsigned)number_motors <= this->number_motors);
  // not implemented!
  t++;
}


// update values h,C
void RandomDyn::update(){
  if(sigmaC !=0){
    C += noiseMatrix(C.getM(), C.getN(), *conf.noiseGenC, sigmaC);
    if(damping)
      C += (C_native-C)*damping;
  }
  if(sigmah != 0){
    h += noiseMatrix(h.getM(), h.getN(), *conf.noiseGenh, sigmah);
    if(damping)
      h *= (1.0-damping);
  }
};

/* stores the controller values to a given file. */
bool RandomDyn::store(FILE* f) const{
  // save matrix values
  C.store(f);
  h.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool RandomDyn::restore(FILE* f){
  // save matrix values
  C.restore(f);
  h.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

