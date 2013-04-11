/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include "qlearning.h"
#include "controller_misc.h"


QLearning::QLearning(double eps, double discount, double exploration, int eligibility,
                     bool random_initQ, bool useSARSA, int tau)
  : Configurable("QLearning", "$Id$"),
    eps(eps), discount(discount),  exploration(exploration),
    eligibility(eligibility), random_initQ(random_initQ), useSARSA(useSARSA), tau(tau){
  if(eligibility<1) eligibility=1;
  ringbuffersize = eligibility+1;
  actions  = new int[ringbuffersize];
  states   = new int[ringbuffersize];
  rewards  = new double[ringbuffersize];
  longrewards = new double[tau];
  memset(actions,0,sizeof(int)*ringbuffersize);
  memset(states,0,sizeof(int)*ringbuffersize);
  memset(rewards,0,sizeof(double)*ringbuffersize);
  memset(longrewards,0,sizeof(double)*tau);
  t=0;
  collectedReward = 0;
  initialised=false;

  addParameter("eps",&this->eps);
  addParameter("discount",&this->discount);
  addParameter("expl",&this->exploration);
  addParameter("elig",&this->eligibility);
}

QLearning::~QLearning(){
  if(actions) delete[] actions;
  if(states) delete[] states;
  if(rewards) delete[] rewards;
  if(longrewards) delete[] longrewards;
};

void QLearning::init(unsigned  int stateDim, unsigned int actionDim, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
 this->randGen=randGen;
  Q.set(stateDim,actionDim);
  if(random_initQ){
    Q= Q.mapP(randGen, random_minusone_to_one)*0.01;
  }
  initialised=true;
}

unsigned int QLearning::select (unsigned int state){
  assert(initialised);
  assert(state < Q.getM());

  matrix::Matrix as = Q.row(state);
  as += as.mapP(randGen, random_minusone_to_one)*0.001; // this is like random
                                              // walk if we know nothing
  int a= argmax(as);
  // exploration
  double r = randGen->rand();
  if(r<exploration){
    a = (int)(randGen->rand()*(double)Q.getN());
  }
  return a;
}

unsigned int QLearning::select_sample (unsigned int state){
  assert(initialised);
  assert(state < Q.getM());
  matrix::Matrix vals = Q.row(state);
  std::cout << "****\n" << vals << std::endl;
  // subtract mean
  double m = - vals.elementSum() / vals.size();
  vals.toMapP(m,plus_);
  // add bias to old action
  vals.val(0,  actions[(t-1)%ringbuffersize])+=m/2.0;
  // cut below mean
  double theta=0;
  vals.toMapP(&theta,lowercutof);
  vals *= (1/vals.map(fabs).elementSum()); // normalize

  // add exploration and bias to old action
  double e = exploration/vals.size();
  vals.toMapP(e,plus_);
  vals *= (1/vals.map(fabs).elementSum()); // normalize
  std::cout  << vals << std::endl;

  return sample(vals);
}

unsigned int QLearning::select_keepold (unsigned int state){
  assert(initialised);
  assert(state < Q.getM());

  // exploration
  double r = randGen->rand();
  if(r<exploration){
    std::cout << "explore\n";
    return int(randGen->rand()*(double)Q.getN());
  }
  matrix::Matrix vals = Q.row(state);
  vals += vals.mapP(randGen, random_minusone_to_one)*0.001; // this is like random
                                                  // walk if we know nothing
  double m = vals.elementSum() / vals.size();
  r = randGen->rand();
  // keep to 80% old if acceptable
  if(vals.val(0,  actions[(t-1)%ringbuffersize])>m && r<0.8){
    std::cout << "keepold\n";
    return actions[(t-1)%ringbuffersize];
  }else {
    int a = argmax(vals);
    // select to 30% second best
    r = randGen->rand();
    if(r<0.3){
      std::cout << "second best\n";
      vals.val(0,a)-=1000;
      return argmax(vals);
    }else {
      std::cout << "best\n";
      return a;
    }
  }
  return sample(vals);
}


matrix::Matrix QLearning::getActionValues(unsigned int state){
  return Q.row(state);
}


double QLearning::learn (unsigned int state,
                         unsigned int action,
                         double reward,
                         double learnRateFactor){
  assert(initialised);
  actions[t%ringbuffersize] = action;
  states[t%ringbuffersize]  = state;
  rewards[t%ringbuffersize] = reward;
  collectedReward -= longrewards[t%tau];
  longrewards[t%tau] = reward;
  collectedReward += reward;
  // Todo: eligibility traces are wrong! see the book of sutton how to do it!
  // learn for previous eligibility steps
  double e_factor = 1; // learning factor for eligibility
  for(int i=1; i<=eligibility; i++){
    if(t-i <0) break;
    int a_t     = actions[(t-i)   %ringbuffersize];
    int a_tp1   = actions[(t-i+1) %ringbuffersize];
    int s_t     = states [(t-i)   %ringbuffersize];
    int s_tp1   = states [(t-i+1) %ringbuffersize];
    double r_t  = rewards[(t-i)   %ringbuffersize];
    double e    = eps*e_factor * learnRateFactor;  // local learning rate
    if(useSARSA)
      Q.val(s_t,a_t) += e*(r_t + discount*Q.val(s_tp1,a_tp1) - Q.val(s_t,a_t));
    else
      Q.val(s_t,a_t) += e*(r_t + discount*max(Q.row(s_tp1)) - Q.val(s_t,a_t));
    e_factor -= 1.0/eligibility;
  }
  t++;
  return Q.val(state,action);
}

void QLearning::reset(){
  t=0;
}

unsigned int QLearning::getStateDim() const{
  return Q.getM();
}

unsigned int QLearning::getActionDim() const{
  return Q.getN();
}

/// returns the collectedReward reward
double QLearning::getCollectedReward() const{
  return collectedReward/(std::min(t,tau));
}


int QLearning::valInCrossProd(const std::list<std::pair<int,int> >& vals){
  int fac = 1;
  int val = 0;
  typedef std::list<std::pair<int,int> > setlist;
  FOREACHC(setlist , vals, v){
    assert(v->first<v->second);
    val+=v->first*fac;
    fac*=v->second;
  }
  return val;
}

std::list<int> QLearning::ConfInCrossProd(const std::list<int>& ranges, int val){
  std::list<int> cfg;

  for(std::list<int>::const_reverse_iterator r=ranges.rbegin(); r!=ranges.rend(); r++){
    cfg.push_front(val%*r);
    val/=*r;
  }
  return cfg;
}



bool QLearning::store(FILE* f) const{
  Q.store(f);
  Configurable::print(f,0);
  return true;
}

bool QLearning::restore(FILE* f) {
  Q.restore(f);
  Configurable::parse(f);
  t=0;
  collectedReward = 0;
  initialised=true;
  return true;
}

