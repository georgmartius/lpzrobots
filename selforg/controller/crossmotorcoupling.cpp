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


#include "crossmotorcoupling.h"

using namespace std;

void CrossMotorCoupling::step(const sensor* sensors, int sensornumber,
                              motor* motors, int motornumber) {

  // teaching
  const matrix::Matrix& m = teachable->getLastMotorValues();
  controller->step(sensors, sensornumber, motors,  motornumber);
  matrix::Matrix teaching = m; // default is to teach with the motor value itself
  for(unsigned int i=0; i< cmc.size(); i++){
    double incom=0;
    FOREACHC(list<int>, cmc[i], src){
      incom += m.val(*src,0);
    }
    if(cmc[i].size()>0){ // only if we have incomming connections...
      incom/= cmc[i].size();
      if(abs(incom)>=threshold)
        teaching.val(i,0)= incom/cmc[i].size();
    }
  }
  if(cmc.size()){
    teachable->setMotorTeaching(teaching);
  }
}


void CrossMotorCoupling::setCMC(const CMC& cmc){
  this->cmc=cmc;
}

CMC CrossMotorCoupling::getCMC(){
  return cmc;
}


CMC CrossMotorCoupling::getPermutationCMC(const std::list<int>& permutation){
  CMC cmc(permutation.size());
  unsigned int i=0;
  FOREACHC(std::list<int>, permutation, p){
    cmc[i].push_back(*p);
    i++;
  }
  return cmc;
}

