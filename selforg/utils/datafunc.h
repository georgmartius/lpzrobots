#ifndef __DATAFUNC_H
#define __DATAFUNC_H

#include <vector>
#include "matrix.h"


/// INPUT / Output Data selectors
typedef matrix::Matrix (*DataFunc)(const std::vector<matrix::Matrix>& data, int time);

matrix::Matrix tm1(const std::vector<matrix::Matrix>& data, int time){
  assert(time>0);
  return data[time-1];
}

matrix::Matrix tm2(const std::vector<matrix::Matrix>& data, int time){
  assert(time>1);
  return data[time-2];
}

matrix::Matrix tm12(const std::vector<matrix::Matrix>& data, int time){
  assert(time>1);
  return data[time-1].above(data[time-2]);
}

matrix::Matrix tm123(const std::vector<matrix::Matrix>& data, int time){
  assert(time>2);
  return data[time-1].above(data[time-2].above(data[time-3]));
}

matrix::Matrix tm125(const std::vector<matrix::Matrix>& data, int time){
  assert(time>4);
  return data[time-1].above(data[time-2].above(data[time-5]));
}


matrix::Matrix tm12345(const std::vector<matrix::Matrix>& data, int time){
  assert(time>4);
  return data[time-1].above(data[time-2].above(data[time-3].above(data[time-4].above(data[time-5]))));
}

matrix::Matrix tm12358(const std::vector<matrix::Matrix>& data, int time){
  assert(time>7);
  return data[time-1].above(data[time-2].above(data[time-3].above(data[time-5].above(data[time-8]))));
}

matrix::Matrix tm125_10_20(const std::vector<matrix::Matrix>& data, int time){
  assert(time>19);
  return data[time-1].above(data[time-2].above(data[time-5].above(data[time-10].above(data[time-20]))));
}

matrix::Matrix tm1_to_20(const std::vector<matrix::Matrix>& data, int time){
  assert(time>19);
  matrix::Matrix rv=data[time-1];
  for(int i=2; i<=20; i++){
    rv = rv.above(data[time-i]);
  }
  return rv;
}

// more selective filters

matrix::Matrix ts01m1_to_10(const std::vector<matrix::Matrix>& data, int time){
  assert(time>9);
  matrix::Matrix rv=data[time].rows(0,1);
  for(int i=1; i<=10; i++){
    rv = rv.above(data[time-i]);
  }
  return rv;
}

matrix::Matrix tm0_to_10s01(const std::vector<matrix::Matrix>& data, int time){
  assert(time>9);
  matrix::Matrix rv=data[time].rows(0,1);
  for(int i=1; i<=10; i++){
    rv = rv.above(data[time-i].rows(0,1));
  }
  return rv;
}


matrix::Matrix tm1s01_m2(const std::vector<matrix::Matrix>& data, int time){
  assert(time>2);
  return data[time-1].rows(0,1).above(data[time-2]);
}


matrix::Matrix ts01_m1s23(const std::vector<matrix::Matrix>& data, int time){
  assert(time>2);
  return data[time].rows(0,1).above(data[time-1].rows(2,3));
}

matrix::Matrix tm1s23(const std::vector<matrix::Matrix>& data, int time){
  assert(time>2);
  return data[time-1].rows(2,3);
}



// selector for current time only

matrix::Matrix t(const std::vector<matrix::Matrix>& data, int time){
  assert(time>=0);
  return data[time];
}

matrix::Matrix ts0(const std::vector<matrix::Matrix>& data, int time){
  assert(time>=0);
  return data[time].rows(0,0);
}

matrix::Matrix ts1(const std::vector<matrix::Matrix>& data, int time){
  assert(time>=0);
  return data[time].rows(1,1);
}

matrix::Matrix ts01(const std::vector<matrix::Matrix>& data, int time){
  assert(time>=0);
  return data[time].rows(0,1);
}

matrix::Matrix ts012(const std::vector<matrix::Matrix>& data, int time){
  assert(time>=0);
  return data[time].rows(0,2);
}

matrix::Matrix ts23(const std::vector<matrix::Matrix>& data, int time){
  assert(time>=0);
  return data[time].rows(2,3);
}

matrix::Matrix tsfrom1(const std::vector<matrix::Matrix>& data, int time){
  assert(time>=0);
  return data[time].rows(1,data[time].getM());
}


DataFunc datafunctions(const std::string& name){
  if(name=="tm1") return &tm1;
  if(name=="tm1s23") return &tm1s23;
  if(name=="tm2") return &tm2;
  if(name=="tm12") return &tm12;
  if(name=="tm123") return &tm123;
  if(name=="tm125") return &tm125;
  if(name=="tm12345") return &tm12345;
  if(name=="tm12358") return &tm12358;
  if(name=="tm125_10_20") return &tm125_10_20;
  if(name=="tm1-20") return &tm1_to_20;
  if(name=="ts01m1-10") return &ts01m1_to_10;
  if(name=="tm0-10s01") return &tm0_to_10s01;
  if(name=="tm1-10s01") return &tm0_to_10s01;
  if(name=="tm1-10s01") return &tm0_to_10s01;
  if(name=="tm1s01_m2") return &tm1s01_m2;           
  if(name=="ts01_m1s23") return &ts01_m1s23;
  if(name=="t") return &t;
  if(name=="ts0") return &ts0;
  if(name=="ts1") return &ts1;
  if(name=="ts01") return &ts01;
  if(name=="ts012") return &ts012;
  if(name=="ts23") return &ts23;
  if(name=="ts1-") return &tsfrom1;
  std::cerr << "Unknown Data-function: " << name << std::endl;
  exit(1);  
  return 0;
}

#endif 
