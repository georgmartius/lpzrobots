#ifndef __ODECONFIG_H
#define __ODECONFIG_H

#include <ode/ode.h>
#include "configurable.h"
#include "odehandle.h"

class OdeConfig : public Configurable {
public:
  OdeConfig(const OdeHandle& odehandle)
    : odeHandle(odehandle) {
    simStepSize=0.01;
    controlInterval=5;
    realTimeFactor=1.0;
    noise=0.1;
    gravity=-9.81;
    drawInterval=calcDrawInterval();
    // prepare name;
    strcpy(name,"Simulation Environment: ");
    Configurable::insertCVSInfo(name + strlen(name), "$RCSfile$", "$Revision$");
  }

  virtual ~OdeConfig(){}

  virtual constparamkey getName() const {
    return name;
  }

  virtual int getParamList(paramkey*& keylist,paramval*& vallist) const {
    int number_params=6;
    int i=0;
    keylist=(paramkey*)malloc(sizeof(paramkey)*number_params);
    vallist=(paramval*)malloc(sizeof(paramval)*number_params);
    keylist[i++]="noise";
    keylist[i++]="simstepsize";
    keylist[i++]="realtimefactor";
    keylist[i++]="drawinterval";
    keylist[i++]="controlinterval";
    keylist[i++]="gravity";
    i=0;
    vallist[i++]=noise;
    vallist[i++]=simStepSize;
    vallist[i++]=realTimeFactor;
    vallist[i++]=drawInterval;
    vallist[i++]=controlInterval;
    vallist[i++]=gravity;
    return number_params;
  }

  paramval getParam(const paramkey key) const {
    if(!key) return 0.0;
    if(strcmp(key, "noise")==0) return noise; 
    else if(strcmp(key, "simstepsize")==0) return simStepSize; 
    else if(strcmp(key, "realtimefactor")==0) return realTimeFactor; 
    else if(strcmp(key, "drawinterval")==0) return drawInterval; 
    else if(strcmp(key, "controlinterval")==0) return controlInterval; 
    else if(strcmp(key, "gravity")==0) return gravity; 
    else  return 0.0;
  }
        
  bool setParam(const paramkey key, const paramval val){
    if(!key) return false;
    if(strcmp(key, "noise")==0) noise = val; 
    else if(strcmp(key, "simstepsize")==0) {
      simStepSize=max(0.0000001,val); 
      drawInterval=calcDrawInterval();
    }else if(strcmp(key, "realtimefactor")==0){
      realTimeFactor=max(0.0,val); 
      drawInterval=calcDrawInterval();
    }
    else if(strcmp(key, "drawinterval")==0) drawInterval=(int)val; 
    else if(strcmp(key, "controlinterval")==0) controlInterval=(int)val; 
    else if(strcmp(key, "gravity")==0) {
      gravity=val; 
      dWorldSetGravity ( odeHandle.world , 0 , 0 , gravity );
    }
    else  return false;
    return true;
  }

private:
  /// calculates the draw interval with simStepSize and realTimeFactor so that we have 25 frames/sec
  int calcDrawInterval(){
    if(realTimeFactor>0 && simStepSize>0){
      return int(ceil(1/(25.0*simStepSize/realTimeFactor)));
    }else return 50;
  }


public:
  double simStepSize;
  double realTimeFactor;
  int drawInterval;
  int controlInterval;
  double noise;
  double gravity;
  char name[100];
  OdeHandle odeHandle;
};

#endif
