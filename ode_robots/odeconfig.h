#ifndef __ODECONFIG_H
#define __ODECONFIG_H

#include <configurable.h>

class OdeConfig : public Configurable {
public:
  OdeConfig(){
    simStepSize=0.01;
    drawInterval=1;
    controlInterval=5;
    noise=0.1;
    gravity=-9.81;
    // prepare name;
    strcpy(name,"Simulation Environment: ");
    Configurable::insertCVSInfo(name + strlen(name), "$RCSfile$", "$Revision$");
  }

  virtual constparamkey getName() const {
    return name;
  }

  virtual int getParamList(paramkey*& keylist,paramval*& vallist) const {
    int number_params=5;
    keylist=(paramkey*)malloc(sizeof(paramkey)*number_params);
    vallist=(paramval*)malloc(sizeof(paramval)*number_params);
    keylist[0]="noise";
    keylist[1]="simstepsize";
    keylist[2]="drawinterval";
    keylist[3]="controlinterval";
    keylist[4]="gravity";
                    
    vallist[0]=noise;
    vallist[1]=simStepSize;
    vallist[2]=drawInterval;
    vallist[3]=controlInterval;
    vallist[4]=gravity;
    return number_params;
  }

  paramval getParam(const paramkey key) const {
    if(!key) return 0.0;
    if(strcmp(key, "noise")==0) return noise; 
    else if(strcmp(key, "simstepsize")==0) return simStepSize; 
    else if(strcmp(key, "drawinterval")==0) return drawInterval; 
    else if(strcmp(key, "controlinterval")==0) return controlInterval; 
    else if(strcmp(key, "gravity")==0) return gravity; 
    else  return 0.0;
  }
        
  bool setParam(const paramkey key, const paramval val){
    if(!key) return false;
    if(strcmp(key, "noise")==0) noise = val; 
    else if(strcmp(key, "simstepsize")==0) simStepSize=val; 
    else if(strcmp(key, "drawinterval")==0) drawInterval=(int)val; 
    else if(strcmp(key, "controlinterval")==0) controlInterval=(int)val; 
    else if(strcmp(key, "gravity")==0) {
      gravity=val; 
      dWorldSetGravity ( world , 0 , 0 , gravity );
    }
    else  return false;
    return true;
  }

public:
  double simStepSize;
  int drawInterval;
  int controlInterval;
  double noise;
  double gravity;
  char name[100];
};

#endif
