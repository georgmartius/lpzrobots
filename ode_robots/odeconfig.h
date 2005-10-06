#ifndef __ODECONFIG_H
#define __ODECONFIG_H

#include <ode/ode.h>
#include "configurable.h"
#include "odehandle.h"

class OdeConfig : public Configurable {
public:
  OdeConfig() :
    name ("Simulation Environment: ")
  {
    simStepSize=0.01;
    controlInterval=5;
    realTimeFactor=1.0;
    noise=0.1;
    gravity=-9.81;
    drawInterval=calcDrawInterval();
    // prepare name;
    Configurable::insertCVSInfo(name, "$RCSfile$", "$Revision$");
  }

  virtual ~OdeConfig(){}

  virtual paramkey getName() const {
    return name;
  }

  virtual paramlist getParamList() const{
    paramlist list;
    list.push_back(pair<paramkey, paramval> (string("noise"), noise));
    list.push_back(pair<paramkey, paramval> (string("simstepsize"), simStepSize));
    list.push_back(pair<paramkey, paramval> (string("realtimefactor"), realTimeFactor));
    list.push_back(pair<paramkey, paramval> (string("drawinterval"), drawInterval));
    list.push_back(pair<paramkey, paramval> (string("controlinterval"), controlInterval));
    list.push_back(pair<paramkey, paramval> (string("gravity"), gravity));
    return list;
  } 


  paramval getParam(const paramkey& key) const {
    if(key == "noise") return noise; 
    else if(key == "simstepsize") return simStepSize; 
    else if(key == "realtimefactor") return realTimeFactor; 
    else if(key == "drawinterval") return drawInterval; 
    else if(key == "controlinterval") return controlInterval; 
    else if(key == "gravity") return gravity; 
    else  return 0.0;
  }
        
  bool setParam(const paramkey& key, paramval val){
    if(key == "noise") noise = val; 
    else if(key == "simstepsize") {
      simStepSize=max(0.0000001,val); 
      drawInterval=calcDrawInterval();
    }else if(key == "realtimefactor"){
      realTimeFactor=max(0.0,val); 
      drawInterval=calcDrawInterval();
    }
    else if(key == "drawinterval") drawInterval=(int)val; 
    else if(key == "controlinterval") controlInterval=(int)val; 
    else if(key == "gravity") {
      gravity=val; 
      dWorldSetGravity ( odeHandle.world , 0 , 0 , gravity );
    }
    else return false;
    return true;
  }

  void setOdeHandle(const OdeHandle& odeHandle){
    this->odeHandle = odeHandle;
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
  string name;
  OdeHandle odeHandle;
};

#endif
