#ifndef __GLOBALDATA_H
#define __GLOBALDATA_H


#include <vector>
using namespace std;

#include "odehandle.h"
#include "odeconfig.h"

class Agent;
class AbstractObstacle;
class Configurable;

typedef vector<AbstractObstacle*> ObstacleList; ///
typedef vector<Configurable*> ConfigList;       ///
typedef vector<Agent*> AgentList;               ///

typedef struct GlobalData
{
  GlobalData(const OdeHandle& odehandle) 
    : odeConfig(odehandle) { 
    time = 0;
  }
  OdeConfig odeConfig;
  ConfigList configs;
  ObstacleList obstacles;
  AgentList agents;
  double time;
} GlobalData;

#endif
