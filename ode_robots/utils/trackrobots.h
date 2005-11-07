#ifndef __TRACKROBOTS_H
#define __TRACKROBOTS_H

#include <stdio.h>

class AbstractRobot;
class Agent;

class TrackRobot {
public:

  friend class Agent;

  TrackRobot(){
    trackPos = false;
    trackSpeed = false;
    trackOrientation = false;
    interval = 1;
    file=0;
    cnt=0;
  }
  /** Set the tracking mode of the simulation environment. 
      If one of the parameters is true the tracking is enabled.
      The tracking is written into a file with the current date and time as name.
   */ 
  TrackRobot(bool trackPos, bool trackSpeed, bool trackOrientation, int interval = 1){
    this->trackPos     = trackPos;
    this->trackSpeed   = trackSpeed;
    this->trackOrientation = trackOrientation;
    this->interval = interval;
    file=0;
    cnt=0;
  }

  bool open(const AbstractRobot* robot);
  void track(AbstractRobot* robot);
  void close();


private:
  bool trackPos;
  bool trackSpeed;
  bool trackOrientation;
  int interval;
  FILE* file;
  long cnt;
};



#endif
