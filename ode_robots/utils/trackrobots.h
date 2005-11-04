#ifndef __TRACKROBOTS_H
#define __TRACKROBOTS_H

#include <stdio.h>

typedef struct TrackOptions {
  TrackOptions(){
    trackPos = false;
    trackSpeed = false;
    trackOrientation = false;
    file=0;
  }
  /** Set the tracking mode of the simulation environment. 
      If one of the parameters is true the tracking is enabled.
      The tracking is written into a file with the current date and time as name.
   */ 
  TrackOptions(bool trackPos, bool trackSpeed, bool trackOrientation){
    this->trackPos     = trackPos;
    this->trackSpeed   = trackSpeed;
    this->trackOrientation = trackOrientation;
  }

  bool trackPos;
  bool trackSpeed;
  bool trackOrientation;
  FILE* file;
}TrackOptions;

class AbstractRobot;

bool openTrackFile(TrackOptions& trackoptions);
void trackRobot(AbstractRobot* robot, const TrackOptions& trackoptions);
void closeTrackFile(TrackOptions& trackoptions);

#endif
