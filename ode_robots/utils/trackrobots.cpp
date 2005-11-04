#include <stdlib.h>
#include <time.h>

#include "trackrobots.h"
#include "abstractrobot.h"


bool openTrackFile(const AbstractRobot* robot, TrackOptions& trackoptions){

  if(!robot) return false;
  if(trackoptions.trackPos || trackoptions.trackSpeed || trackoptions.trackOrientation){

    char filename[100];
    time_t t = time(0);
    strftime(filename, 50, "%F_%H-%M-%S", localtime(&t));
    strcat(filename,robot->getName());
    strcat(filename,".log");
    if(trackoptions.file){
      fclose(trackoptions.file);
    }
    trackoptions.file = fopen(filename,"w");
    if(!trackoptions.file) return false;
    fprintf(trackoptions.file, "# ");
    if(trackoptions.trackPos)   fprintf(trackoptions.file, "x y z ");
    if(trackoptions.trackSpeed) fprintf(trackoptions.file, "vx vy vz ");
    if( trackoptions.trackOrientation) fprintf(trackoptions.file," Not Implemented");    
    fprintf(trackoptions.file,"\n");  
  } 
  return true;
}

void trackRobot(const AbstractRobot* robot, const TrackOptions& trackoptions){
  if(!trackoptions.file || !robot) return;
  if(trackoptions.trackPos){
    Position p = robot->getPosition();
    fprintf(trackoptions.file, "%g %g %g ", p.x, p.y, p.z);
  }
  if(trackoptions.trackSpeed){
    fprintf(stderr," SpeedTracking is not implemented yet");
  }
  if( trackoptions.trackOrientation){
    fprintf(stderr," OrientationTracking is not implemented yet");
  }
}

void closeTrackFile(TrackOptions& trackoptions){
  if(trackoptions.file) fclose(trackoptions.file);
  trackoptions.file=0;
}
