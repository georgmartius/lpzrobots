#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>

#include "trackrobots.h"
#include "abstractrobot.h"


bool TrackRobot::open(const AbstractRobot* robot){

  if(!robot) return false;
  if(trackPos || trackSpeed || trackOrientation){

    if(file){
      fclose(file);
    }

    char filename[100];
    char filename2[100];
    time_t t = time(0);
    struct stat filestat;
    strftime(filename, 50, "%F_%H-%M-%S_", localtime(&t));
    strcat(filename,robot->getName().c_str());
    sprintf(filename2, "%s.log", filename);
    // try to stat file and if it exists then try to append a number
    for(int i=1; i< 20; i++){
      if(stat(filename2, &filestat) == -1){
	break;
      }else{
	sprintf(filename2, "%s%i.log", filename, i );
      }      
    }

    file = fopen(filename2,"w");

    if(!file) return false;
    fprintf(file, "#C t ");
    if(trackPos)   fprintf(file, "x y z ");
    if(trackSpeed) fprintf(file, "vx vy vz ");
    if( trackOrientation) fprintf(file," Not Implemented");    
    fprintf(file,"\n");  
  } 
  return true;
}

void TrackRobot::track(AbstractRobot* robot) {
  if(!file || !robot) return;
  if(cnt % interval==0){
    fprintf(file, "%i ", cnt);
    if(trackPos){
      Position p = robot->getPosition();
      fprintf(file, "%g %g %g ", p.x, p.y, p.z);
    }
    if(trackSpeed){
      fprintf(stderr," SpeedTracking is not implemented yet");
    }
    if( trackOrientation){
      fprintf(stderr," OrientationTracking is not implemented yet");
    }
    fprintf(file, "\n");
  }
  cnt++;
}

void TrackRobot::close() {
  if(file) fclose(file);
  file=0;
}
