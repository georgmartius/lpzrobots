/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2006-07-14 12:24:02  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.2  2006/01/31 15:48:37  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.1  2005/11/16 11:24:27  martius
 *   moved to selforg
 *
 *   Revision 1.4  2005/11/10 09:08:16  martius
 *   trace has a name
 *
 *   Revision 1.3  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
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
    if(scene){      
      strcat(filename,scene);
      strcat(filename,"_");
    }
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
    fprintf(file, "%li ", cnt);
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
