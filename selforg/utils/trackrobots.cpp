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
 *   Revision 1.3  2007-03-28 07:15:54  martius
 *   speed and orientation tracking enabled
 *
 *   Revision 1.2  2006/07/14 12:24:02  martius
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

    char date[128];
    char filename[256];
    time_t t = time(0);
    struct stat filestat;
    strftime(date, 128, "%F_%H-%M-%S", localtime(&t));
    sprintf(filename, "%s_track_%s_%s.log", robot->getName().c_str(), scene, date);

    file = fopen(filename,"w");

    if(!file) return false;
    fprintf(file, "#C t ");
    if(trackPos)   fprintf(file, "x y z ");
    if(trackSpeed) fprintf(file, "vx vy vz ");
    if( trackOrientation)  fprintf(file, "o11 o12 o13 o21 o22 o23 o31 o32 o33 ");
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
      Position s = robot->getSpeed();
      fprintf(file, "%g %g %g ", s.x, s.y, s.z);
    }
    if( trackOrientation){
      const matrix::Matrix& o = robot->getOrientation();
      for(int i=0; i<3; i++){
	for(int j=0; j<3; j++){
	  fprintf(file, "%g ", o.val(i,j));	  
	}	
      }
    }
    fprintf(file, "\n");
  }
  cnt++;
}

void TrackRobot::close() {
  if(file) fclose(file);
  file=0;
}
