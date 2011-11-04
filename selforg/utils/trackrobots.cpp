/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>

#include "trackrobots.h"
#include "abstractrobot.h"
#include "matrix.h"

bool TrackRobot::open(const AbstractRobot* robot){

  if(!robot)
    return false;

  if(trackPos || trackSpeed || trackOrientation)
  {

    if(file){
      fclose(file);
    }

    char date[128];
    char filename[256];
    time_t t = time(0);
    strftime(date, 128, "%F_%H-%M-%S", localtime(&t));
    sprintf(filename, "%s_track_%s_%s.log", robot->getName().c_str(), scene, date);

    file = fopen(filename, "w");

    if(!file)
      return false;

    // copy filename for later reuse
    strncpy(this->filename, filename, 256);

    fprintf(file, "#C t");
    if(trackPos)   fprintf(file, " x y z");
    if(trackSpeed) fprintf(file, " vx vy vz wx wy wz");
    if( trackOrientation)  fprintf(file, " o11 o12 o13 o21 o22 o23 o31 o32 o33");
    fprintf(file,"\n");
    fprintf(file,"# Recorded every %ith time step\n", interval);
  }
  return true;
}

void TrackRobot::track(AbstractRobot* robot, double time)
{
  if(!file || !robot)
    return;

  if(cnt % interval==0){
    //   fprintf(file, "%li ", cnt);
    fprintf(file, "%f", time);
    if(trackPos){
      Position p = robot->getPosition();
      fprintf(file, " %g %g %g", p.x, p.y, p.z);
    }
    if(trackSpeed){
      Position s = robot->getSpeed();
      fprintf(file, " %g %g %g", s.x, s.y, s.z);
      s = robot->getAngularSpeed();
      fprintf(file, " %g %g %g", s.x, s.y, s.z);
    }
    if( trackOrientation){
      const matrix::Matrix& o = robot->getOrientation();
      for(int i=0; i<3; i++){
	for(int j=0; j<3; j++){
	  fprintf(file, " %g", o.val(i,j));
	}
      }
    }
    fprintf(file, "\n");
  }
  cnt++;
}

void TrackRobot::close()
{
  if(file)
    fclose(file);
  file = 0;
}

void TrackRobot::deepcopy (TrackRobot &lhs, const TrackRobot &rhs)
{
  lhs.trackPos         = rhs.trackPos;
  lhs.trackSpeed       = rhs.trackSpeed;
  lhs.trackOrientation = rhs.trackOrientation;
  lhs.displayTrace     = rhs.displayTrace;
  lhs.interval         = rhs.interval;
  lhs.scene            = strdup(rhs.scene);
  lhs.cnt              = rhs.cnt;

  // TODO: we can't reuse the file pointer, open again for appendig -> right?
  if ( strlen(rhs.filename) > 0 )
  {
    strncpy(lhs.filename, rhs.filename, 256);
    lhs.file = fopen("lhs.filename", "a");
  } else
  {
    lhs.file = 0;
  }

}



