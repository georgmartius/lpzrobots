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

bool TrackRobot::open(const Trackable* robot){

  if(!robot)
    return false;

  if(isTrackingSomething() && conf.writeFile)
  {

    if(file){
      fclose(file);
    }

    char filename[1024];
    if(conf.autoFilename){
      char date[128];
      time_t t = time(0);
      strftime(date, 128, "%F_%H-%M-%S", localtime(&t));
      if(conf.id>=0)
        sprintf(filename, "%s_track_%s_%i_%s.log",
                robot->getTrackableName().c_str(), conf.scene.c_str(), conf.id, date);
      else
        sprintf(filename, "%s_track_%s_%s.log",
                robot->getTrackableName().c_str(), conf.scene.c_str(), date);
    }else{
      if(conf.id>=0)
        sprintf(filename, "%s_%i.log", conf.scene.c_str(), conf.id);
      else
        sprintf(filename, "%s.log", conf.scene.c_str());
    }

    file = fopen(filename, "w");

    if(!file)
      return false;

    fprintf(file, "#C t");
    if(conf.trackPos)   fprintf(file, " x y z");
    if(conf.trackSpeed) fprintf(file, " vx vy vz wx wy wz");
    if(conf.trackOrientation)  fprintf(file, " o11 o12 o13 o21 o22 o23 o31 o32 o33");
    fprintf(file,"\n");
    fprintf(file,"# Recorded every %ith time step\n", conf.interval);
  }
  return true;
}

void TrackRobot::track(const Trackable* robot, double time)
{
  if(!file || !robot)
    return;

  if(cnt % conf.interval==0){
    //   fprintf(file, "%li ", cnt);
    fprintf(file, "%f", time);
    if(conf.trackPos){
      Position p = robot->getPosition();
      fprintf(file, " %g %g %g", p.x, p.y, p.z);
    }
    if(conf.trackSpeed){
      Position s = robot->getSpeed();
      fprintf(file, " %g %g %g", s.x, s.y, s.z);
      s = robot->getAngularSpeed();
      fprintf(file, " %g %g %g", s.x, s.y, s.z);
    }
    if( conf.trackOrientation){
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

// void TrackRobot::deepcopy (TrackRobot &lhs, const TrackRobot &rhs)
// {
//   lhs.conf          = rhs.conf;
//   lhs.cnt           = rhs.cnt;
//   lhs.conf.id       = rhs.conf.id+100;
//   lhs.file          = 0;
// }



