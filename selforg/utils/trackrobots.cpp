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
 *   Revision 1.10  2009-08-04 14:55:22  jhoffmann
 *   Remove two memory leaks, but fix needs review for the open file pointers
 *
 *   Revision 1.9  2009/04/21 16:36:27  martius
 *   removed space at end of lines in log-files
 *
 *   Revision 1.8  2008/09/12 10:11:36  martius
 *   added space to speed output
 *
 *   Revision 1.7  2008/04/28 11:11:01  guettler
 *   include "matrix.h" from trackable class removed, used forward declaration
 *   instead - this change effectuates that no robot must be recompiled if
 *   matrix.h has changed.
 *
 *   Revision 1.6  2007/08/29 11:33:20  martius
 *   simulation time enters logfile
 *
 *   Revision 1.5  2007/04/05 15:14:15  martius
 *   angular speed tracking
 *
 *   Revision 1.4  2007/04/04 06:55:35  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2007/03/28 07:15:54  martius
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



