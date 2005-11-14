/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.8.4.1  2005-11-14 17:37:25  martius
 *   moved to selforg
 *
 *   Revision 1.8  2005/09/20 10:55:15  robot3
 *   camera module:
 *   -pressing key c now centers on focused robot
 *   -pressing key b now moves 5.0f behind the robot
 *   -fixed a few bugs (nullpointer crashes etc.)
 *
 *   Revision 1.7  2005/09/02 17:19:24  martius
 *   camera modes changed
 *
 *   Revision 1.6  2005/08/23 11:41:20  robot1
 *   advancedFollowing mode included
 *
 *   Revision 1.5  2005/08/22 12:38:32  robot1
 *   -advancedTV mode implemented, early version
 *   -internal code optimized
 *   -printMode prints now the current camera mode on stdout
 *
 *   Revision 1.4  2005/08/12 11:56:46  robot1
 *   tiny bugfixing
 *
 *   Revision 1.3  2005/08/09 11:08:49  robot1
 *   following mode included
 *
 *
 *   Revision 1.1  2005/08/08 11:06:47  martius
 *   camera is a module for camera movements
 *   includes cleaned
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __CAMERA_H
#define __CAMERA_H

#include "oderobot.h"

typedef enum CameraType { Static, TV, Following, advancedTV, advancedFollowing};

// moves the camera so that the robot is the watching object
// cameraType decides which type of camera movement is used
void moveCamera( CameraType camType, OdeRobot& robot);

// moves camera behind the robot
void moveBehindRobot(OdeRobot& robot);

// moves camera to robot position
void moveOnRobot(OdeRobot& robot);

// prints the last used camera mode
void printMode(CameraType camType);

#endif
