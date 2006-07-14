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
 ***************************************************************************
 *                                                                         *
 *  Camera Manipulation by mouse and keyboard                              *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2006-07-14 12:23:34  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/03/19 10:52:55  robot3
 *   follow mode now centers the view on the robot
 *   if the robot is choosed (only once)
 *
 *   Revision 1.1.2.2  2006/03/08 13:18:34  robot3
 *   follow mode now works, basic modifications
 *
 *   Revision 1.1.2.1  2006/03/06 16:59:18  robot3
 *   first dummy follow-version
 *
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __CAMERAMANIPULATORFOLLOW_H
#define __CAMERAMANIPULATORFOLLOW_H

#include "osgforwarddecl.h"
#include "cameramanipulator.h"

namespace lpzrobots {

  /**
     CameraManipulatorFollow is a MatrixManipulator which provides Flying simulator-like
     updating of the camera position & orientation. 
     Left mouse button: Pan and tilt
     Right mouse button: forward and sideways
     Middle mouse button: up and sideways
  */

  class CameraManipulatorFollow : public CameraManipulator {

    /** returns the classname of the manipulator
	it's NECCESSARY to define this funtion, otherwise
	the new manipulator WON'T WORK! (but ask me not why)
     */
    virtual const char* className() const { return "Following Camera"; }

  public:
    
    CameraManipulatorFollow(osg::Node* node,GlobalData& global);
      
  protected:

    virtual ~CameraManipulatorFollow();

    /** This handles robot movements, so that the camera movemenent is right affected.
	should be overwritten by new cameramanipulator
    */
    virtual void calcMovementByAgent();


    /** Sets the right view and eye if the robot has changed.
	Is called from manageRobots();
	should be overwritten by new cameramanipulator if needed
    */
        virtual void setHomeViewByAgent();

    
   
  };
}

#endif
