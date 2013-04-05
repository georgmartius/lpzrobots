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
#ifndef __CAMERAMANIPULATORRACE_H
#define __CAMERAMANIPULATORRACE_H

#include "osgforwarddecl.h"
#include "cameramanipulator.h"

namespace lpzrobots {

  /**
     CameraManipulatorRace is a MatrixManipulator which provides Flying simulator-like
     updating of the camera position & orientation.
     Left mouse button: Pan and tilt
     Right mouse button: forward and sideways
     Middle mouse button: up and sideways
  */

  class CameraManipulatorRace : public CameraManipulator {

  public:

    CameraManipulatorRace(osg::Node* node,GlobalData& global, CameraHandle& cameraHandle);

    /** returns the classname of the manipulator
        it's NECCESSARY to define this funtion, otherwise
        the new manipulator WON'T WORK! (but ask me not why)
     */
    virtual const char* className() const { return "Race-Camera"; }

  protected:

    virtual ~CameraManipulatorRace();

    /** This handles robot movements, so that the camera movemenent is right affected.
        should be overwritten by new cameramanipulator
    */
       virtual void calcMovementByAgent();

  };
}

#endif
