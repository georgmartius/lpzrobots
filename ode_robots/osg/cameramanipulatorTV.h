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
 ***************************************************************************
 *                                                                         *
 *  Camera Manipulation by mouse and keyboard                              *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.1  2006-02-01 10:24:34  robot3
 *   new camera manipulator added
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __CAMERAMANIPULATORTV_H
#define __CAMERAMANIPULATORTV_H

#include "osgforwarddecl.h"
#include "cameramanipulator.h"

namespace lpzrobots {

  /**
     CameraManipulatorTV is a MatrixManipulator which provides Flying simulator-like
     updating of the camera position & orientation. 
     Left mouse button: Pan and tilt
     Right mouse button: forward and sideways
     Middle mouse button: up and sideways
  */

  class CameraManipulatorTV : public CameraManipulator {

  public:
    
    CameraManipulatorTV(osg::Node* node);
      
  protected:
    
    virtual ~CameraManipulatorTV();
      
  };
}

#endif
