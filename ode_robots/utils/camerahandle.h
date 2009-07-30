/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *  DESCRIPTION                                                            *
 *  
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.1  2009-07-30 11:52:32  guettler
 *  new CameraHandle replacing static variables in the CameraManipulators
 *										   *
 *                                                                         *
 **************************************************************************/
#ifndef _CAMERAHANDLE_H_
#define _CAMERAHANDLE_H_


#include <selforg/position.h>
#include <osg/Vec3f>

// forward declarations
namespace lpzrobots
{
  class OSGPrimitive;
  class OdeAgent;
}

namespace osg
{
  class Vec3f;
  typedef Vec3f Vec3;
}

namespace lpzrobots
{

  /**
   * Class which holds all data used by CameraManipulators.
   * The avoidance of static variables enables multithreaded (tasked)
   * simulations.
   */
  class CameraHandle
  {
  public:
    enum ManipulationType { No, Translational, Rotational};

    osg::Vec3 eye;
    osg::Vec3 view;
    osg::Vec3 home_eye;
    osg::Vec3 home_view;
    osg::Vec3 desiredEye;
    osg::Vec3 desiredView;
    bool home_externally_set;
    OdeAgent* watchingAgent;
    bool watchingAgentDefined;
    Position oldPositionOfAgent;
    bool oldPositionOfAgentDefined;

    ManipulationType doManipulation;
    osg::Vec3 manipulationPoint;
    OSGPrimitive* manipulationViz;

    CameraHandle();

    virtual ~CameraHandle();
  };

}

#endif /* _CAMERAHANDLE_H_ */
