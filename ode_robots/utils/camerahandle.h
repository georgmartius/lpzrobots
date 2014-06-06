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
  class Camera;
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
    enum ManipulationType { No, Translational, TranslationalHorizontal , Rotational};

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
    OSGPrimitive* manipulationViz2;
    double manipulationForce;

    osg::Camera* cam;

    CameraHandle();

    virtual ~CameraHandle();
  };

}

#endif /* _CAMERAHANDLE_H_ */
