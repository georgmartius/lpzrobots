/***************************************************************************
 *   Copyright (C) 2007 by Robot Group Leipzig                             *
 *    georg@nld.ds.mpg.de                                                  *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 ** Started on  Mon Oct 15 16:48:03 2007 Georg Martius *
 ** Last update Mon Oct 15 16:48:03 2007 Georg Martius *
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
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2010-03-05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef   	CAMERA_H_
# define   	CAMERA_H_

#include <osg/Matrix>
#include <osg/Camera>
// #include <osgViewer/View>

#include "osghandle.h"
#include "odehandle.h"

namespace osg {
class Texture2D;
}

namespace lpzrobots {

  class OSGCylinder;
  class OSGBox;
  class Transform;


  /** Camera Sensor. Implements a simulated camera with full OpenGL rendering.
  */
  class Camera {
  public:  
    enum Type { Isotrop, Foveal }; // Todo: check real names

    /** @param type type of CCD
        @param width number of pixels horizontally
        @param height number of pixels vertically
        @param fov field of view (opening angle of lenses)
        @param drawSize size of visual appearance of camera
    */
    Camera( Type type = Isotrop, 
            int width = 320, int height = 200, 
            float fov = 90, 
            float drawSize=0.2);
    virtual ~Camera();
  
    virtual void init(const OdeHandle& odeHandle,
		      const OsgHandle& osgHandle, 
		      Primitive* body, 
		      const osg::Matrix& pose,
                      bool draw = true, bool showImage = true);
  
    virtual bool sense(const GlobalData& globaldata);


    virtual void update();

  private:
    //    osgViewer::View* view;
    Type type; 
    int width;
    int height;
    float fov;
    bool draw;
    float drawSize;
    bool showImage;
    
    osg::Texture2D* texture;
    osg::Camera* cam;

    Primitive* body;
    osg::Matrix pose;

    OSGBox* sensorBody1;
    OSGCylinder* sensorBody2;
    OsgHandle osgHandle;
    Transform* transform;

    bool initialised;

  };  

}

#endif 	    /* !CAMERA_H_ */
