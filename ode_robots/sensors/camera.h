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
 *   Revision 1.5  2010-03-19 17:46:21  martius
 *   camerasensors added
 *   camera works great now. Near and far plane fixed by hand and optimal positioning
 *   many image processings added
 *
 *   Revision 1.4  2010/03/17 17:26:36  martius
 *   robotcameramanager uses keyboard and respects resize
 *   (robot) camera is has a conf object
 *   image processing implemented, with a few standard procedures
 *
 *   Revision 1.3  2010/03/16 23:24:38  martius
 *   scaling added
 *   eventhandling in robotcameramanager added
 *
 *   Revision 1.2  2010/03/16 15:41:11  martius
 *   Camera is working now! Using the new lpzviewer it is possible to run render it at
 *    the control cycle independent of the graphics
 *
 *   Revision 1.1  2010/03/05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __CAMERA_H_
#define __CAMERA_H_

#include "osgforwarddecl.h"

#include "osghandle.h"
#include "odehandle.h"

#include <osg/Matrix>
#include <osg/Camera>


namespace osg {
class Image;
}

namespace lpzrobots {

  class OSGCylinder;
  class OSGBox;
  class Transform;

  class ImageProcessor;
  typedef std::vector<ImageProcessor* > ImageProcessors;


  struct CameraConf {
    int width;       ///< horizontal resolution (power of 2)
    int height;      ///< vertical resolution (power of 2)
    float fov;       ///< field of view in degree (opening angle of lenses)
    float anamorph;  ///< anamorph ratio of focal length in vertical and horizontal direction
    float behind;    ///< distance of which the camera is behind the actually drawn position (to aviod near clipping)
    float draw;      ///< whether to draw a physical camera object
    float camSize;   ///< size of the physical camera object
    bool show;       ///< whether to show the image on the screen
    float scale;     ///< scaling for display
    std::string name; ///< name of the camera
    ImageProcessors processors; ///< list of image processors that filter the raw image
  };

  /** A Robot Camera. Implements a simulated camera with full OpenGL rendering.
   */
  class Camera {
  public:  
    struct PostDrawCallback : public osg::Camera::DrawCallback {
      PostDrawCallback(Camera* cam):
        cam(cam) { }
      virtual void operator () (const osg::Camera& /*camera*/) const;
      Camera* cam;
    };


    /// structure to store the image data and information for display
    struct CameraImage{
      CameraImage(){};
      CameraImage(osg::Image* img, bool show, float scale, const std::string& name)
        : img(img), show(show), scale(scale), name(name){
      }
      osg::Image* img;
      bool show;       ///< whether to show the image on the screen
      float scale;     ///< scaling for display
      std::string name; ///< name of the image
    };

    typedef std::vector<CameraImage > CameraImages;

    /** Creates a camera. 
        Note that the order in which the image processors are positioned 
         in conf.imageProcessors matters.
        The resulting CameraImages are stored in a list (see getImages) and 
        usually the processors use the last image in this list (result of last processing).
     */

    Camera( const CameraConf& conf = getDefaultConf() );

    static CameraConf getDefaultConf(){
      CameraConf c;
      c.width     = 256;
      c.height    = 128;
      c.fov       = 90;
      c.anamorph  = 1;
      c.behind    = 0.04; // since the nearplane is at 0.05
      c.camSize   = 0.2;
      c.draw      = true;
      c.show      = true;
      c.scale     = 1.0;
      c.name      = "raw";
      return c;
    }

    virtual ~Camera();

    /** initializes the camera. The OSG camera is created and the 
        raw image and the imageprocessor is initialized.
     */
    virtual void init(const OdeHandle& odeHandle,
		      const OsgHandle& osgHandle, 
		      Primitive* body, 
		      const osg::Matrix& pose);
  
    // virtual bool sense(const GlobalData& globaldata);

    /// all images (raw and processed)
    virtual const CameraImages& getImages() const  { return cameraImages;}

    /// last image of processing stack
    virtual const osg::Image* getImage() const  { return cameraImages.back().img;}

    virtual osg::Camera* getRRTCam() { return cam;}

    virtual void update();

    bool isInitialized() { return initialized; }
  private:
    CameraConf conf;
    
    osg::Camera* cam;
    osg::Image* ccd;
    CameraImages cameraImages;

    Primitive* body;
    osg::Matrix pose;

    OSGBox* sensorBody1;
    OSGCylinder* sensorBody2;
    OsgHandle osgHandle;
    Transform* transform;

    bool initialized;
  };  

}

#endif 	    /* __CAMERA_H_ */
