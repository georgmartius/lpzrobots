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
 *   Revision 1.6  2010-03-21 21:48:59  martius
 *   camera sensor bugfixing (reference to osghandle)
 *   twowheeled robot added (nimm2 with camera)
 *   sense function added to robots (before control): sensors (type Sensor) are checked here
 *   position and optical flow camera sensors added
 *
 *   Revision 1.5  2010/03/19 17:46:21  martius
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
 *   Revision 1.2  2010/03/16 15:41:23  martius
 *   Camera is working now! Using the new lpzviewer it is possible to run render it at
 *    the control cycle independent of the graphics
 *
 *   Revision 1.1  2010/03/05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *******************************************`********************************/

#include "camera.h"

#include <assert.h>

#include <selforg/position.h>
#include <osg/Matrix>
#include <osg/Camera>
#include <osg/Image>

#include "osgprimitive.h"
#include "robotcameramanager.h"
#include "imageprocessor.h"
#include "primitive.h"
#include "pos.h"
#include "axis.h"
#include <selforg/stl_adds.h>


namespace lpzrobots {
  
  
  void Camera::PostDrawCallback::operator () (const osg::Camera& /*camera*/) const {          
    // start the image processing chain now
    FOREACH(ImageProcessors, cam->conf.processors, ip){      
      (*ip)->process();
    }
  }    

  Camera::Camera( const CameraConf& conf)
    : conf(conf) {
    sensorBody1 = 0;
    sensorBody2 = 0;
    ccd = 0;
  }

  Camera::~Camera(){
    if(ccd) ccd->unref();
    if(sensorBody1) delete sensorBody1;
    if(sensorBody2) delete sensorBody2;
    FOREACH(ImageProcessors, conf.processors, ip){ 
      if (*ip) delete *ip;
    }

  }
  
  void Camera::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                    Primitive* body, const osg::Matrix& pose){
    assert(body);
    this->osgHandle = osgHandle;
    this->body=body;
    this->pose=pose;
    
    if(conf.draw){
      sensorBody1 = new OSGBox(conf.camSize, conf.camSize, conf.camSize / 6.0);
      sensorBody2 = new OSGCylinder(conf.camSize/3, conf.camSize / 2.0);
      sensorBody1->init(this->osgHandle);
      sensorBody2->init(this->osgHandle.changeColor(Color(0, 0, 0)));
      // sensorBody1->setColor(Color(0.2, 0.2, 0.2));

    }        
    conf.behind -= conf.camSize/2 + conf.camSize/6;

    // set up the render to texture (image) camera.
    cam = new osg::Camera;
    cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // set up projection.
    float ratio = (double)conf.width/(double)conf.height;
    cam->setProjectionMatrixAsPerspective(conf.fov/ratio, conf.anamorph*ratio,0.05,50);        
    cam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR); 

    // set view
    cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    osg::Matrix p = pose * body->getPose();
    cam->setViewMatrixAsLookAt(Pos(0,0,-conf.behind)*p, Pos(Axis(0,0,1)*p), Pos(Axis(0,1,0)*p));

    cam->setViewport(0, 0, conf.width, conf.height);
    
    // Frame buffer objects are the best option
    cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);    
    // We need to render to the texture BEFORE we render to the screen 
    //  (does not matter because we do it offscreen)
    cam->setRenderOrder(osg::Camera::PRE_RENDER);    
    // Add world to be drawn to the texture/image
    cam->addChild(osgHandle.scene->world_noshadow);
  
    ccd = new osg::Image;
    ccd->allocateImage(conf.width, conf.height, 1, GL_RGB, GL_UNSIGNED_BYTE);    
    // The camera will render into the image and its copied on each time it is rendered
    cam->attach(osg::Camera::COLOR_BUFFER, ccd);   
    cam->setPostDrawCallback(new PostDrawCallback(this));

        
    cameraImages.push_back(CameraImage(ccd, conf.show, conf.scale, conf.name));

    FOREACH(ImageProcessors, conf.processors, ip){      
      cameraImages.push_back((*ip)->init(cameraImages));
    }
  
    this->osgHandle.scene->robotCamManager->addCamera(this);
    initialized = true;
  }
   
  // const unsigned char* Camera::getData() const {
//     return ccd->data();    
//   };  

  void Camera::update(){  
    osg::Matrix p = pose * body->getPose();
    if(sensorBody1) {          
      sensorBody1->setMatrix(osg::Matrix::translate(0, 0, conf.camSize/12.0) * p);
      sensorBody2->setMatrix(osg::Matrix::translate(0,-conf.camSize/6.0,
                                                    conf.camSize/6.0 + conf.camSize/4.0) * p);
    }
    cam->setViewMatrixAsLookAt(Pos(0,0,-conf.behind)*p, Pos(0,0,1)*p, Pos(Axis(0,1,0)*p));
  }
  
}
