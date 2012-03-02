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
  
  void CameraConf::removeProcessors() {
    FOREACH(ImageProcessors, processors, p){
      if(*p) delete *p;
    }
    processors.clear();
  }

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
    conf.removeProcessors();
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

  /// changes the relative pose of the camera
  void Camera::setPose(const osg::Matrix& pose){
    this->pose=pose;
  }

  /// returns the relative pose of the camera
  osg::Matrix Camera::getPose(){
    return this->pose;
  }

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
