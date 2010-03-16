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
 *   Revision 1.2  2010-03-16 15:41:23  martius
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

#include <selforg/position.h>
#include <osg/Matrix>
#include <osg/Camera>
#include <osg/Image>

#include "osgprimitive.h"
#include "robotcameramanager.h"
#include "primitive.h"
#include "pos.h"
#include "axis.h"


namespace lpzrobots {
  
  Camera::Camera( int width, int height, float fov, float drawSize, float anamorph)
    : width(width), height(height), fov(fov), drawSize(drawSize), anamorph(anamorph) {
    // image = new sensor[width*height];
    sensorBody1 = 0;
    sensorBody2 = 0;
  }

  Camera::~Camera(){
    //    delete[] image;
  }
  
  void Camera::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                    Primitive* body, const osg::Matrix& pose,
                    bool draw, bool showImage){
    this->osgHandle = osgHandle;
    this->draw = draw;
    this->showImage = showImage;
    this->body=body;
    this->pose=pose;
    
    if(draw){
      sensorBody1 = new OSGBox(drawSize, drawSize, drawSize / 6.0);
      sensorBody2 = new OSGCylinder(drawSize/3, drawSize / 2.0);
      sensorBody1->init(osgHandle);
      sensorBody2->init(osgHandle);
      // sensorBody1->setColor(Color(0.2, 0.2, 0.2));
      sensorBody2->setColor(Color(0, 0, 0));
    }        

    // set up the render to texture (image) camera.
    cam = new osg::Camera;
    cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // set up projection.
    float ratio = (double)width/(double)height;
    cam->setProjectionMatrixAsPerspective(fov/ratio, anamorph*ratio,0.1,30);    
    // set view
    cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    osg::Matrix p = pose * body->getPose();
    cam->setViewMatrixAsLookAt(Pos(0,0,0)*p, Pos(Axis(0,0,1)*p), Pos(Axis(0,1,0)*p));

    cam->setViewport(0, 0, width, height);
    
    // Frame buffer objects are the best option
    cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);    
    // We need to render to the texture BEFORE we render to the screen 
    //  (does not matter because we do it offscreen)
    cam->setRenderOrder(osg::Camera::PRE_RENDER);    
    // Add world to be drawn to the texture/image
    cam->addChild(osgHandle.scene->world_noshadow);
  
    ccd = new osg::Image;
    ccd->allocateImage(width, height, 1, GL_RGB, GL_UNSIGNED_BYTE);    
    // The camera will render into the image and its copied on each time it is rendered
    cam->attach(osg::Camera::COLOR_BUFFER, ccd);   
        
    cameraImages.push_back(CameraImage(ccd,showImage));

    initialised = true;
    
    this->osgHandle.scene->robotCamManager->addCamera(this);
  }
  
  bool Camera::sense(const GlobalData& globaldata){
    return true;
  }


  const unsigned char* Camera::getData() const {
    return ccd->data();    
  };  

  void Camera::update(){  
    osg::Matrix p = pose * body->getPose();
    if(sensorBody1) {          
      sensorBody1->setMatrix(osg::Matrix::translate(0, 0, drawSize/12.0) * p);
      sensorBody2->setMatrix(osg::Matrix::translate(0,-drawSize/6.0,drawSize/6.0 + drawSize/4.0) * p);
    }
    cam->setViewMatrixAsLookAt(Pos(0,0,0)*p, Pos(0,0,1)*p, Pos(Axis(0,1,0)*p));
  }
  
}
