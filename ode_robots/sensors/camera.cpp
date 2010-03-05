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
 *   Revision 1.1  2010-03-05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *******************************************`********************************/

#include "camera.h"

#include <selforg/position.h>
#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/Texture2D>
#include <osg/Texture>

#include "osgprimitive.h"
#include "primitive.h"
#include "pos.h"
#include "axis.h"


namespace lpzrobots {
  
  Camera::Camera( Type type, int width, int height, float fov, float drawSize)
    : type (type), width(width), height(height), fov(fov), drawSize(drawSize) {
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
      sensorBody1->setColor(Color(0.2, 0.2, 0.2));
      sensorBody2->setColor(Color(0, 0, 0));
    }    

    // Create the texture to render to
    texture = new osg::Texture2D;
    texture->setTextureSize(width, height);
    texture->setInternalFormat(GL_RGBA);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

    //texture->setShadowComparison(true);
    //texture->setShadowTextureMode(Texture::LUMINANCE);
        
    /*    transform = new Transform(body, screenQuad, pose);
    OdeHandle myOdeHandle(odeHandle);  
    transform->init(odeHandle, 0, osgHandle, 
    Primitive::Draw);*/
    

    // set up the render to texture camera.
    cam = new osg::Camera;
    cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // set up projection.
    cam->setProjectionMatrixAsPerspective(fov/2, (double)width/(double)height,0.1,30);    
    // set view
    cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    osg::Matrix p = pose * body->getPose();
    cam->setViewMatrixAsLookAt(Pos(0,0,0)*p, Pos(Axis(0,0,1)*p), Pos(Axis(0,1,0)*p));

    cam->setViewport(0, 0, width, height);
    
    // Frame buffer objects are the best option
    cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);    
    // We need to render to the texture BEFORE we render to the screen
    cam->setRenderOrder(osg::Camera::PRE_RENDER);    
    // The camera will render into the texture that we created earlier
    cam->attach(osg::Camera::COLOR_BUFFER, texture);
    // Add world to be drawn to the texture
    cam->addChild(osgHandle.world);
    
    // view->addSlave(cam,false); // this was a test as a child view (but does not work)
    osgHandle.root->addChild(cam);

    if(showImage){
      // Then we create the camera that will overlay a quad on the screen 
      //  displaying the texture we rendered.

      float overlayWidth=width/2;
      float overlayHeight=height/2;
      // set up place to show
      osg::ref_ptr<osg::Geometry> screenQuad;
      screenQuad = osg::createTexturedQuadGeometry(osg::Vec3(),
                                                   osg::Vec3(overlayWidth, 0.0, 0.0),
                                                   osg::Vec3(0.0, overlayHeight, 0.0));
      osg::ref_ptr<osg::Geode> quadGeode = new osg::Geode;
      quadGeode->addDrawable(screenQuad.get());
      osg::StateSet *quadState = quadGeode->getOrCreateStateSet();
      quadState->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
      quadState->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);


      osg::ref_ptr<osg::Camera> orthoCamera = new osg::Camera;
      // We don't want to apply perspective, just overlay using orthographic
      orthoCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, overlayWidth, 0, overlayHeight));
      
      orthoCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
      orthoCamera->setViewMatrix(osg::Matrix::identity());
      
      
      orthoCamera->setViewport(0, 400, overlayWidth, overlayHeight);      
      // Make sure to render this after rendering the scene
      // in order to overlay the quad on top
      orthoCamera->setRenderOrder(osg::Camera::POST_RENDER);
      
      // Render only the quad
      orthoCamera->addChild(quadGeode.get());
      osgHandle.root->addChild(orthoCamera.get());      
    }


    initialised = true;

  }
  
  bool Camera::sense(const GlobalData& globaldata){
    return true;
  }
  
  void Camera::update(){  
    osg::Matrix p = pose * body->getPose();
    if(sensorBody1) {          
      sensorBody1->setMatrix(osg::Matrix::translate(0, 0, drawSize/12.0) * p);
      sensorBody2->setMatrix(osg::Matrix::translate(0,-drawSize/6.0,drawSize/6.0 + drawSize/4.0) * p);
    }
    cam->setViewMatrixAsLookAt(Pos(0,0,0)*p, Pos(0,0,1)*p, Pos(Axis(0,1,0)*p));
  }
  
}
