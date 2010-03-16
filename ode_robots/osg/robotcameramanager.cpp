/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *
 ***************************************************************************/

#include "robotcameramanager.h"

#include <osg/Vec3>
#include <osg/Camera>
#include <osg/Image>
#include <osg/Texture2D>
#include <osg/Texture>
#include <osg/Geometry>
#include <osg/Geode>

#include <selforg/stl_adds.h>


namespace lpzrobots {

  RobotCameraManager::Overlay::Overlay(std::pair<osg::Image*, bool> image_show) :
    img(image_show.first), show(image_show.second), texture(0),overlay(0) {
  }

  RobotCameraManager::Overlay::~Overlay(){
  }


  
  RobotCameraManager::RobotCameraManager(){
    // create nodes
    display = new osg::Group();
    offscreen = new osg::Group();
  }

  void RobotCameraManager::addCamera(Camera* cam){
    if(!cam) return;
    RobotCam robotcam;
    robotcam.cam    = cam;
    const Camera::CameraImages& imgs = cam->getImages();
    FOREACHC(Camera::CameraImages, imgs, it){
      robotcam.overlays.push_back(*it);
    }
    cameras.push_back(robotcam);
    offscreen->addChild(cam->getRRTCam());
    updateView();
  }

  void RobotCameraManager::removeCamera(Camera* cam){
    FOREACH(RobotCams,cameras,it){
      if(it->cam == cam){
        cameras.erase(it);
        break;
      }
    }
    offscreen->removeChild(cam->getRRTCam());
    updateView();
  }

  

  void RobotCameraManager::updateView(){
    display->removeChildren(0,display->getNumChildren());
    int screenW=800, screenH=600;
    int x=screenW,y=screenH;
    int maxheight_in_row=0;
    FOREACH(RobotCams,cameras,rc){
      FOREACH(Overlays, rc->overlays, ol){
        if(ol->show){
          osg::Image* img = ol->img;
          //          if(!ol->texture){
          // Create the texture to render to
          ol->texture = new osg::Texture2D;
          ol->texture->setTextureSize(img->s(), img->t());
          ol->texture->setInternalFormat(GL_RGBA);
          ol->texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
          ol->texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
          ol->texture->setImage(0, img);
          //ol->texture->setShadowComparison(true);
          //ol->texture->setShadowTextureMode(Texture::LUMINANCE);
          // }
          ol->overlayWidth  = img->s();
          ol->overlayHeight = img->t();                      
          if(x-ol->overlayWidth<0){
            y-=maxheight_in_row+10;
            x=0;
            maxheight_in_row=0;
          }
          maxheight_in_row = std::max(maxheight_in_row,ol->overlayHeight);
          ol->overlayX=x - ol->overlayWidth;
          ol->overlayY=y - ol->overlayHeight;
          x-=ol->overlayWidth+10;
          
          // set up place to show
          osg::ref_ptr<osg::Geometry> screenQuad;
          screenQuad = osg::createTexturedQuadGeometry(osg::Vec3(),
                                                       osg::Vec3(ol->overlayWidth, 0.0, 0.0),
                                                       osg::Vec3(0.0, ol->overlayHeight, 0.0));
          osg::ref_ptr<osg::Geode> quadGeode = new osg::Geode;
          quadGeode->addDrawable(screenQuad.get());
          osg::StateSet *quadState = quadGeode->getOrCreateStateSet();
          quadState->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
          quadState->setTextureAttributeAndModes(0, ol->texture, osg::StateAttribute::ON);
                   
          osg::ref_ptr<osg::Camera> orthoCamera = new osg::Camera;
          // We don't want to apply perspective, just overlay using orthographic
          orthoCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, ol->overlayWidth, 0, ol->overlayHeight));
          
          orthoCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
          orthoCamera->setViewMatrix(osg::Matrix::identity());
                    
          orthoCamera->setViewport(ol->overlayX, ol->overlayY, ol->overlayWidth, ol->overlayHeight);
          // Make sure to render this after rendering the scene
          //  in order to overlay the quad on top
          orthoCamera->setRenderOrder(osg::Camera::POST_RENDER);          
          // Render only the quad
          orthoCamera->addChild(quadGeode.get());
          ol->overlay = orthoCamera.get();
          display->addChild(ol->overlay);
        }
      }
    }
  }

}
