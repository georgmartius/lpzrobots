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

  RobotCameraManager::Overlay::Overlay(const Camera::CameraImage& image) :
    camImg(image), texture(0),overlay(0) {
  }

  RobotCameraManager::Overlay::~Overlay(){
  }



  RobotCameraManager::RobotCameraManager(int windowWidth, int windowHeight)
    : enabled(true), scale(1.0),
      windowWidth(windowWidth), windowHeight(windowHeight)
  {
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
      it->img->ref(); // add one to the reference counter, because otherwise it is deleted
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
    if(!enabled) return;
    int border=10;
    int padding=2;
    int x=windowWidth-border,y=windowHeight-border;
    int maxheight_in_row=0;
    FOREACH(RobotCams,cameras,rc){
      FOREACH(Overlays, rc->overlays, ol){
        if(ol->camImg.show){
          osg::Image* img = ol->camImg.img;
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
          ol->overlayW  = img->s()*ol->camImg.scale*scale;
          ol->overlayH  = img->t()*ol->camImg.scale*scale;
          if(x-ol->overlayW-border<0){
            y-= maxheight_in_row+border;
            x = windowWidth-border;
            maxheight_in_row=0;
          }
          maxheight_in_row = std::max(maxheight_in_row,ol->overlayH);
          ol->overlayX=x - ol->overlayW;
          ol->overlayY=y - ol->overlayH;
          x-=ol->overlayW+border;

          // set up place to show
          osg::ref_ptr<osg::Geometry> screenQuad;
          screenQuad = osg::createTexturedQuadGeometry(osg::Vec3(),
                                                       osg::Vec3(ol->overlayW, 0.0, 0.0),
                                                       osg::Vec3(0.0, ol->overlayH, 0.0));
          osg::ref_ptr<osg::Geode> quadGeode = new osg::Geode;
          quadGeode->addDrawable(screenQuad.get());
          osg::StateSet *quadState = quadGeode->getOrCreateStateSet();
          quadState->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
          quadState->setTextureAttributeAndModes(0, ol->texture, osg::StateAttribute::ON);

          osg::ref_ptr<osg::Camera> orthoCamera = new osg::Camera;
          // We don't want to apply perspective, just overlay using orthographic
          orthoCamera->setProjectionMatrix(osg::Matrix::ortho2D(-padding, ol->overlayW+padding,
                                                                -padding, ol->overlayH+padding));

          orthoCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
          orthoCamera->setViewMatrix(osg::Matrix::identity());

          orthoCamera->setViewport(ol->overlayX, ol->overlayY,
                                   ol->overlayW+2*padding, ol->overlayH+2*padding);
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

  bool RobotCameraManager::handle (const osgGA::GUIEventAdapter& ea,
                                   osgGA::GUIActionAdapter& aa,
                                   osg::Object* o, osg::NodeVisitor* nv){
    bool handled = false;
    switch(ea.getEventType()) {
    case(osgGA::GUIEventAdapter::KEYDOWN): {
      switch(ea.getKey()) {
      case 15 : // Ctrl - o
        enabled = !enabled;
        handled= true;
        break;
      case 40 : // (
        scale /= 1.5;
        handled=true;
        break;
      case 41 : // )
        scale *= 1.5;
        handled=true;
        break;
      default:
        break;
      }
    } break;
    case(osgGA::GUIEventAdapter::RESIZE):
      if(ea.getXmax() != windowWidth || ea.getYmax() != windowHeight){
        windowWidth  = ea.getWindowWidth();
        windowHeight = ea.getWindowHeight();
        handled=true;
      }
    default:
      break;
    }
    if(handled) updateView();

    return handled;
  }

  void RobotCameraManager::getUsage (osg::ApplicationUsage& au) const {
    au.addKeyboardMouseBinding("Overlay: Ctrl-o","Robot camera overlay on/off");
    au.addKeyboardMouseBinding("Overlay: ()","Decrease/increase overlay display");
  }


}

