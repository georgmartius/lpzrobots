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
#ifndef __ROBOTCAMERAMANAGER_H
#define __ROBOTCAMERAMANAGER_H

#include "camera.h"
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>

namespace lpzrobots {

  /**
     Manages camera sensors. The cameras are rendered to texture offscreen, 
     meaning independent of the normal graphical rendering.
     Additionally the view of the cameras is displayed as an overlay.     
   */
  class RobotCameraManager : public osgGA::GUIEventHandler {    
    struct Overlay {
      Overlay(const Camera::CameraImage& image);
      ~Overlay();
      osg::Image* img;
      bool show;
      float scale;
      osg::Texture2D* texture;
      int overlayWidth;
      int overlayHeight;
      int overlayX;
      int overlayY;
      osg::Node* overlay; 
    };
    typedef std::vector<Overlay> Overlays;
    struct RobotCam {
      Camera* cam;
      Overlays overlays;
    };
    typedef std::vector<RobotCam> RobotCams;
    
  public:
    RobotCameraManager();
    virtual void addCamera(Camera* cam);
    virtual void removeCamera(Camera* cam);

    virtual osg::Group* getDisplay() { return display; }
    virtual osg::Group* getOffScreen()  { return offscreen; }
    
    /* ** GUIEventHandler interface **/
    virtual bool handle (const osgGA::GUIEventAdapter& ea, 
			 osgGA::GUIActionAdapter& aa, 
			 osg::Object* o, osg::NodeVisitor* nv);
    virtual void getUsage (osg::ApplicationUsage &) const;
    
  protected:

    virtual void updateView();

    osg::ref_ptr<osg::Group> display;
    osg::ref_ptr<osg::Group> offscreen;
    RobotCams cameras;

    bool enabled;
    float scale;
  };


}

#endif
