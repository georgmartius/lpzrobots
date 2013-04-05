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
      Camera::CameraImage camImg;
      osg::Texture2D* texture;
      int overlayW;
      int overlayH;
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
    RobotCameraManager(int windowWidth, int windowHeight);
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
    int windowWidth;
    int windowHeight;
  };


}

#endif
