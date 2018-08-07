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

#include "lpzhelphandler.h"

#include <osg/Version>
#include <osg/PolygonMode>
#include <osgText/Text>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Renderer>


namespace lpzrobots{

  /* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
   *
   * This library is open source and may be redistributed and/or modified under
   * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
   * (at your option) any later version.  The full license is in LICENSE file
   * included with this distribution, and on the openscenegraph.org website.
   *
   * This library is distributed in the hope that it will be useful,
   * but WITHOUT ANY WARRANTY; without even the implied warranty of
   * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   * OpenSceneGraph Public License for more details.
   */

#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Renderer>

#include <osg/PolygonMode>

#include <osgText/Text>

  using namespace osgViewer;

  LpzHelpHandler::LpzHelpHandler(osg::ApplicationUsage* au):
    _applicationUsage(au),
    _keyEventTogglesOnScreenHelp('h'),
    _helpEnabled(false),
    _initialized(false)
  {
    _camera = new osg::Camera;
    _camera->setRenderer(new Renderer(_camera.get()));
    _camera->setRenderOrder(osg::Camera::POST_RENDER, 11);
  }


  void LpzHelpHandler::reset()
  {
    _initialized = false;
    _camera->setGraphicsContext(0);
  }

  bool LpzHelpHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
  {
    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
    if (!view) return false;

    osgViewer::ViewerBase* viewer = view->getViewerBase();
    if (!viewer) return false;

    if (ea.getHandled()) return false;

    switch(ea.getEventType())
      {
      case(osgGA::GUIEventAdapter::KEYDOWN):
        {
          if (ea.getKey()==_keyEventTogglesOnScreenHelp)
            {
              if (!_initialized)
                {
                  setUpHUDCamera(viewer);
                  setUpScene(viewer);
                }

              _helpEnabled = !_helpEnabled;

              if (_helpEnabled)
                {
                  _camera->setNodeMask(0xffffffff);
                }
              else
                {
                  _camera->setNodeMask(0);
                }
              return true;
            }
        }
      default: break;
      }

    return false;

  }

  void LpzHelpHandler::setUpHUDCamera(osgViewer::ViewerBase* viewer)
  {
    osgViewer::GraphicsWindow* window = dynamic_cast<osgViewer::GraphicsWindow*>(_camera->getGraphicsContext());

    if (!window)
      {
        osgViewer::Viewer::Windows windows;
        viewer->getWindows(windows);

        if (windows.empty()) return;

        window = windows.front();

        _camera->setGraphicsContext(window);
      }

    _camera->setGraphicsContext(window);
    _camera->setViewport(0, 0, window->getTraits()->width, window->getTraits()->height);

    _camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1280,0,1024));
    _camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    _camera->setClearMask(0);

    _initialized = true;
  }

  void LpzHelpHandler::setUpScene(osgViewer::ViewerBase* viewer)
  {
    _switch = new osg::Switch;

    _camera->addChild(_switch.get());

    osg::StateSet* stateset = _switch->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
    stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
    stateset->setAttribute(new osg::PolygonMode(), osg::StateAttribute::PROTECTED);

    std::string font("fonts/arial.ttf"); //fonts/fudd.ttf

    if (!_applicationUsage) setApplicationUsage(new osg::ApplicationUsage());

    viewer->getUsage(*_applicationUsage);

    float leftPos = 5.0f;
    float startDescription = 300.0f;
    float characterSize = 30.0f;

    osg::Vec3 pos(leftPos,1024.0f,0.0f);
    //    osg::Vec4 color(1.0f,1.0f,1.0f,1.0f);
    osg::Vec4 color(.0f,.0f,.0f,1.0f);

    osg::Geode* geode = new osg::Geode();
    _switch->addChild(geode, true);

    // application description
    if (!_applicationUsage->getDescription().empty())
      {

        osg::ref_ptr<osgText::Text> label = new osgText::Text;
        geode->addDrawable( label.get() );

        label->setColor(color);
        //        label->setBackdropType(osgText::Text::OUTLINE);
        label->setFont(font);
        label->setCharacterSize(characterSize);
        label->setPosition(pos);
        label->setText(_applicationUsage->getDescription());

#if OPENSCENEGRAPH_MAJOR_VERSION <= 3 &&  OPENSCENEGRAPH_MINOR_VERSION < 4
        pos.x() = label->getBound().xMax();
#else
        pos.x() = label->getBoundingBox().xMax();
#endif
        pos.y() -= characterSize*2.0f;
      }

    const osg::ApplicationUsage::UsageMap& keyboardBinding = _applicationUsage->getKeyboardMouseBindings();

    for(osg::ApplicationUsage::UsageMap::const_iterator itr = keyboardBinding.begin();
        itr != keyboardBinding.end();
        ++itr)
      {
        pos.x() = leftPos;

        osg::ref_ptr<osgText::Text> key = new osgText::Text;
        geode->addDrawable( key.get() );
        key->setColor(color);
        //        key->setBackdropType(osgText::Text::OUTLINE);
        key->setFont(font);
        key->setCharacterSize(characterSize);
        key->setPosition(pos);
        key->setText(itr->first);

        pos.x() = startDescription;

        osg::ref_ptr<osgText::Text> description = new osgText::Text;
        geode->addDrawable( description.get() );
        description->setColor(color);
        //description->setBackdropType(osgText::Text::OUTLINE);
        description->setFont(font);
        description->setCharacterSize(characterSize);
        description->setPosition(pos);

        description->setText(itr->second);

        pos.y() -= characterSize*1.1f;

      }

    osg::BoundingBox bb = geode->getBoundingBox();
    if (bb.valid())
      {
        float width = bb.xMax() - bb.xMin();
        float height = bb.yMax() - bb.yMin();
        float ratio = 1.0;
        if (width > 1200.0f) ratio = 1200.0f/width;
        if (height*ratio > 950.0f) ratio = 950.0f/height;
        printf("ratio %f\n", ratio);

        _camera->setViewMatrix(osg::Matrix::translate(-bb.center()) *
                               osg::Matrix::scale(ratio,ratio,ratio) *
                               osg::Matrix::translate(osg::Vec3(640.0f, 540.0f, 0.0f)));
      }
  }


  void LpzHelpHandler::getUsage(osg::ApplicationUsage& usage) const
  {
    usage.addKeyboardMouseBinding("h","Onscreen help.");
  }

}
