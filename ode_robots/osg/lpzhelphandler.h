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

#ifndef LPZHELPHANDLER_H
#define LPZHELPHANDLER_H

#include <osgViewer/ViewerEventHandlers>

namespace lpzrobots {


/** Event handler for adding on screen help to Viewers.*/
class LpzHelpHandler : public osgGA::GUIEventHandler 
{
    public: 

        LpzHelpHandler(osg::ApplicationUsage* au=0);
        
        void setApplicationUsage(osg::ApplicationUsage* au) { _applicationUsage = au; }
        osg::ApplicationUsage* getApplicationUsage() { return _applicationUsage.get(); }
        const osg::ApplicationUsage* getApplicationUsage() const { return _applicationUsage.get(); }

        void setKeyEventTogglesOnScreenHelp(int key) { _keyEventTogglesOnScreenHelp = key; }
        int getKeyEventTogglesOnScreenHelp() const { return _keyEventTogglesOnScreenHelp; }
        
        void reset();

        osg::Camera* getCamera() { return _camera.get(); }
        const osg::Camera* getCamera() const { return _camera.get(); }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

        /** Get the keyboard and mouse usage of this manipulator.*/
        virtual void getUsage(osg::ApplicationUsage& usage) const;

    protected:

        void setUpHUDCamera(osgViewer::ViewerBase* viewer);

        void setUpScene(osgViewer::ViewerBase* viewer);
        
        osg::ref_ptr<osg::ApplicationUsage> _applicationUsage;

        int                                 _keyEventTogglesOnScreenHelp;

        bool                                _helpEnabled;

        bool                                _initialized;
        osg::ref_ptr<osg::Camera>           _camera;
        osg::ref_ptr<osg::Switch>           _switch;
        
};

}
#endif
