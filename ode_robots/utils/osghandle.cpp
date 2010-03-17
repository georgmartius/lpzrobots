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
 ***************************************************************************
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.11  2010-03-17 17:26:36  martius
 *   robotcameramanager uses keyboard and respects resize
 *   (robot) camera is has a conf object
 *   image processing implemented, with a few standard procedures
 *
 *   Revision 1.10  2010/03/17 08:46:08  martius
 *   tidy up
 *
 *   Revision 1.9  2010/03/16 15:48:02  martius
 *   osgHandle has now substructures osgConfig and osgScene
 *    that minimized amount of redundant data (this causes a lot of changes)
 *   Scenegraph is slightly changed. There is a world and a world_noshadow now.
 *    Main idea is to have a world without shadow all the time avaiable for the
 *    Robot cameras (since they do not see the right shadow for some reason)
 *   tidied up old files
 *
 *   Revision 1.8  2010/03/07 22:48:23  guettler
 *   moved shadow to OsgHandle.shadowType (TODO: move it to OsgConfig)
 *
 *   Revision 1.7  2010/03/05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *   Revision 1.6  2009/07/30 11:23:45  guettler
 *   new noGraphics state for OSGPrimitives
 *
 *   Revision 1.5  2009/07/29 14:19:49  jhoffmann
 *   Various bugfixing, remove memory leaks (with valgrind->memcheck / alleyoop)
 *
 *   Revision 1.4  2007/07/30 14:14:10  martius
 *   drawBoundings moved here
 *
 *   Revision 1.3  2006/12/11 18:26:55  martius
 *   destructor, but without any function
 *
 *   Revision 1.2  2006/07/14 12:23:56  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.7  2006/06/15 09:16:59  martius
 *   restores changeAlpha
 *
 *   Revision 1.1.2.6  2006/06/12 13:40:02  robot3
 *   changeAlpha is now a dummy method (for fast implement issues)
 *
 *   Revision 1.1.2.5  2006/06/12 13:37:12  robot3
 *   added missing const OsgHandle->changeAlpha(const float& alpha);
 *
 *   Revision 1.1.2.4  2006/01/12 14:39:06  martius
 *   transparent stateset
 *
 *   Revision 1.1.2.3  2005/12/29 16:48:06  martius
 *   changeColor
 *
 *   Revision 1.1.2.2  2005/12/22 14:10:14  martius
 *   different tesselhints
 *
 *   Revision 1.1.2.1  2005/12/13 18:12:20  martius
 *   some utils
 *
 *
 *                                                                 *
 ***************************************************************************/


#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>
#include <osgGA/StateSetManipulator>

#include "osghandle.h"
#include "robotcameramanager.h"

namespace lpzrobots {

  OsgHandle::OsgHandle( ) 
    : drawBoundings(false), scene(0), parent(0) {
  };



  void OsgHandle::init(){
    cfg = new OsgConfig();

    cfg->normalState = new osg::StateSet();
    cfg->normalState->ref();
    
    // set up blending for transparent stateset
    osg::StateSet* stateset = new osg::StateSet();
    osg::BlendFunc* transBlend = new osg::BlendFunc;
    transBlend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
    stateset->setAttributeAndModes(transBlend, osg::StateAttribute::ON);
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //stateset->setRenderBinDetails(5,"RenderBin");
    stateset->setMode(GL_CULL_FACE,osg::StateAttribute::ON); // disable backface because of problems
    
    cfg->transparentState = stateset;
    cfg->transparentState->ref();

    for(int i=0; i<3; i++) {
      cfg->tesselhints[i] = new osg::TessellationHints();
      cfg->tesselhints[i]->ref();
    }
    cfg->tesselhints[0]->setDetailRatio(0.1f); // Low
    cfg->tesselhints[1]->setDetailRatio(1.0f); // Middle
    cfg->tesselhints[2]->setDetailRatio(3.0f); // High

    scene = new OsgScene(); 

    color = Color(1,1,1,1);
  }

  void OsgHandle::close(){
    if(cfg->normalState)
      cfg->normalState->unref();
    if(cfg->transparentState)
      cfg->transparentState->unref();
    for(int i=0; i<3; i++) {
      if(cfg->tesselhints[i])
	cfg->tesselhints[i]->unref();
    }
    delete cfg;    
    // don't delete the camManager because it is deleted automatically (eventhandler)
    // delete scene->robotCamManager; 
    if(scene->world) scene->world->unref();
    if(scene->world_noshadow) scene->world_noshadow->unref();
    delete scene;
  }

  void OsgHandle::setup(int windowW, int windowH){
    scene->robotCamManager = new RobotCameraManager(windowW, windowH);    
  }
   
//   OsgHandle::OsgHandle( osg::Group* root, osg::Group* world, osg::Group* scene, 
//                         osg::TessellationHints* tesselhints[3], 
//                         osg::StateSet* normalState, osg::StateSet* transparentState,
//                         const Color& color, int shadowType)
//   {
//     this->root = root;
//     this->world = world;
//     this->worldNoShadow = world;
//     this->scene = scene;
//     this->robotCamManager = 0;
//     for(int i=0; i<3; i++){
//       this->tesselhints[i] = tesselhints[i];
//     }
//     this->normalState = normalState;
//     this->transparentState = transparentState;
//     this->color = color;
//     drawBoundings=false;
//     noGraphics=false;
//     this->shadowType=shadowType;
//   }

  OsgHandle::~OsgHandle(){
    // we should not delete any of the refs, because they are global
  }

  OsgHandle OsgHandle::changeColor(const Color& color) const {
    OsgHandle copy(*this);
    copy.color = color;
    return copy;
  }

  OsgHandle OsgHandle::changeAlpha(double alpha) const {
    OsgHandle copy(*this);
    copy.color.alpha() = alpha;
    return copy;
  }
}
