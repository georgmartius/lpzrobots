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


#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>
#include <osgGA/StateSetManipulator>

#include "osghandle.h"
#include "robotcameramanager.h"

namespace lpzrobots {

  OsgHandle::OsgHandle()
    : drawBoundings(false), cfg(0), scene(0), parent(0), color_set(0)
  {
  };



  void OsgHandle::init(){
    cfg = new OsgConfig();

    cfg->normalState = new osg::StateSet();
    cfg->normalState->ref();
    cfg->cs = new ColorSchema();
    // load colors....


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
    cfg->tesselhints[1]->setDetailRatio(1.0f); // Middle // maybe use 0.5 here
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
    if(cfg->cs) delete cfg->cs;
    delete cfg;
    cfg=0;
    // don't delete the camManager because it is deleted automatically (eventhandler)
    // delete scene->robotCamManager;
    if(scene->world) scene->world->unref();
    if(scene->world_noshadow) scene->world_noshadow->unref();
    delete scene;
    scene=0;
  }

  void OsgHandle::setup(int windowW, int windowH){
    scene->robotCamManager = new RobotCameraManager(windowW, windowH);
  }

  OsgHandle::~OsgHandle(){
    // we should not delete any of the refs, because they are global
  }

  OsgHandle OsgHandle::changeColor(const Color& color) const {
    OsgHandle copy(*this);
    copy.color = color;
    return copy;
  }

  OsgHandle OsgHandle::changeColor(double r, double g, double b, double a) const {
    OsgHandle copy(*this);
    copy.color = Color(r,g,b,a);
    return copy;
  }

  OsgHandle OsgHandle::changeAlpha(double alpha) const {
    OsgHandle copy(*this);
    copy.color.alpha() = alpha;
    return copy;
  }

  OsgHandle OsgHandle::changeColor(const std::string& name) const {
    OsgHandle copy(*this);
    if(cfg && cfg->cs)
      copy.color = cfg->cs->color(name,color_set);
    return copy;
  }

  Color OsgHandle::getColor(const std::string& name) const {
    if(cfg && cfg->cs)
      return cfg->cs->color(name,color_set);
    else {
      return Color();
    }
  }


  OsgHandle OsgHandle::changeColorDef(const std::string& name, const Color& defcolor) const{
    OsgHandle copy(*this);
    if(cfg && cfg->cs){
      if(!cfg->cs->color(copy.color, name,color_set)){
        copy.color = defcolor;
      }
    }
    return copy;
  }

  ColorSchema* OsgHandle::colorSchema(){
    return cfg->cs;
  }

  const ColorSchema* OsgHandle::colorSchema() const {
    return cfg->cs;
  }


  /** returns a new osghandle with a changed color (alias) set */
  OsgHandle OsgHandle::changeColorSet(int color_set) const {
    OsgHandle copy(*this);
    copy.setColorSet(color_set);
    return copy;
  }

  void OsgHandle::setColorSet(int color_set) {
    if(color_set>0)
      this->color_set=color_set;
  }



}
