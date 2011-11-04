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
#ifndef __OSGHANDLE_H
#define __OSGHANDLE_H

#include "osgforwarddecl.h"
#include "color.h"

namespace osgShadow {
  class ShadowedScene;
}

namespace lpzrobots {

  class RobotCameraManager;


  /** Data structure containing some configuration variables for OSG */
  struct OsgConfig {
    OsgConfig() : normalState(0), transparentState(0), noGraphics(false) {}
    osg::TessellationHints* tesselhints[3];  
    osg::StateSet* normalState;  
    osg::StateSet* transparentState;  
    int shadowType;
    bool noGraphics;        
  };

  /** Data structure containing the scene notes (e.g. with and without shadow)*/
  struct OsgScene {
    OsgScene() :  root(0), world(0),world_noshadow(0),scene(0),
                  shadowedScene(0), shadowedSceneRoot(0), groundScene(0), 
                  lightSource(0), worldtransform(0),
                  robotCamManager(0) {}
    osg::Group* root;  // master note (contains world,hud..)
    osg::Group* world; // world note  (contains ground,sky and shadowed scene)
    osg::Group* world_noshadow; // world note without shadow (contains ground,sky and scene)
    osg::Group* scene; // actual scene for robots and stuff    

    osgShadow::ShadowedScene* shadowedScene;
    osg::Group* shadowedSceneRoot; // root node of shadowed scene 
    osg::Node* groundScene;

    osg::LightSource* lightSource;  // the light source
    osg::Transform* worldtransform; // unit transformation at the moment

    RobotCameraManager* robotCamManager; // manages robot cameras and their display
  };

  


/** Data structure for accessing the OpenSceneGraph */
class OsgHandle
{
public:
  OsgHandle();

  ~OsgHandle();

  /// initialization of the structure
  void init();
  /// set up robotcameramanager (must be called after init but before usage of the structure)
  void setup(int windowW, int windowH);
  /// deletes all internal variables
  void close();

  /// decides whether to draw bounding boxes 
  bool drawBoundings;   

  Color color;    

  OsgConfig* cfg; // the config is shared
  OsgScene*  scene;  // the scene is shared
  osg::Group* parent; // the place where individual osgprimitives are added

  // returns a new osghandle with only the color changed
  OsgHandle changeColor(const Color& color) const;
  // returns a new osghandle with only the color changed
  OsgHandle changeColor(double r, double g, double b, double a=1.0) const;
  // returns a new osghandle with only the alpha channel changed
  OsgHandle changeAlpha(double alpha) const; 
  
};



}

#endif

