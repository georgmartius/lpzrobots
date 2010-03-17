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
 *   Revision 1.10  2010-03-17 17:26:36  martius
 *   robotcameramanager uses keyboard and respects resize
 *   (robot) camera is has a conf object
 *   image processing implemented, with a few standard procedures
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
 *   Revision 1.4  2007/07/30 14:13:40  martius
 *   drawBoundings moved here
 *
 *   Revision 1.3  2006/12/11 18:26:55  martius
 *   destructor, but without any function
 *
 *   Revision 1.2  2006/07/14 12:23:56  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.8  2006/06/29 16:39:56  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.1.2.7  2006/06/15 09:17:09  martius
 *   restores changeAlpha
 *
 *   Revision 1.1.2.6  2006/06/12 13:37:55  robot3
 *   added missing const OsgHandle->changeAlpha(const float& alpha);
 *
 *   Revision 1.1.2.5  2006/01/12 14:39:06  martius
 *   transparent stateset
 *
 *   Revision 1.1.2.4  2005/12/29 16:48:06  martius
 *   changeColor
 *
 *   Revision 1.1.2.3  2005/12/22 14:09:56  martius
 *   different tesselhints for different level of detail
 *
 *   Revision 1.1.2.2  2005/12/13 18:12:20  martius
 *   some utils
 *
 *   Revision 1.1.2.1  2005/12/06 16:18:02  martius
 *   handle class for OpenSceneGraph
 *
 *
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

  // Georg: removed. Brauchen wir den wirklich?
//   /**
//    * TODO: Separation of OSGHandle and OSGConfig
//    * @param root
//    * @param world
//    * @param scene
//    * @param tesselhints
//    * @param normalState
//    * @param transparentState
//    * @param color
//    * @param shadowType
//    * @return
//    */
//   OsgHandle( osg::Group* root, osg::Group* world, osg::Group* scene, 
//              osg::TessellationHints* tesselhints[3], 
// 	     osg::StateSet* normalState, osg::StateSet* transparentState, 
// 	     const Color& color, int shadowType);

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

  OsgConfig* cfg;
  OsgScene*  scene;
  osg::Group* parent; // the place there individual osgprimitives are added

  // returns a new osghandle with only the color changed
  OsgHandle changeColor(const Color& color) const;
  // returns a new osghandle with only the alpha channel changed
  OsgHandle changeAlpha(double alpha) const; 
  
};



}

#endif

