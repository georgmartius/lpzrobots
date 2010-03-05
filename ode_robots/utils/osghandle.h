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
 *   Revision 1.7  2010-03-05 14:32:55  martius
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

namespace lpzrobots {

/** Data structure for accessing the ODE */
class OsgHandle
{
public:
  OsgHandle();

  OsgHandle( osg::Group* root, osg::Group* world, osg::Group* scene, 
             osg::TessellationHints* tesselhints[3], 
	     osg::StateSet* normalState, osg::StateSet* transparentState, 
	     const Color& color);

  ~OsgHandle();

  /// decides whether to draw bounding boxes 
  bool drawBoundings;   
  osg::Group* root;  // master note (contains world,hud..)
  osg::Group* world; // world note  (contains ground,sky and scene)
  osg::Group* scene; // actual scene for robots and stuff
  osg::TessellationHints* tesselhints[3];  
  osg::StateSet* normalState;  
  osg::StateSet* transparentState;  
  Color color;
  bool noGraphics;

  // returns a new osghandle with only the color changed
  OsgHandle changeColor(const Color& color) const;
  // returns a new osghandle with only the alpha channel changed
  OsgHandle changeAlpha(double alpha) const; 
  
};

}

#endif

