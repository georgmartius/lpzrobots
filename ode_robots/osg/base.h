/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *  base.h provides osg stuff for basic environment with sky and so on.    *
 *                                                                         *
 *   $Log$
 *   Revision 1.31  2011-03-22 16:42:25  guettler
 *   - adpaptions to enhanced configurable and inspectable interface
 *   - agents are always in globalData.configs
 *   - showParams now done by simulations base code instead manually called
 *     in method start()
 *   - showParams(ConfigList&) is marked as deprecated
 *   - Configurable inheritance of Simulation moved to Base, Base is no longer derived
 *     from BackCaller (because Configurable is derived from BackCaller)
 *   - removed some old deprecated member lists in base
 *
 *   Revision 1.30  2010/09/23 08:35:22  martius
 *   light generation improved
 *
 *   Revision 1.29  2010/03/17 09:33:16  martius
 *   removed memory leaks and some small bugs
 *   valgrind suppression file is updated
 *
 *   Revision 1.28  2010/03/16 15:47:46  martius
 *   osgHandle has now substructures osgConfig and osgScene
 *    that minimized amount of redundant data (this causes a lot of changes)
 *   Scenegraph is slightly changed. There is a world and a world_noshadow now.
 *    Main idea is to have a world without shadow all the time avaiable for the
 *    Robot cameras (since they do not see the right shadow for some reason)
 *   tidied up old files
 *
 *   Revision 1.27  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.26  2010/03/07 22:39:08  guettler
 *   moved shadow to OsgHandle.shadowType (TODO: move it to OsgConfig)
 *
 *   Revision 1.25  2010/03/05 14:32:55  martius
 *   camera sensor added
 *   for that the scenegraph structure was changed into root, world, scene
 *   camera does not work with shadows
 *   works with newest version of ode (0.11)
 *
 *   Revision 1.24  2009/11/26 10:06:45  martius
 *   added createHUDManager function
 *
 *   Revision 1.23  2009/09/03 12:53:25  guettler
 *   reverted changes of revision 1.37:
 *   - getHUDSM is called without calling createHUDSM before
 *     if simulation is started with -nographics
 *   FIX: createHUDSM calls now getHUDSM (fix of HUDSM was not complete)
 *
 *   Revision 1.22  2009/08/10 14:55:13  der
 *   shadowTexSize and shadow are integer
 *
 *   Revision 1.21  2009/08/10 08:40:10  guettler
 *   changed g&uumlettler to guettler
 *
 *   Revision 1.20  2009/08/10 08:37:17  guettler
 *   try to set guettler in caption to correct characters
 *
 *   Revision 1.19  2009/08/10 07:54:32  guettler
 *   - uses new BackCaller implementation
 *   - bugfix: avoid crash if noGraphics when getting HUDSM
 *
 *   Revision 1.18  2009/08/07 13:27:18  martius
 *   makePhysicalScene to create phyiscal scene independent of graphical scene
 *     (to cope with new noGraphics implementation)
 *
 *   Revision 1.17  2009/08/05 23:23:42  martius
 *   corrected coding in "Guettler"
 *
 *   Revision 1.16  2009/07/30 12:09:12  guettler
 *   commented out unused variables
 *
 *   Revision 1.15  2009/03/31 15:45:50  martius
 *   caption is a std::string and can be changed on-line
 *
 *   Revision 1.14  2009/03/27 06:21:31  guettler
 *   CTRL +S  changes now the shadow type in the simulation: cleaned up the code
 *
 *   Revision 1.13  2009/03/25 15:44:23  guettler
 *   ParallelSplitShadowMap: corrected light direction (using directional light), complete ground is now shadowed
 *
 *   Revision 1.12  2009/01/20 17:27:34  martius
 *   texture for background changeable from outside
 *
 *   Revision 1.11  2008/09/11 15:24:01  martius
 *   motioncallback resurrected
 *   noContact substance
 *   use slider center of the connecting objects for slider drawing
 *
 *   Revision 1.10  2008/05/05 09:35:35  guettler
 *   hud now displays if in pause mode
 *
 *   Revision 1.9  2008/04/23 07:17:16  martius
 *   makefiles cleaned
 *   new also true realtime factor displayed,
 *    warning if out of sync
 *   drawinterval in full speed is 10 frames, independent of the speed
 *
 *   Revision 1.8  2008/04/17 15:59:00  martius
 *   OSG2 port finished
 *
 *   Revision 1.7.2.5  2008/04/11 13:46:50  martius
 *   quickMP multithreading included
 *
 *   Revision 1.7.2.4  2008/04/11 10:41:35  martius
 *   config file added
 *
 *   Revision 1.7.2.3  2008/04/09 14:25:35  martius
 *   shadow cmd line option
 *
 *   Revision 1.7.2.2  2008/04/09 13:57:59  guettler
 *   New ShadowTechnique added.
 *
 *   Revision 1.7.2.1  2008/04/09 10:18:41  martius
 *   fullscreen and window options done
 *   fonts on hud changed
 *
 *   Revision 1.7  2007/09/28 12:31:49  robot3
 *   The HUDSM is not anymore deduced from StatisticalTools, so the statistics
 *   can be updated independently from the HUD
 *   addPhysicsCallbackable and addGraphicsCallbackable now exists in Simulation
 *
 *   Revision 1.6  2007/09/28 10:24:05  robot3
 *   The WindowStatisticsManager is now called HUDStatisticsManager
 *
 *   Revision 1.5  2007/09/27 10:47:04  robot3
 *   mathutils: moved abs to selforg/stl_adds.h
 *   simulation,base: added callbackable support,
 *   added WSM (HUDStatisticsManager) funtionality
 *
 *   Revision 1.4  2007/08/29 13:07:48  martius
 *   added HUD
 *
 *   Revision 1.3  2006/09/20 15:30:47  martius
 *   shadowsize, light
 *
 *   Revision 1.2  2006/07/14 12:23:33  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.7  2006/06/29 16:35:56  robot3
 *   includes cleared up
 *
 *   Revision 1.1.2.6  2006/05/28 22:14:56  martius
 *   heightfield included
 *
 *   Revision 1.1.2.5  2006/05/18 11:45:51  robot3
 *   -shadowing the normal scene integrated (first version)
 *   -note that there is a bug that the shadow disappears
 *    after some time (e.g. 60 minutes)
 *
 *   Revision 1.1.2.4  2006/01/31 15:45:02  martius
 *   virtual destructor
 *
 *   Revision 1.1.2.3  2006/01/12 14:21:00  martius
 *   drawmode, material
 *
 *   Revision 1.1.2.2  2005/12/09 16:54:16  martius
 *   camera is woring now
 *
 *   Revision 1.1.2.1  2005/12/06 17:40:59  martius
 *   base class for simulation
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __BASE_H
#define __BASE_H

#include <ode-dbl/ode.h>
#include <osg/Transform>
#include <osgText/Text>

#include "osghandle.h"
#include "odehandle.h"

#include "hudstatistics.h"
#include <selforg/configurable.h>

namespace osgShadow
{
  class ShadowedScene;
}

namespace lpzrobots
{

  class MoveEarthySkyWithEyePointTransform : public osg::Transform
  {
    public:
      /** Get the transformation matrix which moves from local coords to world coords.*/
      virtual bool computeLocalToWorldMatrix(osg::Matrix& matrix, osg::NodeVisitor* nv) const;

      /** Get the transformation matrix which moves from world coords to local coords.*/
      virtual bool computeWorldToLocalMatrix(osg::Matrix& matrix, osg::NodeVisitor* nv) const;
  };

  class Base : public Configurable
  {
  public:
    Base(const std::string& caption="lpzrobots Simulator          Martius, Der, Guettler");

    static const int PHYSICS_CALLBACKABLE = 1; //!< called each ode/physics step
    static const int GRAPHICS_CALLBACKABLE = 2; //!< called each osg/draw step

    /// create the ground plane
    virtual void makePhysicsScene(); 
    /** creates the base scene graph with world, sky and floor and shadow and HUD
        and stores it in scene
     */
    virtual void makeScene(OsgScene* scene);
    virtual osg::Node* makeSky();
    virtual osg::Node* makeGround();
    virtual osg::Node* createHUD();
    virtual void createHUDManager(osg::Geode* geode, osgText::Font* font);
    /// adds light to the node
    virtual void makeLights(osg::Group* node);

    /** Shadow types:
     * 1 - ShadowVolume
     * 2 - ShadowTextue
     * 3 - ParallelSplitShadowMap
     * 4 - SoftShadowMap
     * 5 - ShadowMap
     */
    virtual osgShadow::ShadowedScene* createShadowedScene(osg::Node* sceneToShadow, osg::LightSource* lightSource, int shadowType);

    virtual void setGroundTexture(const char* filename) {
      this->groundTexture = filename;
    }

    virtual void setCaption(const std::string& caption);

    /**
     * Create HUDStatisticsManager and register it for being called back every step.
     * But do not display if the system is initialised with -nographics.
     * @return the actual HUDStatisticsManager
     */
    virtual HUDStatisticsManager* getHUDSM();

    virtual ~Base();

  protected:
    virtual void setTimeStats(double time, double realtimefactor,
			      double truerealtimefactor,bool pause);

    /**
     * Changes the currently used shadow technique.
     * The switch is realized between:
     * 0 - NoShadow
     * 3 - ParallelSplitShadowMap
     * 4 - SoftShadowMap
     * 5 - ShadowMap (simple)
     * Currently not supported by this function:
     * 1 - ShadowVolume
     * 2 - ShadowTextue
     */
    virtual void changeShadowTechnique();

    /// deletes the stuff that is created on makescene and the like
    virtual void base_close();

    dGeomID ground;


    OsgHandle osgHandle;
    // ODE globals
    OdeHandle odeHandle;
    std::string caption;
    std::string groundTexture;

    osg::Group* dummy;

    osg::Node* hud;
    osgText::Text* timestats;
    osgText::Text* captionline;
    osgText::Text* statisticLine;

    Primitive* plane;

    /// this manager provides methods for displaying statistics on the graphical window!
    HUDStatisticsManager* hUDStatisticsManager;

    int ReceivesShadowTraversalMask;
    int CastsShadowTraversalMask;

    // the types are double because they are configurable and stored to the cfg file
    // int shadow;     // set by child class Simulation, now found in OsgHandle
    int shadowTexSize;  // set by child class Simulation
    bool useNVidia;      // unused: if false use ATI Radeon!


  public:
    // Helper
    /// returns the index+1 if the list contains the given string or 0 if not
    static int contains(char **list, int len,  const char *str);

  };
}

#endif
