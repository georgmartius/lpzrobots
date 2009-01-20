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
 *   Revision 1.12  2009-01-20 17:27:34  martius
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

#include<ode/ode.h>
#include<osg/Transform>
#include <osgText/Text>

#include "osghandle.h"
#include "odehandle.h"

#include "hudstatistics.h"

class osg::Node;
class Callbackable;


namespace lpzrobots {

  class MoveEarthySkyWithEyePointTransform : public osg::Transform {
    public:
      /** Get the transformation matrix which moves from local coords to world coords.*/
      virtual bool computeLocalToWorldMatrix(osg::Matrix& matrix,osg::NodeVisitor* nv) const;

      /** Get the transformation matrix which moves from world coords to local coords.*/
      virtual bool computeWorldToLocalMatrix(osg::Matrix& matrix,osg::NodeVisitor* nv) const;
  };

  class Base {
  public:
    Base(const char* caption=0);

    virtual osg::Group* makeScene();
    virtual osg::Node* makeSky();
    virtual osg::Node* makeGround();
    virtual osg::Node* createHUD();
    virtual osg::LightSource* makeLights(osg::StateSet* stateset);
    virtual osg::Group* createShadowedScene(osg::Node* shadowed,
					    osg::Vec3 posOfLight,
					    unsigned int unit);
    /** Shadow types:
     * 1 - ShadowVolume
     * 2 - ShadowTextue
     * 3 - ParallelSplitShadowMap
     * 4 - SoftShadowMap
     * 5 - ShadowMap
     */
    virtual osg::Group* createShadowedScene(osg::Node* sceneToShadow);

    virtual void setGroundTexture(const char* filename) {
      this->groundTexture = filename;
    }

    virtual void setCaption(const char* caption) {
      this->caption = caption;
    }

  /** adds an Callbackable object for getting a callback every step.
   * note that the object are not called back in this class. This must
   * be done in the deduced class (here: Simulation).
   */
    virtual void addGraphicsCallbackable(Callbackable* callbackable);

    virtual void addPhysicsCallbackable(Callbackable* callbackable);

    virtual HUDStatisticsManager* getHUDSM() { return this->hUDStatisticsManager; }

    virtual ~Base();

  protected:
    virtual void setTimeStats(double time, double realtimefactor,
			      double truerealtimefactor,bool pause);

    dGeomID ground;


    OsgHandle osgHandle;
    // ODE globals
    OdeHandle odeHandle;
    const char* caption;
    std::string groundTexture;

    osg::Group* root;
    osg::Node* hud;
    osgText::Text* timestats;
    osgText::Text* statisticLine;

    /// this manager provides methods for displaying statistics on the graphical window!
    HUDStatisticsManager* hUDStatisticsManager;

    std::list<Callbackable*> graphicsCallbackables;
    std::vector<Callbackable*> physicsCallbackables; // vector because we need to parallelise it

    int ReceivesShadowTraversalMask;
    int CastsShadowTraversalMask;

    // the types are double because they are configurable and stored to the cfg file
    double shadow;     // set by child class Simulation
    double shadowTexSize;  // set by child class Simulation
    double useNVidia;      // if 0 use ATI Radeon!


  public:
    // Helper
    /// returns the index+1 if the list contains the given string or 0 if not
    static int contains(char **list, int len,  const char *str);


  };
}

#endif
