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

  struct StatLineProperties {
    StatLineProperties(int fontSizeTime, int fontSizeText, const std::string& fontColor)
      :fontSizeTime(fontSizeTime), fontSizeText(fontSizeText), fontColor(fontColor) {
    }
    int fontSizeTime;
    int fontSizeText;
    std::string fontColor;
  };

  class Base : public Configurable
  {
  public:
    Base(const std::string& caption="LpzRobots Simulator (Martius et al)");

    static const int PHYSICS_CALLBACKABLE = 1; //!< called each ode/physics step
    static const int GRAPHICS_CALLBACKABLE = 2; //!< called each osg/draw step

    /// create the ground plane
    virtual void makePhysicsScene();
    /** creates the base scene graph with world, sky and floor and shadow and HUD
        and stores it in scene
     */
    virtual void makeScene(OsgScene* scene, const OsgConfig& config);
    virtual osg::Node* makeSky(const OsgConfig& config);
    virtual osg::Node* makeGround(const OsgConfig& config);
    /** creates hud and is supposed to return the camera to it and
        adds the geode of the hud to the scene */
    virtual osg::Node* createHUD(OsgScene* scene, const OsgConfig& config);
    virtual void createHUDManager(osg::Geode* geode, osgText::Font* font);
    /// adds light to the node
    virtual void makeLights(osg::Group* node, const OsgConfig& config);

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

    virtual Substance getGroundSubstance();
    virtual void setGroundSubstance(const Substance& substance);

    /// sets the cpation that is printed at the right of the status line
    virtual void setCaption(const std::string& caption);

    /// sets the title that is printed in the center of the status line
    virtual void setTitle(const std::string& title);

    virtual StatLineProperties getStatLineProperties() { return statlineprop; }
    /// sets the properties of the status line, do it before the scene is initialized
    virtual void setStatLineProperties(const StatLineProperties& statlineprop){
      this->statlineprop = statlineprop;
    }

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
    std::string title;
    std::string groundTexture;

    osg::Group* dummy;

    osg::Node* hud;
    osgText::Text* timestats;
    osgText::Text* captionline;
    osgText::Text* titleline;
    StatLineProperties statlineprop;

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
