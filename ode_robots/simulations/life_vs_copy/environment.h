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
 *                                                                 *
 ***************************************************************************/

#ifndef __ENVIRONMENT_H
#define __ENVIRONMENT_H

#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>

#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>
#include <ode_robots/seesaw.h>
#include <ode_robots/boxpile.h>

using namespace lpzrobots;
using namespace std;


class Env : public Configurable {
public:
  enum EnvType { None, Normal, Octa, Pit, Pit2, OpenPit, Uterus, Stacked };

  Env(EnvType t = None) : Configurable("Environment","1.0") {
    type         = t;
    widthground  = 25.85;
    heightInner  = 2;
    pitsize      = 1;
    pitPosition  = Pos(0,0,0);
    height       = 0.8;
    thickness    = 0.2;
    uterussize   = 1;
    roughness    = 2;
    hardness     = 10;
    numgrounds   = 2;
    distance     = 2;
    useColorSchema = false;


    numSpheres  = 0;
    numBoxes    = 0;
    numCapsules = 0;
    numSeeSaws  = 0;
    numBoxPiles = 0;

    if(t==Octa || t==Pit || t==None){
      addParameter("pitheight", &heightInner, 0, 10, "height of circular pit");
      addParameter("pitsize",   &pitsize,     0, 10, "size of the pit (diameter)");
    }
    addParameter("height",    &height,      0, 10, "height of square walls");
    addParameter("roughness", &roughness,   0, 10,
                 "roughness of ground (friction parameters) (and walls in pit)");
    addParameter("hardness", &hardness,   0, 50,
                 "hardness of ground (friction parameters) (and walls in pit)");
    if(t==Stacked){
      addParameter("numgrounds", &numgrounds, 0, 10,
                   "number of stacked rectangular playgrounds");
      addParameter("distance", &distance, 0.2, 10,
                   "distance between stacked rectangular playgrounds");
      addParameter("heightincrease", &heightincrease, 0., 1,
                   "height increase between subsequent stacked rectangular playgrounds");
    }
    if(t==Normal || t==Octa || t==Stacked || t==None){
      addParameter("size", &widthground, 0.2, 20,
                   "size of rectangular playgrounds");
    }
  }

  EnvType type;
  std::list<AbstractGround*> playgrounds;

  // playground parameter
  double widthground;
  double heightground;
  double heightInner;
  double pitsize;
  double height;
  double thickness;
  double uterussize;
  double roughness;
  double hardness;

  Pos pitPosition;
  Pos pit2Position;

  int    numgrounds;
  double distance;
  double heightincrease;

  bool useColorSchema; // playgrounds are black and white and get color from schema

  // obstacles
  int numSpheres;
  int numBoxes;
  int numCapsules;
  int numSeeSaws;
  int numBoxPiles;

  OdeHandle odeHandle;
  OsgHandle osgHandle;
  GlobalData* global ;


  /** creates the Environment
   */
  void create(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
              GlobalData& global, bool recreate=false){
    this->odeHandle=odeHandle;
    this->osgHandle=osgHandle;
    this->global=&global;
    if(recreate && !playgrounds.empty()){
      FOREACH(std::list<AbstractGround*>, playgrounds, p){
        removeElement(global.obstacles, *p);
        delete (*p);
      }
      playgrounds.clear();
    }
    AbstractGround* playground;
    switch (type){
    case Octa:
    case Normal:
      {
        playground = new Playground(odeHandle, osgHandle,
                                    osg::Vec3(widthground, 0.208, height));
        //     playground->setTexture("Images/really_white.rgb");
        //        playground->setGroundTexture("Images/yellow_velour.rgb");
        if(useColorSchema)
          playground->setTexture(0,0,TextureDescr("Images/wall_bw.jpg",-1.5,-3)); // was: wall.rgb
        playground->setGroundThickness(0.2);
        playground->setPosition(osg::Vec3(0,0,.0));
        Substance substance(roughness, 0.0, hardness, 0.95);
        playground->setGroundSubstance(substance);
        global.obstacles.push_back(playground);
        playgrounds.push_back(playground);
      }
      if (type==Octa) { // this playground comes second for dropping obstacles
        playground = new OctaPlayground(odeHandle, osgHandle,
                                        osg::Vec3(pitsize, thickness, heightInner), 12, false);
        playground->setTexture("Images/really_white.rgb");
        Color c = osgHandle.getColor("Monaco");
        c.alpha()=0.15;
        playground->setColor(c);
        playground->setPosition(pitPosition); // playground positionieren und generieren
        global.obstacles.push_back(playground);
        playgrounds.push_back(playground);
      }
      break;
    case Pit2:
    case Pit:
      {
        Substance soft = Substance::getPlastic(2);
        soft.roughness = roughness;
        soft.hardness  = hardness;
        OdeHandle myHandle = odeHandle;
        myHandle.substance = soft;
        playground = new Playground(myHandle, osgHandle,
                                    osg::Vec3(pitsize, thickness, height),
                                    1, true);
        playground->setTexture("Images/really_white.rgb");
        playground->setGroundSubstance(soft);

        Color c = osgHandle.getColor("Monaco");
        c.alpha()=0.15;
        playground->setColor(c);
        playground->setPosition(pitPosition); // playground positionieren und generieren
        global.obstacles.push_back(playground);
        playgrounds.push_back(playground);
      }
      if(type==Pit2){
        Substance soft = Substance::getPlastic(2);
        soft.roughness = roughness;
        soft.hardness  = hardness;
        OdeHandle myHandle = odeHandle;
        myHandle.substance = soft;
        playground = new Playground(myHandle, osgHandle,
                                    osg::Vec3(pitsize, thickness, height),
                                    1, true);
        playground->setTexture("Images/really_white.rgb");
        playground->setGroundSubstance(soft);

        Color c = osgHandle.getColor("Monaco");
        c.alpha()=0.15;
        playground->setColor(c);
        playground->setPosition(pit2Position); // playground positionieren und generieren
        global.obstacles.push_back(playground);
        playgrounds.push_back(playground);
      }
      break;
    case OpenPit:
    case Uterus:
      {
        // we stack two playgrounds in each other.
        // The outer one is hard (and invisible) and the inner one is soft
        int anzgrounds=2;
        // this is the utterus imitation: high slip, medium roughness, high elasticity, soft
        Substance uterus(roughness, 0.1 /*slip*/,
                         hardness, 0.95 /*elasticity*/);
        double thickness = 0.4;
        for (int i=0; i< anzgrounds; i++){
          OdeHandle myHandle = odeHandle;
          if(i==0){
            myHandle.substance = uterus;
          }else{
            myHandle.substance.toMetal(.2);
          }
          Playground* playground = new Playground(myHandle, osgHandle,
                                                  osg::Vec3(uterussize+2*thickness*i,
                                                            i==0 ? thickness : .5, height),
                                                  1, i==0);
          playground->setTexture("Images/dusty.rgb");
          if(i==0){ // set ground also to the soft substance
            playground->setGroundSubstance(uterus);
          }
          playground->setColor(Color(0.5,0.1,0.1,i==0? .2 : 0)); // outer ground is not visible (alpha=0)
          playground->setPosition(osg::Vec3(0,0,i==0? thickness : 0 )); // playground positionieren und generieren
          global.obstacles.push_back(playground);
          playgrounds.push_back(playground);
        }

        break;
      }
    case Stacked:
      {
        for (int i=0; i< numgrounds; i++){
          playground = new Playground(odeHandle, osgHandle,
                                      osg::Vec3(widthground+distance*i, .2,
                                                0.2+height+heightincrease*i),
                                      1, i==(numgrounds-1));

          if(useColorSchema)
            playground->setTexture(0,0,TextureDescr("Images/wall_bw.jpg",-1.5,-3));
          playground->setGroundThickness(0.2);
          playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
          Substance substance(roughness, 0.0, hardness, 0.95);
          playground->setGroundSubstance(substance);

          global.obstacles.push_back(playground);
          playgrounds.push_back(playground);
        }
        break;
      }
    default:
      break;
    }
  }

  void placeObstacles(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      GlobalData& global){


    for(int i=0; i<numSeeSaws; i++){
      Seesaw* seesaw = new Seesaw(odeHandle, osgHandle);
      seesaw->setColor("wall");
      seesaw->setPose(ROTM(M_PI/2.0,0,0,1)*TRANSM(1, -i,.0));
      global.obstacles.push_back(seesaw);
    }

    for(int i=0; i<numBoxPiles; i++){
      Boxpile* boxpile = new Boxpile(odeHandle, osgHandle);
      boxpile->setColor("wall");
      boxpile->setPose(ROTM(M_PI/5.0,0,0,1)*TRANSM(-5, -5-5*i,0.2));
      global.obstacles.push_back(boxpile);
    }

    for(int i=0; i<numSpheres; i++){
      PassiveSphere* s =
        new PassiveSphere(odeHandle, osgHandle.changeColor("Monaco"), 0.2);
      s->setPosition(Pos(i*0.5-2, 3+i*4, 1.0));
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);
    }

    for(int i=0; i<numBoxes; i++){
      PassiveBox* b =
        new PassiveBox(odeHandle, osgHandle.changeColor("Weissgrau"),
                       osg::Vec3(0.4+i*0.1,0.4+i*0.1,0.4+i*0.1));

      b->setTexture("Images/light_chess.rgb");
      b->setPosition(Pos(i*0.5-5, i*0.5, 1.0));
      global.obstacles.push_back(b);
    }

    for(int i=0; i<numCapsules; i++){
      PassiveCapsule* c =
        new PassiveCapsule(odeHandle, osgHandle, 0.2f, 0.3f, 0.3f);
      c->setPosition(Pos(i-1, -i, 1.0));
      c->setColor(Color(0.2f,0.2f,1.0f,0.5f));
      c->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(c);
    }

  }

  virtual void notifyOnChange(const paramkey& key){
    pitsize = max(pitsize,0.3);
    create(odeHandle,osgHandle,*global,true);
  }

};


#endif

