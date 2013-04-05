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

using namespace lpzrobots;
using namespace std;


class Env {
public:
  enum EnvType { None, Normal, Octa, Pit, Uterus, Stacked };

  Env(){
    type         = None;
    playground   = 0;
    widthground  = 25.85;
    heightground = .8;
    diamOcta     = 2;
    uterussize   = 1;

    numSpheres  = 0;
    numBoxes    = 0;
    numCapsules = 0;
  }

  AbstractGround* playground;
  EnvType type;

  // playground parameter
  double widthground;
  double heightground;
  double diamOcta;
  double pitsize;
  double pitheight;
  double uterussize;

  // obstacles
  int numSpheres;
  int numBoxes;
  int numCapsules;

  /** creates the Environment
   */
  void create(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
              GlobalData& global, bool recreate=false){
    if(recreate && playground){
      removeElement(global.obstacles, playground);
      delete playground;
      playground=0;
    }

    switch (type){
    case Normal:
      {
        playground = new Playground(odeHandle, osgHandle,osg::Vec3(widthground, 0.208, heightground));
        //     playground->setTexture("Images/really_white.rgb");
        //        playground->setGroundTexture("Images/yellow_velour.rgb");
        playground->setTexture(0,0,TextureDescr("Images/wall_bw.jpg",-1.5,-3)); // was: wall.rgb
        playground->setPosition(osg::Vec3(0,0,.03));
        // CHECK:
        /* Substance substance; */
        /* substance.toRubber(5); */
        /* playground->setGroundSubstance(substance); */
        global.obstacles.push_back(playground);

        Seesaw* seesaw = new Seesaw(odeHandle, osgHandle);
        seesaw->setColor("wall");
        seesaw->setPose(ROTM(M_PI/2.0,0,0,1)*TRANSM(1,-0,.0));
        global.obstacles.push_back(seesaw);
        break;
      }
    case Octa:
      {
        playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(diamOcta, 0.2,/*Height*/ 10), 12,false);
        playground->setTexture("Images/really_white.rgb");
        playground->setColor(Color(0.4,0.8,0.4,0.2));
        playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
        global.obstacles.push_back(playground);
        break;
      }
    case Pit:
      {
        //TODO: change playground refs....
        // we stack two playgrounds in each other.
        // The outer one is hard and the inner one is softer
        int       anzgrounds    = 2;
        Substance soft          = Substance::getRubber(5);
        double    thicknessSoft = 0.1;
        for (int i=0; i< anzgrounds; i++){
          OdeHandle myHandle = odeHandle;
          if(i==0){
            myHandle.substance = soft;
          }else{
            myHandle.substance.toMetal(1);
          }
          Playground* playground = new Playground(myHandle, osgHandle,
                                                  osg::Vec3(pitsize+2*thicknessSoft*i, thicknessSoft + 12*i, pitheight),
                                                  1, i==(anzgrounds-1));
          if(i==(anzgrounds-1)){ // set ground also to the soft substance
            playground->setGroundSubstance(soft);
          }
          if(i==0) this->playground=playground;
          playground->setColor(Color(0.5,0.1,0.1,i==0 ? 0 : .99)); // inner wall invisible
          playground->setPosition(osg::Vec3(0,0,thicknessSoft)); // playground positionieren und generieren
          global.obstacles.push_back(playground);
        }

        break;
      }
    case Uterus:
      {
        // we stack two playgrounds in each other.
        // The outer one is hard (and invisible) and the inner one is soft
        int anzgrounds=2;
        // this is the utterus imitation: high slip, medium roughness, high elasticity, soft
        Substance uterus(0.2/*roughness*/, 0.1 /*slip*/,
                         .5 /*hardness*/, 0.95 /*elasticity*/);
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
                                                            i==0 ? thickness : .5, pitheight),
                                                  1, i==0);
          playground->setTexture("Images/dusty.rgb");
          if(i==0){ // set ground also to the soft substance
            playground->setGroundSubstance(uterus);
          }
          if(i==0) this->playground=playground;
          playground->setColor(Color(0.5,0.1,0.1,i==0? .2 : 0)); // outer ground is not visible (alpha=0)
          playground->setPosition(osg::Vec3(0,0,i==0? thickness : 0 )); // playground positionieren und generieren
          global.obstacles.push_back(playground);
        }

        break;
      }
    case Stacked:
      {
        int anzgrounds=2;
        for (int i=0; i< anzgrounds; i++){
          playground = new Playground(odeHandle, osgHandle, osg::Vec3(10+4*i, .2, .95+0.15*i), 1, i==(anzgrounds-1));
          //OdeHandle myhandle = odeHandle;
          //      myhandle.substance.toFoam(10);
          // playground = new Playground(myhandle, osgHandle, osg::Vec3(/*base length=*/50.5,/*wall = */.1, /*height=*/1));
          playground->setPosition(osg::Vec3(0,0,0.2)); // playground positionieren und generieren

          global.obstacles.push_back(playground);
        }
        break;
      }
    default:
      break;
    }
  }

  void placeObstacles(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      GlobalData& global){

    for(int i=0; i<numSpheres; i++){
      PassiveSphere* s =
        new PassiveSphere(odeHandle,
                          osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.2);
      s->setPosition(Pos(i*0.5-2, i*0.5, 1.0));
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);
    }

    for(int i=0; i<numBoxes; i++){
      PassiveBox* b =
        new PassiveBox(odeHandle,
                          osgHandle, osg::Vec3(0.2+i*0.1,0.2+i*0.1,0.2+i*0.1));
      b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
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

};


#endif

