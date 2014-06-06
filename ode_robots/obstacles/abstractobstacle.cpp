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
#include "abstractobstacle.h"
#include "osgprimitive.h"
#include "primitive.h"
#include <selforg/stl_adds.h>
#include <selforg/position.h>

#include "mathutils.h"
#include "pos.h"

using namespace std;

namespace lpzrobots {

  /*
   * Constructor
   * @param odeHandle containing ODE stuff like world, space and jointgroup
   * @param osgHandle containing OSG stuff like scene, color...
   * be used for creation of obstacles
   */
  AbstractObstacle::AbstractObstacle(const OdeHandle& odeHandle, const OsgHandle& osgHandle)
    : pose(osg::Matrix::translate(0,0,0)), odeHandle(odeHandle), osgHandle(osgHandle)
  {
    // initialize the pose matrix correctly
    obstacle_exists=false;
  };

  AbstractObstacle::~AbstractObstacle(){
    if(obstacle_exists) AbstractObstacle::destroy();
  }

  void AbstractObstacle::update(){
    if (obstacle_exists){
      for (unsigned int i=0; i<obst.size(); i++){
        if(obst[i]) obst[i]->update();
      }
    }
  };

  /*
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  void AbstractObstacle::setPos(const osg::Vec3& pos) {
    pose.setTrans(pos);
    setPose(pose);
  };

  /*
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  void AbstractObstacle::setPosition(const osg::Vec3& pos) {
    setPos(pos);
  };

  /*
   * gives actual position of the obstacle
   */
  osg::Vec3 AbstractObstacle::getPos(){
    return pose.getTrans();
  }

  /*
   * gives actual pose of the obstacle
   */
  osg::Matrix AbstractObstacle::getPose(){ return pose; }

  void AbstractObstacle::setColor(const Color& color) {
    osgHandle.color = color;
    if (obstacle_exists) {
      FOREACH(vector<Primitive*>, obst, it){
        (*it)->setColor(color);
      }
    }
  }

  void AbstractObstacle::setColor(const string& color) {
    osgHandle.color = osgHandle.getColor(color);
    if (obstacle_exists) {
      FOREACH(vector<Primitive*>, obst, it){
        (*it)->setColor(color);
      }
    }
  }

  void AbstractObstacle::setTexture(const std::string& texturefilename){
    setTexture(0,TextureDescr(texturefilename,-1,-1));
  }

  void AbstractObstacle::setTexture(const TextureDescr& texture){
    setTexture(0,texture);
  }

  void AbstractObstacle::setTexture(int surface, const TextureDescr& texture){
    if(obstacle_exists){
      FOREACH( std::vector<Primitive*>, obst, o){
        if(*o) (*o)->setTexture(surface,texture);
      }
    }else{
      setTexture(0,surface,texture);
    }
  }

  void AbstractObstacle::setTexture(int primitive, int surface, const TextureDescr& texture){
    if(obstacle_exists){
      if(primitive < (signed)textures.size())
        obst[primitive]->setTexture(surface,texture);
    }else{
      if(primitive >= (signed)textures.size()) textures.resize(primitive+1);
      if(surface >= (signed)textures[primitive].size()) textures[primitive].resize(surface+1);
      textures[primitive][surface]=texture;
    }
  }


  TextureDescr AbstractObstacle::getTexture(int primitive, int surface) const{
    // take the last primitive we have texture information for.
    if(primitive >= (signed)textures.size())
      primitive = textures.size()-1;
    // take the last surface we have texture information for.
    if(surface >= (signed)textures[primitive].size()) surface = textures[primitive].size()-1;
    if(primitive<0 || surface<0) return TextureDescr();
    return textures[primitive][surface];
  }

  std::vector<TextureDescr> AbstractObstacle::getTextures(int primitive) const{
    // take the last primitive we have texture information for.
    if(primitive >= (signed)textures.size())
      primitive = textures.size()-1;
    // take the last surface we have texture information for.
    if(primitive<0) return std::vector<TextureDescr>();
    return textures[primitive];
  }


  void AbstractObstacle::setSubstance(const Substance& substance){
    odeHandle.substance = substance;
    if (obstacle_exists) {
      FOREACH(vector<Primitive*>, obst, it){
        (*it)->substance=substance;
      }
    }
  }

  const Substance& AbstractObstacle::getSubstance(){
    return odeHandle.substance;
  }


  void AbstractObstacle::destroy(){
    FOREACH(vector<Primitive*>, obst, it){
      if(*it) delete(*it);
    }
    obst.clear();
    obstacle_exists=false;
  };

/** returns position of the object
@return vector of position (x,y,z)
  */
Position AbstractObstacle::getPosition() const {
  const Primitive* o = getMainPrimitive();

    // using the Geom has maybe the advantage to get the position of transform objects
    // (e.g. hand of muscledArm)
  if (o && o->getGeom())
    return Position(dGeomGetPosition(o->getGeom()));
  else if(o->getBody())
    return Position(dBodyGetPosition(o->getBody()));
  else return Position(0,0,0);
}

 Position AbstractObstacle::getSpeed() const {
  const Primitive* o = getMainPrimitive();
  if (o && o->getBody())
    return Position(dBodyGetLinearVel(o->getBody()));
  else return Position(0,0,0);
}

Position AbstractObstacle::getAngularSpeed() const {
  const Primitive* o = getMainPrimitive();
  if (o && o->getBody())
    return Position(dBodyGetAngularVel(o->getBody()));
  else return Position(0,0,0);
}

matrix::Matrix AbstractObstacle::getOrientation() const {
  const Primitive* o = getMainPrimitive();
  if (o && o->getBody()){
    return odeRto3x3RotationMatrix(dBodyGetRotation(o->getBody()));
  } else {
    matrix::Matrix R(3,3);
    return R^0; // identity
  }
}


}
