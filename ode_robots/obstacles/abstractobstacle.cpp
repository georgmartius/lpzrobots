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
 *                                                                         *
 *   $Log$
 *   Revision 1.9  2011-01-02 23:09:52  martius
 *   texture handling of boxes changed
 *   playground walls changed
 *
 *   Revision 1.8  2010/10/21 12:58:57  martius
 *   new member getsubstance
 *
 *   Revision 1.7  2009/04/02 10:12:25  martius
 *   Texture handling changed
 *
 *   Revision 1.6  2007/12/06 10:02:49  der
 *   abstractground: returns now cornerpoints
 *   abstractobstacle: is now trackable
 *   hudstatistics: supports now AbstractmMeasure
 *
 *   Revision 1.5  2007/08/28 09:24:36  martius
 *   nonvirtual call in destructor, and FOREACH loop
 *
 *   Revision 1.4  2007/03/16 11:01:37  martius
 *   abstractobstacle gets mor functionallity
 *   setSubstance
 *
 *   Revision 1.3  2006/12/13 09:17:16  martius
 *   obstacle_exists should be set in create if childs
 *
 *   Revision 1.2  2006/07/14 12:23:32  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2006/06/29 16:43:20  robot3
 *   abstract classes have now own .cpp-files
 *
 *   Revision 1.8.4.8  2006/06/16 22:27:26  martius
 *   getMainPrimtive
 *
 *   Revision 1.8.4.7  2006/05/23 13:38:02  robot3
 *   -fixed some creating bugs
 *   -setColor,setTexture and createGround must be
 *    called before setPosition now
 *
 *   Revision 1.8.4.6  2006/05/18 12:54:24  robot3
 *   -fixed not being able to change the color after positioning
 *    the obstacle
 *   -cleared the files up
 *
 *   Revision 1.8.4.5  2006/05/11 08:59:15  robot3
 *   -fixed a positioning bug (e.g. for passivesphere)
 *   -some methods moved to abstractobstacle.h for avoiding inconsistencies
 *
 *   Revision 1.8.4.4  2006/03/30 12:34:51  martius
 *   documentation updated
 *
 *   Revision 1.8.4.3  2006/03/29 15:04:38  martius
 *   have pose now
 *
 *   Revision 1.8.4.2  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.8.4.1  2005/11/14 17:37:14  martius
 *   moved to selforg
 *
 *   Revision 1.8  2005/10/25 19:26:56  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.7  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.6  2005/09/13 13:20:12  martius
 *   initialised color
 *
 *   Revision 1.5  2005/08/29 06:32:25  martius
 *   added virtual destructor
 *
 *   Revision 1.4  2005/07/18 14:52:33  martius
 *   world and space are not pointers anymore.
 *
 *   Revision 1.3  2005/06/15 14:22:11  martius
 *   GPL included
 *                                                                 *
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
    : odeHandle(odeHandle), osgHandle(osgHandle) {
    // initialize the pose matrix correctly
    pose=osg::Matrix::translate(0,0,0);
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


  /*
   * sets the obstacle color
   * @param color values in RGBA
   */
  void AbstractObstacle::setColor(const Color& color) {
    osgHandle.color = color;
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
