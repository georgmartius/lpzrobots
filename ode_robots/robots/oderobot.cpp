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
 ***************************************************************************
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2006-08-08 17:04:46  martius
 *   added new sensor model
 *
 *   Revision 1.3  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:41  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.5  2006/06/25 16:57:14  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.4  2006/05/26 15:47:20  fhesse
 *   using Geom in getPosition() (temporary version)
 *
 *   Revision 1.1.2.3  2005/12/14 15:37:09  martius
 *   robots are working with osg
 *
 *   Revision 1.1.2.2  2005/12/13 18:11:40  martius
 *   still trying to port robots
 *
 *   Revision 1.1.2.1  2005/12/13 12:31:09  martius
 *   moved to cpp file
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/

#include "oderobot.h"
#include "primitive.h"
#include "mathutils.h"

using namespace std;

namespace lpzrobots {

  /**
   * Constructor
   * @param odehandle structure with all global ODE variables
   */
  OdeRobot::OdeRobot(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
		     const std::string& name,const std::string& revision)
    : AbstractRobot(name, revision), odeHandle(odeHandle), osgHandle(osgHandle) {
    parentspace = odeHandle.space;
  };

  OdeRobot::~OdeRobot(){}

  /** sets color of the robot
      @param col Color struct with desired Color
   */
  void OdeRobot::setColor(const Color& col){
    osgHandle.color = col;
  };

  /** sets the vehicle to position pos
      @params pos desired position of the robot
  */
  void OdeRobot::place(const Pos& pos){
    place(osg::Matrix::translate(pos));
  }

  /** returns position of the object
      @return vector of position (x,y,z)
  */
  Position OdeRobot::getPosition() const {
    const Primitive* o = getMainPrimitive();    
    //    if (o && o->getBody()){
    //      return Position(dBodyGetPosition(o->getBody()));
    //    } else {

      /*********************************
Testing
      osg::Vec3 p(o->getPosition());
     //((Transform*)o)->parent->getOSGPrimitive()->getTransform().computeLocalToWorldMatrix(p,NodeVisitor());
     // osg::MatrixTransform m(((Transform*)o)->parent->getOSGPrimitive()->getTransform(), CopyOp());

      //dummer weise ist das kein osg::Transform, sondern ein lpzrobots::Transform 
      ((Transform*)o)->getOSGPrimitive()->getTransform().computeLocalToWorldMatrix(p,NodeVisitor());;

      std::cout<<"Position1("<<p[0]<<", "<<p[1]<<", "<<p[2]<<") \n";
      Position p2(dGeomGetPosition(o->parent->getGeom()));
      Position p3(p2.x+p[0],p2.y+p[1],p2.z+p[2]);
      std::cout<<"Position3("<<p3.x<<", "<<p3.y<<", "<<p3.z<<") \n";
      return  p3;//Position(0,0,0);
    }
      ***************************************/   


    // using the Geom has maybe the advantage to get the position of transform objects 
    // (e.g. hand of muscledArm)
    if (o && o->getGeom()){
      return Position(dGeomGetPosition(o->getGeom()));
    } else return Position(0,0,0);
  }
  
  /** returns linear speed vector of the object
      @return vector  (vx,vy,vz)
  */
  Position OdeRobot::getSpeed() const {
    const Primitive* o = getMainPrimitive();
    if (o && o->getBody()){
      return Position(dBodyGetLinearVel(o->getBody()));
    } else return Position(0,0,0);
  }
  
  /** returns the orientation of the object
      @return 3x3 rotation matrix
  */
  matrix::Matrix OdeRobot::getOrientation() const {
    const Primitive* o = getMainPrimitive();
    if (o && o->getBody()){
      return odeRto3x3RotationMatrix(dBodyGetRotation(o->getBody())); 
    } else {
      matrix::Matrix R(3,3); 
      return R^0; // identity
    }
  }
  

  bool OdeRobot::isGeomInPrimitiveList(Primitive** ps, int len, dGeomID geom){  
    for(int i=0; i < len; i++){
      if(geom == ps[i]->getGeom()) return true;
    }
    return false;
  }

  bool OdeRobot::isGeomInPrimitiveList(list<Primitive*> ps, dGeomID geom){  
    for(list<Primitive*>::iterator i=ps.begin(); i != ps.end(); i++){
      if(geom == (*i)->getGeom()) return true;
    }
    return false;
  }


}
