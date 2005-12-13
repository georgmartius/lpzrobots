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
 *   Revision 1.1.2.2  2005-12-13 18:11:40  martius
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

namespace lpzrobots {

  /**
   * Constructor
   * @param odehandle structure with all global ODE variables
   */
  OdeRobot::OdeRobot(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const char* name)
    : AbstractRobot(name), odeHandle(odeHandle), osgHandle(osgHandle) {
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
    if (o && o->getBody()){
      return Position(dBodyGetPosition(o->getBody()));
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

  matrix::Matrix OdeRobot::odeRto3x3RotationMatrixT ( const double R[12] ) {  
    matrix::Matrix matrix(3,3);
    matrix.val(0,0)=R[0];
    matrix.val(0,1)=R[4];
    matrix.val(0,2)=R[8];
    matrix.val(1,0)=R[1];
    matrix.val(1,1)=R[5];
    matrix.val(1,2)=R[9];
    matrix.val(2,0)=R[2];
    matrix.val(2,1)=R[6];
    matrix.val(2,2)=R[10];
    return matrix;
  }

  matrix::Matrix OdeRobot::odeRto3x3RotationMatrix ( const double R[12] ) {  
    matrix::Matrix matrix(3,3);
    matrix.val(0,0)=R[0];
    matrix.val(1,0)=R[4];
    matrix.val(2,0)=R[8];
    matrix.val(0,1)=R[1];
    matrix.val(1,1)=R[5];
    matrix.val(2,1)=R[9];
    matrix.val(0,2)=R[2];
    matrix.val(1,2)=R[6];
    matrix.val(2,2)=R[10];
    return matrix;
  }

}
