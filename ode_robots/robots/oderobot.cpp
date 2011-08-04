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
 *   Revision 1.14  2011-08-04 16:43:53  martius
 *   guilogger is positioned beside simulation window (can still be improved)
 *   ctrl-h can be used to move observed agent to 0,0,0 position
 *
 *   Revision 1.13  2011/06/09 16:01:05  martius
 *   add help for o/O
 *   soxexpand: getter and setter
 *
 *   Revision 1.12  2011/06/03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.11  2011/06/01 22:02:56  martius
 *   getAllPrimitives changed to vector return type
 *   inspectables infolines are printed without name again (for guilogger)
 *
 *   Revision 1.10  2010/11/05 13:54:05  martius
 *   store and restore for robots implemented
 *
 *   Revision 1.9  2010/01/26 09:54:56  martius
 *   getposition and getVel... is done via the primitives
 *
 *   Revision 1.8  2008/09/16 14:53:40  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.7  2008/05/07 16:45:52  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.6  2007/12/06 10:02:49  der
 *   abstractground: returns now cornerpoints
 *   abstractobstacle: is now trackable
 *   hudstatistics: supports now AbstractmMeasure
 *
 *   Revision 1.5  2007/04/05 15:11:42  martius
 *   angular speed tracking
 *
 *   Revision 1.4  2006/08/08 17:04:46  martius
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
#include "joint.h"
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

  OdeRobot::~OdeRobot(){
    cleanup();
  }

  void OdeRobot::cleanup(){
    FOREACH(std::vector<Primitive*>, objects, o){
      if(*o) delete *o;
    }
    objects.clear();
    FOREACH(std::vector<Joint*>, joints, j){
      if(*j) delete *j;
    }
    joints.clear();
  }


  /** sets color of the robot
      @param col Color struct with desired Color
   */
  void OdeRobot::setColor(const Color& col){
    osgHandle.color = col;
  };

  /** sets the vehicle to position pos
      @param pos desired position of the robot
  */
  void OdeRobot::place(const Pos& pos){
    place(osg::Matrix::translate(pos));
  }
  

  bool OdeRobot::isGeomInPrimitiveList(Primitive** ps, int len, dGeomID geom){  
    for(int i=0; i < len; i++){
      if(geom == ps[i]->getGeom()) return true;
    }
    return false;
  }

  bool OdeRobot::isGeomInPrimitiveList(std::list<Primitive*> ps, dGeomID geom){  
    for(list<Primitive*>::iterator i=ps.begin(); i != ps.end(); i++){
      if(geom == (*i)->getGeom()) return true;
    }
    return false;
  }

  void OdeRobot::moveToPosition(Pos pos){
    const vector<Primitive*>& ps = this->getAllPrimitives();
    double min=10e8;
    Pos robpos; // reference position of robot (we use the lowest body part)
    // find lowest body part and its position
    FOREACHC(vector<Primitive*>, ps, p){
      double z = (*p)->getPosition().z();
      if(z<min){
        z=min;
        robpos=(*p)->getPosition();
      }
    }
    // move robot
    FOREACHC(vector<Primitive*>, ps, p){
      Pos local = (*p)->getPosition() - robpos; // relative local position of that primitive
      (*p)->setPosition(pos+local);
    }
  }


  /*********** BEGIN TRACKABLE INTERFACE ****************/
  
  /** returns position of the object
@return vector of position (x,y,z)
  */
Position OdeRobot::getPosition() const {
  const Primitive* o = getMainPrimitive();    
    // using the Geom has maybe the advantage to get the position of transform objects 
    // (e.g. hand of muscledArm)
  if (o){
    return o->getPosition().toPosition();
  }else   
    return Position(0,0,0);
}

Position OdeRobot::getSpeed() const {
  const Primitive* o = getMainPrimitive();
  if(o)
    return o->getVel().toPosition();
  else 
    return Position(0,0,0);
}

Position OdeRobot::getAngularSpeed() const {
  const Primitive* o = getMainPrimitive();
  if (o)    
    return o->getAngularVel().toPosition();
  else 
    return Position(0,0,0);
}

matrix::Matrix OdeRobot::getOrientation() const {
  const Primitive* o = getMainPrimitive();
  if (o && o->getBody()){
    return odeRto3x3RotationMatrix(dBodyGetRotation(o->getBody())); 
  } else {
    matrix::Matrix R(3,3); 
    return R^0; // identity
  }
}
  


  /*********** END TRACKABLE INTERFACE ****************/


  bool OdeRobot::store(FILE* f) const{
    /* we do here a typecase to OdeRobot* get rid of the const, 
       but the primitives are not changed anyway, so it is okay */
    if(!f) return false;
    fwrite("ROBOT", sizeof(char), 5, f); // maybe also print name
    const vector<Primitive*>& ps = this->getAllPrimitives();
    cout << ps.size() << endl;
    FOREACHC(vector<Primitive*>,ps,p){
      if(*p)
        if(!(*p)->store(f)) return false;      
    }
    return true;
  }
  
  bool OdeRobot::restore(FILE* f){
    if(!f) return false;
    char robotstring[5];
    if(fread(robotstring, sizeof(char),5,f) != 5 || strncmp(robotstring,"ROBOT",5)!=0){
      fprintf(stderr,"OdeRobot::restore: the file is not a robot save file\n");
      return false;
    }
    const vector<Primitive*>& ps = getAllPrimitives();
    FOREACHC(vector<Primitive*>,ps,p){
      if(*p)
        if(!(*p)->restore(f)) return false;      
    }
    return true;
  }

}
