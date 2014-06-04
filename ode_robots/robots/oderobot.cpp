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

#include "oderobot.h"
#include "joint.h"
#include "primitive.h"
#include "mathutils.h"
#include "torquesensor.h"

using namespace std;

namespace lpzrobots {

  /**
   * Constructor
   * @param odehandle structure with all global ODE variables
   */
  OdeRobot::OdeRobot(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                     const std::string& name,const std::string& revision)
    : AbstractRobot(name, revision),
      initialPose(osg::Matrix::identity()),
      initialRelativePose(osg::Matrix::identity()),
      odeHandle(odeHandle), osgHandle(osgHandle),
      initialized(false), askedfornumber(false) {
    parentspace = odeHandle.space;
    fixationTmpJoint = 0;
  };

  OdeRobot::~OdeRobot(){
    cleanup();
  }

  void OdeRobot::placeIntern(const osg::Matrix& pose){
    assert("should never be called"); // just to satisfy linker (really weird linker error)
  };


  int  OdeRobot::getSensors(sensor* sensors_, int sensornumber) {
    assert(initialized);
    int len = 0;
    len += getSensorsIntern(sensors_,sensornumber);
    for(auto& i: sensors){
      len += i.first->get(sensors_ + len, sensornumber - len);
    }
    return len;
  }

  void OdeRobot::setMotors(const double* motors_, int motornumber) {
    assert(initialized);
    int len = 0;
    len = getMotorNumberIntern();
    setMotorsIntern(motors_, len);
    for(auto& i: motors){
      len += i.first->set(motors_ + len, motornumber - len);
    }
  }

  int  OdeRobot::getSensorNumber() {
    assert(initialized);
    int s=0;
    for ( auto &i: sensors){
      s += i.first->getSensorNumber();
    }
    askedfornumber=true;
    return getSensorNumberIntern()  + s;
  }

  int  OdeRobot::getMotorNumber() {
    assert(initialized);
    int s=0;
    for(auto& i: motors){
      s += i.first->getMotorNumber();
    }
    askedfornumber=true;
    return getMotorNumberIntern() + s;
  }

  list<SensorMotorInfo> OdeRobot::getSensorInfos(){
    list<SensorMotorInfo> l;
    int num = getSensorNumberIntern();
    for(int k=0; k<num; k++){ l += SensorMotorInfo(); }
    for ( auto &i: sensors){
      l += i.first->getSensorInfos();
    }
    return l;
  }

  list<SensorMotorInfo> OdeRobot::getMotorInfos(){
    list<SensorMotorInfo> l;
    int num = getMotorNumberIntern();
    for(int k=0; k<num; k++){ l += SensorMotorInfo(); }
    for ( auto &i: motors){
      l += i.first->getMotorInfos();
    }
    return l;
  }


  void OdeRobot::attachSensor(SensorAttachment& sa){
    Primitive* p;
    if(sa.second.primitiveIndex<0){
      p = getMainPrimitive();
    } else {
      assert((int)getAllPrimitives().size() > sa.second.primitiveIndex);
      p = getAllPrimitives()[sa.second.primitiveIndex];
    }
    Joint* j=0;
    if(sa.second.jointIndex>=0){
      assert((int)getAllJoints().size() > sa.second.jointIndex);
      j = getAllJoints()[sa.second.jointIndex];
    }
    sa.first->init(p, j);
  }

  void OdeRobot::attachMotor(MotorAttachment& ma){
    Primitive* p;
    if(ma.second.primitiveIndex<0){
      p = getMainPrimitive();
    } else {
      assert((int)getAllPrimitives().size() > ma.second.primitiveIndex);
      p = getAllPrimitives()[ma.second.primitiveIndex];
    }
    Joint* j=0;
    if(ma.second.jointIndex>=0){
      assert((int)getAllJoints().size() > ma.second.jointIndex);
      j = getAllJoints()[ma.second.jointIndex];
    }
    ma.first->init(p, j);
  }

  void OdeRobot::addSensor(std::shared_ptr<Sensor> sensor, Attachment attachment){
    assert(!askedfornumber);
    assert(sensor);
    SensorAttachment sa(std::shared_ptr<Sensor>(sensor),attachment);
    if(initialized){
      attachSensor(sa);
    }
    sensors.push_back(sa);
  }

  void OdeRobot::addMotor(std::shared_ptr<Motor> motor, Attachment attachment){
    assert(!askedfornumber);
    assert(motor);
    MotorAttachment ma(std::shared_ptr<Motor>(motor),attachment);
    if(initialized){
      attachMotor(ma);
    }
    motors.push_back(ma);
  }

  void OdeRobot::addTorqueSensors(double maxtorque, int avg ){
    if(!initialized){
      cerr << "call addTorqueSensors() after place()!";
      assert(initialized);
    }
    int numJoints = getAllJoints().size();
    for(int j=0; j<numJoints; j++){
      addSensor(std::make_shared<TorqueSensor>(maxtorque, avg), Attachment(-1,j));
    }
  }




  void OdeRobot::cleanup(){
    sensors.clear();
    motors.clear(); // this also deletes the pointers
    FOREACH(std::vector<Joint*>, joints, j){
      if(*j) delete *j;
    }
    joints.clear();
    FOREACH(std::vector<Primitive*>, objects, o){
      if(*o) delete *o;
    }
    objects.clear();
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
  void OdeRobot::place(const Pos& pos) {
    place(osg::Matrix::translate(pos));
  }

  void OdeRobot::place(const osg::Matrix& pose) {
    placeIntern(pose);
    for( auto &i: sensors){
      attachSensor(i);
    }
    for( auto &i: motors){
      attachMotor(i);
    }
    assert(getMainPrimitive());
    initialPose=getMainPrimitive()->getPose();
    Pose inv;
    inv.invert(pose);
    initialRelativePose=getMainPrimitive()->getPose() * inv;
    initialized=true;
  }

  void OdeRobot::sense(GlobalData& globalData){
    for(auto& i: sensors){
      i.first->sense(globalData);
    }
  }

  void OdeRobot::doInternalStuff(GlobalData& globalData){
    for(auto &i: motors){
      i.first->act(globalData);
    }
  }

  void OdeRobot::update(){
    for(auto& i : objects){
      if(i) i->update();
    }
    for(auto& i : joints){
      if(i) i->update();
    }
    for(auto& i: sensors){
      i.first->update();
    }
  }


  bool OdeRobot::isGeomInPrimitiveList(Primitive** ps, int len, dGeomID geom){
    for(int i=0; i < len; i++){
      if(geom == ps[i]->getGeom()) return true;
    }
    return false;
  }

  bool OdeRobot::isGeomInPrimitiveList(std::list<Primitive*> ps, dGeomID geom){
    for( auto& i : ps){
      if(geom == i->getGeom()) return true;
    }
    return false;
  }

  void OdeRobot::moveToPosition(Pos pos, int primitiveID){
    const vector<Primitive*>& ps = this->getAllPrimitives();
    Pos robpos; // reference position of robot
    if(primitiveID==-1){
      if(!getMainPrimitive()) return;
      robpos = getMainPrimitive()->getPosition();
    }else if(primitiveID==-2){
        double min=10e8;
        // find lowest body part and its position
        FOREACHC(vector<Primitive*>, ps, p){
          double z = (*p)->getPosition().z();
          if(z<min){
            z=min;
            robpos=(*p)->getPosition();
          }
        }
    }else if(primitiveID>=0 && primitiveID < (signed)ps.size()){
        if(!ps[primitiveID]) return;
        robpos=ps[primitiveID]->getPosition();
    }else return;
    // move robot
    FOREACHC(vector<Primitive*>, ps, p){
      Pos local = (*p)->getPosition() - robpos; // relative local position of that primitive
      (*p)->setPosition(pos+local);
      dBodySetLinearVel((*p)->getBody(), 0, 0, 0);
      dBodySetAngularVel((*p)->getBody(),0, 0, 0);
    }
  }

  void OdeRobot::moveToPose(Pose pose, int primitiveID){
    const vector<Primitive*>& ps = this->getAllPrimitives();
    Primitive* ref;
    Pose robpose; // reference pose of primitive
    if(primitiveID==-1){
      if(!getMainPrimitive()) return;
      ref = getMainPrimitive();
    }else if(primitiveID>=0 && primitiveID < (signed)ps.size()){
        if(!ps[primitiveID]) return;
        ref = ps[primitiveID];
    }else {
      fprintf(stderr,"primitive index out of bounds %i (of %lui)", primitiveID, ps.size());
      return;
    }
    // move robot
    // calc transformation on robot
    Pose refInvertPose;
    refInvertPose.invert(ref->getPose());
    Pose transformation = refInvertPose*pose;
    FOREACHC(vector<Primitive*>, ps, p){
      (*p)->setPose((*p)->getPose()*transformation);
      dBodySetLinearVel((*p)->getBody(), 0, 0, 0);
      dBodySetAngularVel((*p)->getBody(),0, 0, 0);
    }
  }


  void OdeRobot::fixate(GlobalData& global, int primitiveID, double duration){
    Primitive* p=0; // primitive to fix
    if(primitiveID==-1){
      p = getMainPrimitive();
    }else{
      const vector<Primitive*>& ps = this->getAllPrimitives();
      if(primitiveID>=0 && primitiveID < (signed)ps.size()){
        p = ps[primitiveID];
      }
    }
    if(p){
      unFixate(global); // unfixate in case we are already fixated
      fixationTmpJoint = new TmpJoint(new FixedJoint(p, global.environment), "joint");
      if(duration<=0) duration=1e12; // some large number will do
      global.addTmpObject(fixationTmpJoint, duration);
    }
  }

  bool OdeRobot::unFixate(GlobalData& global){
    bool fixed=false;
    if(fixationTmpJoint){
      fixed=global.removeTmpObject(fixationTmpJoint);
      fixationTmpJoint=0;
    }
    return fixed;
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
    // cout << ps.size() << endl;
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
