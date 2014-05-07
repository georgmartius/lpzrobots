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

#include "defaultCaterpillar.h"
#include "osgprimitive.h"

using namespace std;

namespace lpzrobots {

  DefaultCaterPillar::DefaultCaterPillar ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                                           const CaterPillarConf& conf,
                                           const std::string& name, const std::string& revision)
    : OdeRobot( odeHandle, osgHandle, name, revision), conf(conf) {
    addParameter("frictionground",&this->conf.frictionGround,0,2);
    addParameter("motorpower", &this->conf.motorPower,0,20);
    addParameter("sensorfactor", &this->conf.sensorFactor,0,5);
    addParameter("frictionjoint",&this->conf.frictionJoint,0,5);

    created=false;
  }

  DefaultCaterPillar::~DefaultCaterPillar()
  {
    if(created) destroy();
  }


  void DefaultCaterPillar::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
    create(pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.segmDia/2)));
  }

  void DefaultCaterPillar::update() {
    OdeRobot::update();
    assert(created); // robot must exist


  }


  void DefaultCaterPillar::notifyOnChange(const paramkey& key){
    if(key == "frictionjoint") {
      for (vector<AngularMotor*>::iterator i = frictionmotors.begin(); i!= frictionmotors.end(); i++){
        if (*i) (*i)->setPower(conf.frictionJoint);
      }
    }
  }


  int DefaultCaterPillar::getSegmentsPosition(std::vector<Position> &poslist){
    assert(created);
    for(int n = 0; n < conf.segmNumber; n++){
      Pos p(objects[n]->getPosition());
      poslist.push_back(p.toPosition());
    }
    return conf.segmNumber;
  }



  /** creates vehicle at desired position
  */
  void DefaultCaterPillar::create(const osg::Matrix& pose) {
    if (created) {
      destroy();
    }

    odeHandle.createNewSimpleSpace(parentspace,false);

    int half = conf.segmNumber/2;

    // linear positioning (snake-like)
    for(int n = 0; n < conf.segmNumber; n++) {
      Primitive* p = new Box(conf.segmDia/2, conf.segmDia*2, conf.segmLength);
      p->setTexture("Images/dusty.rgb");
      p->init(odeHandle, conf.segmMass, osgHandle);
      p->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) *
                 osg::Matrix::translate((n-half)*conf.segmLength*0.7, 0, conf.segmDia/2) * // made boxes overlapping for not seeing any gaps (*0.7)
                 pose);
      objects.push_back(p);
    }

    created=true;
  }

  /** destroys vehicle and space
   */
  void DefaultCaterPillar::destroy(){
    if (created){
      for (vector<AngularMotor*>::iterator i = frictionmotors.begin(); i!= frictionmotors.end(); i++){
        if(*i) delete *i;
      }
      frictionmotors.clear();
      cleanup();
      odeHandle.deleteSpace();
    }
    created=false;
  }




//   /** fix segment 0 in the sky
//    */
//   void Schlange::fixInSky(){
//     for (int i=0; i<2; i++){
//       skyJoints.push_back( dJointCreateHinge ( world , 0 ) );
//       dJointAttach ( skyJoints.back(), objektliste[0].body , 0 );
//       dJointSetUniversalAnchor ( skyJoints.back(),
//                                  dBodyGetPositionAll ( objektliste[0].body , 1 ) ,
//                                  dBodyGetPositionAll ( objektliste[0].body , 2 ) ,
//                                  dBodyGetPositionAll ( objektliste[0].body , 3 ) );
//       if (i==0) dJointSetHingeAxis(skyJoints.back(),1,0,0);
//       if (i==1) dJointSetHingeAxis(skyJoints.back(),0,1,0);
//       dJointSetFixed(skyJoints.back());
//     }
//     /*
//       jointliste.push_back( dJointCreateHinge ( world , 0 ) );
//       dJointAttach ( jointliste.back() , objektliste[0].body , 0 );
//       dJointSetUniversalAnchor ( jointliste.back() ,
//       dBodyGetPositionAll ( objektliste[0].body , 1 ) ,
//       dBodyGetPositionAll ( objektliste[0].body , 2 ) ,
//       dBodyGetPositionAll ( objektliste[0].body , 3 ) );
//       dJointSetHingeAxis(jointliste.back(),0,1,0);
//       dJointSetFixed(jointliste.back());
//     */
//   };


}



