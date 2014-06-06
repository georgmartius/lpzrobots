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

#include "schlange.h"
#include "osgprimitive.h"

using namespace std;

namespace lpzrobots {

  Schlange::Schlange ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       const SchlangeConf& conf, const std::string& name, const std::string& revision)
    : OdeRobot( odeHandle, osgHandle, name, revision), conf(conf) {

    addParameter("motorpower", &this->conf.motorPower,0,20);
    addParameter("sensorfactor", &this->conf.sensorFactor,0,5);
    addParameter("frictionjoint",&this->conf.frictionJoint,0,5);
    created=false;
  }

  Schlange::~Schlange()
  {
    if(created) destroy();
  }


  void Schlange::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
    create(pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.segmDia/2)));
  }

  void Schlange::update() {
    OdeRobot::update();
    assert(created); // robot must exist


  }


  int Schlange::getSegmentsPosition(std::vector<Position> &poslist){
    assert(created);
    for(int n = 0; n < conf.segmNumber; n++){
      Pos p(objects[n]->getPosition());
      poslist.push_back(p.toPosition());
    }
    return conf.segmNumber;
  }

  void Schlange::notifyOnChange(const paramkey& key){
    if(key == "frictionjoint") {
      for (vector<AngularMotor*>::iterator i = frictionmotors.begin(); i!= frictionmotors.end(); i++){
        if (*i) (*i)->setPower(conf.frictionJoint);
      }
    }
  }


  /** creates vehicle at desired position
  */
  void Schlange::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    odeHandle.createNewSimpleSpace(parentspace,false);
    int half = conf.segmNumber/2;
    int segperspace=5;
    if(conf.useSpaces){
      int spacenum=conf.segmNumber/segperspace+1;
      spaces.resize(spacenum);
      for(int i=0; i<spacenum; i++){
        OdeHandle o(odeHandle);
        o.createNewSimpleSpace(odeHandle.space,true);
        spaces[i]=o;
      }
    }

    if(conf.frictionRatio != 1)
      odeHandle.substance.toAnisotropFriction(conf.frictionRatio, Axis(0,0,1));

    for ( int n = 0; n < conf.segmNumber; n++ ) {
      Primitive* p;
      if(conf.useSpaces)
        p = createSegment(n, spaces[n/segperspace]);
      else
        p = createSegment(n, odeHandle);
      p->setPose(p->getPose() * osg::Matrix::rotate(M_PI/2, 0, 1, 0)
                 * osg::Matrix::translate((n-half)*conf.segmLength, 0 , conf.segmDia/2) * pose);

      objects.push_back(p);

    }

//       if (n==-1* conf.segmNumber/2) {
//                 p = new Box(conf.segmLength*1.8,conf.segmLength*.8, conf.segmLength*1);
//                 //p = new Capsule(conf.segmDia*2 , conf.segmLength);
//         p->init(odeHandle, conf.segmMass*2, osgHandle);
//       }
//       //    else {
//       //if(n==0 || n== conf.segmNumber){

//       //  p = new Box(conf.segmLength,conf.segmLength*2, conf.segmLength);
//       //  p->init(odeHandle, conf.segmMass*2, osgHandle);
//       //        }
//       else{


//         if(n==-1/*== 0 | n== conf.segmNumber-1*/) {
//           p = new Capsule(conf.segmDia*.8/*2.8*/ , conf.segmLength*1);
//         // p = new Box(conf.segmLength*.3,conf.segmLength, conf.segmLength*.9);
//         p->init(odeHandle, conf.segmMass*4, osgHandle);}
//         else{
//  p = new Capsule(conf.segmDia*.8 , conf.segmLength);
//  //        p = new Box(conf.segmLength*.3,conf.segmLength*0.3, conf.segmLength*1.0);
//  p->init(odeHandle, conf.segmMass, osgHandle);
//         } }
//         //        else {

//       //      p->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) *
//       //                 osg::Matrix::translate((n-half)*conf.segmLength*(1+((double)n)/10), 0 , conf.segmDia/2) *
//       //                 pose);
//       p->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) *
//              //  p->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) *
//                  osg::Matrix::translate((n-half)*conf.segmLength, 0 , conf.segmDia/2) *
//                  pose);
//       //      p->getOSGPrimitive()->setTexture("Images/wood.rgb");
//       //  p->getOSGPrimitive()->setTexture("Images/tire.rgb");
//       p->getOSGPrimitive()->setTexture("Images/whitemetal_farbig.rgb");
//       //      p->getOSGPrimitive()->setColor(Color(0.0f,0.0f,1.0f,0.2f));

//       objects.push_back(p);
//         }

    created=true;
  };

  Primitive* Schlange::createSegment(int index, const OdeHandle& odeHandle){
    Primitive* p;
    p = new Capsule(conf.segmDia * 0.8, conf.segmLength);
    // if (index==0)
    //   p = new Box(conf.segmLength*.1,conf.segmLength*.9, conf.segmLength*.6);
    p->setTexture("Images/whitemetal_farbig_small.rgb");
    p->init(odeHandle, conf.segmMass, osgHandle);
    if(index==0)
      p->setColor(conf.headColor);
    else
      p->setColor(conf.bodyColor);
    return p;
  }

  void Schlange::setTexture(const std::string& filename){
    if(created) {
      // go through all objects (primitives)
      for(int n = 0; n < conf.segmNumber; n++){
        objects[n]->setTexture(filename);
      }
    }
  }


  void Schlange::setHeadTexture(const std::string& filename){
    if(created) {
      objects[0]->getOSGPrimitive()->setTexture(filename);
    }
  }

  void Schlange::setHeadColor(const Color& color) {
    if (created) {
      objects[0]->getOSGPrimitive()->setColor(color);
    }
  }



  /** destroys vehicle and space    */

  void Schlange::destroy(){
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



