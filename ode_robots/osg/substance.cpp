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

#include "substance.h"
#include "globaldata.h"
#include "axis.h"
#include "primitive.h"
#include <osg/Matrix>
#include <iostream>
#include <assert.h>
using namespace std;

namespace lpzrobots {

  Substance::Substance()
    : callback(0), userdata(0)
  {
    toDefaultSubstance();
  }

  Substance::Substance( float roughness, float slip, float hardness, float elasticity)
    : roughness(roughness), slip(slip), hardness(hardness), elasticity(elasticity),
      callback(0), userdata(0)
  {
  }

  void Substance::setCollisionCallback(CollisionCallback func, void* userdata_){
    callback = func;
    userdata=userdata_;
  }

  // Combination of two surfaces
  void Substance::getSurfaceParams(dSurfaceParameters& sp, const Substance& s1, const Substance& s2, double stepsize){
    sp.mu = s1.roughness * s2.roughness;
    //    cout << "r1= " << s1.roughness << ", r2 = " << s2.roughness << ", rges= " << sp.mu << std::endl;
    //    sp.bounce;
    //    sp.bounce_vel;
    dReal kp   = 100*s1.hardness* s2.hardness / (s1.hardness + s2.hardness);
    double kd1 = (1.00-s1.elasticity);
    double kd2 = (1.00-s2.elasticity);
    dReal kd   = 50*(kd1 * s2.hardness + kd2 * s1.hardness) / (s1.hardness + s2.hardness);
//     kp=30;
//     kd=1;
    //     cout << "spring: " << kp << "\t "<<kd << "\t step: " << stepsize << endl;
    sp.soft_erp = stepsize*kp / ( stepsize*kp + kd);
    sp.soft_cfm =  1 / (stepsize*kp + kd);
//     if(sp.soft_cfm>0.1) {
//       fprintf(stderr,"CFM on collision to large!\n");
//     }
//     if(sp.soft_erp<0.1) {
//       fprintf(stderr,"ERP on collision to small!\n");
//     }
//     cout << "ERP: " << sp.soft_erp << "\t CFM:  "<<  sp.soft_cfm << endl;
    //    sp.motion1,motion2;

    sp.slip1=s1.slip + s2.slip;
    sp.slip2=s1.slip + s2.slip;
    //cout << "s1= " << s1.slip << ", s2 = " << s2.slip << ", sges= " << sp.slip1 << std::endl;
    if(sp.slip1<0.0001) sp.mode=0;
    else sp.mode = dContactSlip1 | dContactSlip2;
    sp.mode |= dContactSoftERP | dContactSoftCFM | dContactApprox1;

  }

  void Substance::printSurfaceParams(const dSurfaceParameters& sp){
    cout << "Surface:" << sp.mu << "\t ";
    cout << sp.soft_erp << "\t ";
    cout << sp.soft_cfm << "\t ";
    cout << sp.slip1 << "\n ";
  }


  // Factory methods
  Substance Substance::getDefaultSubstance(){
    Substance s;
    s.toDefaultSubstance();
    return s;
  }

  void Substance::toDefaultSubstance(){
    toPlastic(0.8);
  }

  // very hard and elastic  with slip
  Substance Substance::getMetal(float _roughness){
    Substance s;
    s.toMetal(_roughness);
    return s;
  }
  // very hard and elastic with slip roughness [0.1-1]
  void Substance::toMetal(float _roughness){
    if(_roughness<0) { cerr << "negative roughness in metal!" << endl;}
    if(_roughness>2) { cerr << "very rough metal used!" << endl;}
    roughness = _roughness;
    hardness   = 200;
    elasticity = 0.8;
    slip       = 0.01;
  }

  // high roughness, no slip, very elastic, hardness : [5-50]
  Substance Substance::getRubber(float _hardness){
    Substance s;
    s.toRubber(_hardness);
    return s;
  }

  // high roughness, no slip, very elastic, hardness : [5-50]
  void Substance::toRubber(float _hardness){
    if(_hardness<5) { cerr << "to soft rubber!" << endl;}
    if(_hardness>50) { cerr << "to hard rubber!" << endl;}
    roughness  = 3;
    hardness   = _hardness;
    elasticity = 0.95;
    slip       = 0.0;
  }

  // medium slip, a bit elastic, medium hardness, roughness [0.5-1]
  Substance Substance::getPlastic(float _roughness){
    Substance s;
    s.toPlastic(_roughness);
    return s;

  }

  // medium slip, a bit elastic, medium hardness, roughness [0.5-2]
  void Substance::toPlastic(float _roughness){
    if(_roughness<0) { cerr << "negative roughness in plastic!" << endl;}
    if(_roughness>3) { cerr << "very rough plastic used!" << endl;}
    roughness  = _roughness;
    hardness   = 40;
    elasticity = 0.5;
    slip       = 0.01;
  }

  // large slip, not elastic, low hardness [1-30], high roughness
  Substance Substance::getFoam(float _hardness) {
    Substance s;
    s.toFoam(_hardness);
    return s;

  }

  // large slip, not elastic, low hardness [1-30], high roughness
  void Substance::toFoam(float _hardness){
    if(_hardness<1) { cerr << "to soft foam!" << endl;}
    if(_hardness>30) { cerr << "to hard foam!" << endl;}
    roughness  = 2;
    hardness   = _hardness;
    elasticity = 0;
    slip       = 0.1;
  }

  // variable slip and roughness [0-1], not elastic, high hardness for solid snow
  // slip = 1 <--> roughness=0.0, slip = 0 <--> roughnes=1.0
  Substance Substance::getSnow(float _slip){
    Substance s;
    s.toSnow(_slip);
    return s;

  }

  // variable slip and roughness [0-1], not elastic, high hardness for solid snow
  // slip = 1 <--> roughness=0.0, slip = 0 <--> roughnes=1.0
  void Substance::toSnow(float _slip){
    if(_slip<0) { cerr << "slip is not defined for values<0!" << endl;}
    if(_slip>1) { cerr << "to high slip!" << endl;}
    roughness  = 1.0-_slip;
    hardness   = 40;
    elasticity = 0;
    slip = _slip;
  }

  // no contact points are generated
  Substance Substance::getNoContact(){
    Substance s;
    s.toNoContact();
    return s;
  }

  // collision function that does nothing and prohibits further treatment of collision event.
  int dummyCallBack(dSurfaceParameters& params, GlobalData& globaldata, void *userdata,
                    dContact* contacts, int numContacts,
                    dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2){
    return 0;
  }

  // no contact points are generated
  void Substance::toNoContact(){
    toDefaultSubstance();
    setCollisionCallback(dummyCallBack,0);
  }


  // *** Anisotop friction stuff ****

  struct AnisotropFrictionData {
    double ratio;
    Axis axis;
  };

  static int anisocallback(dSurfaceParameters& params, GlobalData& globaldata, void *userdata,
                           dContact* contacts, int numContacts,
                           dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2){
    // The other substance should not have a callback itself,
    //   because then we don't know. It could be a IR sensor for example,
    //   so we just behave as we would be a normal substance
    if(s2.callback) return 1;

    AnisotropFrictionData* data = (AnisotropFrictionData*)userdata;
    assert(data && "anisocallback does not have correct userdata!");

    // we have to set the vectors in contacts
    osg::Matrix pose = osgPose(dGeomGetPosition(o1), dGeomGetRotation(o1));
    Pos objectaxis = data->axis*pose;

    for(int i=0; i< numContacts; i++){
      Pos normal(contacts[i].geom.normal);
      Pos dir = objectaxis^normal;
      if(dir.isNaN() || dir.length2()<0.1){ // the collision is in the perpendicular direction
        return 1; // do normal friction.
      } else {
        dir.normalize();
        contacts[i].fdir1[0]=dir.x();
        contacts[i].fdir1[1]=dir.y();
        contacts[i].fdir1[2]=dir.z();
      }
    }

    // calc default params
    Substance::getSurfaceParams(params, s1,s2,globaldata.odeConfig.simStepSize);
    // set new friction parameters
    params.mu2=params.mu*(data->ratio);
    params.mode |= dContactMu2 | dContactFDir1;

    return 2;
  }

  void Substance::toAnisotropFriction(double _ratio, const Axis& _axis){
    // this is a memory leak but we cannot circumvent it
    AnisotropFrictionData* data = new AnisotropFrictionData;
    data->ratio = _ratio;
    data->axis  = _axis;
    setCollisionCallback(anisocallback,data);
  }

  // ***  end anisotropfriction stuff;


  DebugSubstance::DebugSubstance(){
    setCollisionCallback(&dbg_output, 0);
  }

  DebugSubstance::DebugSubstance( float roughness, float slip,
                                  float hardness, float elasticity):
    Substance(roughness, slip, hardness, elasticity) {
    setCollisionCallback(dbg_output, 0 );

  }

  int DebugSubstance::dbg_output(dSurfaceParameters& params, GlobalData& globaldata,
                             void *userdata, dContact* contacts, int numContacts,
                             dGeomID o1, dGeomID o2,
                             const Substance& s1, const Substance& s2){
    dSurfaceParameters sp;
    getSurfaceParams(sp, s1, s2, globaldata.odeConfig.simStepSize);
    printSurfaceParams(sp);

    return 1;
  }



};
