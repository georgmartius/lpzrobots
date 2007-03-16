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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2007-03-16 10:50:43  martius
 *   initial version
 *
 *
 *                                                                 *
 ***************************************************************************/

#include "substance.h" 
#include "globaldata.h"
#include <iostream>
using namespace std;

namespace lpzrobots {

  Substance::Substance()
    : callback(0) 
  {
    toDefaultSubstance();
  }

  Substance::Substance( float roughness, float slip, float hardness, float elasticity)
    : roughness(roughness), slip(slip), hardness(hardness), elasticity(elasticity), callback(0)
  {
  }

  void Substance::setCollisionCallback(CollisionCallback func, void* userdata){
    callback = func;
    userdata=0;
  }

  // Combination of two surfaces
  dSurfaceParameters Substance::getSurfaceParams(const Substance& s1, const Substance& s2){
    dSurfaceParameters sp;    
    sp.mu = s1.roughness * s2.roughness;
    //    sp.bounce;
    //    sp.bounce_vel;
    dReal kp   = 100*s1.hardness* s2.hardness / (s1.hardness + s2.hardness);
    double kd1 = (1.00-s1.elasticity);
    double kd2 = (1.00-s2.elasticity);
    dReal kd   = 10*(kd1 * s2.hardness + kd2 * s1.hardness) / (s1.hardness + s2.hardness);
    //  cout << "spring: " << kp << "\t "<<kd << endl;
    sp.soft_erp = kp / ( kp + kd);
    sp.soft_cfm = 1  / (kp + kd);
    //    sp.motion1,motion2;
    sp.slip1=s1.slip + s2.slip;
    sp.slip2=s1.slip + s2.slip;
    if(sp.slip1<0.0001) sp.mode=0;
    else sp.mode = dContactSlip1 | dContactSlip2;
    sp.mode |= dContactSoftERP | dContactSoftCFM | dContactApprox1;
    
    return sp;
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

  // very hard and elastic 
  Substance Substance::getMetal(float _roughness){
    Substance s;
    s.toMetal(_roughness);
    return s;
  }
  // very hard and elastic
  void Substance::toMetal(float _roughness){
    roughness = _roughness;
    hardness   = 20;
    elasticity = 1.0;
    slip       = 0.01;
  }
  
  Substance Substance::getRubber(float _hardness){
    Substance s;
    s.toRubber(_hardness);
    return s;
  }

  // high roughness, no slip, a bit elastic, hardness : [0.1-0.5]
  void Substance::toRubber(float _hardness){
    roughness  = 3;
    hardness   = _hardness;
    elasticity = 0.3;
    slip       = 0.0;
  }
  
  // medium slip, a bit elastic, medium hardness, roughness [0.5-1]
  Substance Substance::getPlastic(float _roughness){
    Substance s;
    s.toPlastic(_roughness);
    return s;

  }

  // medium slip, a bit elastic, medium hardness, roughness [0.5-1]
  void Substance::toPlastic(float _roughness){
    roughness  = _roughness;
    hardness   = 5;
    elasticity = 0.5;
    slip       = 0.01;
  }

  // large slip, not elastic, low hardness [0.01-0.3], high roughness
  Substance Substance::getFoam(float _hardness){
    Substance s;
    s.toFoam(_hardness);
    return s;

  }

  // large slip, not elastic, low hardness [0.01-0.3], high roughness
  void Substance::toFoam(float _hardness){
    roughness  = 2;
    hardness   = _hardness;
    elasticity = 0;
    slip       = 0.1;
  }

  
}
