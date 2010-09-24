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
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2010-09-24 13:36:12  martius
 *   new substance to do direction dependent friction
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __ANISOTROPFRICTION_H
#define __ANISOTROPFRICTION_H

namespace lpzrobots {

/** A substance with anisotrop friction (direction dependence) along one axis.
    The friction along a given axis (local to the body)
    has a different friction coefficient.
    This mimics scales of snakes.
 */
class AnisotropFriction : public Substance {
public:
  // we cannot have any member variables since they do not fit into the substance object in OdeHandle
  struct AnisotropFrictionData {
    double ratio;
    Axis axis;
  };  

  /** Constructs a substance with anisotrop friction.
      The friction along the given axis is ratio*friction in the other directions
   */
  AnisotropFriction(double _ratio, Axis _axis=Axis(0,0,1))
    : Substance() {
    // this is a memory leak but we cannot circumvent it
    AnisotropFrictionData* data = new AnisotropFrictionData; 
    data->ratio = _ratio; 
    data->axis  = _axis;
    setCollisionCallback(anisocallback,data);
  }

  
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
    getSurfaceParams(params, s1,s2,globaldata.odeConfig.simStepSize);
    // set new friction parameters
    params.mu2=params.mu*(data->ratio); 
    params.mode |= dContactMu2 | dContactFDir1;
    
    return 2;
  }
  
};


}

#endif


