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
 *                                                                         *
 *   $Log$
 *   Revision 1.20.4.5  2006-02-01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.20.4.4  2006/01/18 09:55:54  martius
 *   created was uninitialised
 *
 *   Revision 1.20.4.3  2005/12/30 22:53:46  martius
 *   removed parentspace init because done in oderobot
 *
 *   Revision 1.20.4.2  2005/12/29 16:45:58  martius
 *   does not inherit from Roboter
 *   moved to osg
 *
 *   Revision 1.20.4.1  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.20  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/

#include "schlange.h"

namespace lpzrobots {

  Schlange::Schlange ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		       const SchlangeConf& conf, const char* name )
    : OdeRobot( odeHandle, osgHandle, name), conf(conf) {

    // prepare name;
    Configurable::insertCVSInfo(this->name, "$RCSfile$", 
				"$Revision$");    	
    created=false;
  }
	
  Schlange::~Schlange()
  {  
    if(created) destroy();
  }

       
  void Schlange::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)    
    create(pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.segmDia/2))); 
  }

  void Schlange::update(){
    assert(created); // robot must exist
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }
  }

  void Schlange::doInternalStuff(const GlobalData& global){
    if(created){
      // mycallback is called for internal collisions! Only once per step
      dSpaceCollide(odeHandle.space, this, mycallback);
    }
  }

  void Schlange::mycallback(void *data, dGeomID o1, dGeomID o2)
  {
    Schlange* me = (Schlange*) data;
    int i=0;
    int o1_index= -1;
    int o2_index= -1;
    for (vector<Primitive*>::iterator n = me->objects.begin(); n!= me->objects.end(); n++, i++){      
      if( (*n)->getGeom() == o1)
	o1_index=i;
      if( (*n)->getGeom() == o2)
	o2_index=i;
    }

    if(o1_index >= 0 && o2_index >= 0 && abs(o1_index - o2_index) > 1){
      // internal collisions
      int i,n;  
      const int N = 10;
      dContact contact[N];  
      n = dCollide (o1, o2, N, &contact[0].geom, sizeof(dContact));	  
      for (i=0; i<n; i++) {
	contact[i].surface.mode = 0;
	contact[i].surface.mu = 0;
	contact[i].surface.mu2 = 0;
	dJointID c = dJointCreateContact( me->odeHandle.world, me->odeHandle.jointGroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;     
      }
    }
  } 
  

  /**
   *This is the collision handling function for snake robots.
   *This overwrides the function collisionCallback of the class robot.
   *@param data
   *@param o1 first geometrical object, which has taken part in the collision
   *@param o2 second geometrical object, which has taken part in the collision
   *@return true if the collision was threated  by the robot, false if not
   **/
  bool Schlange::collisionCallback(void *data, dGeomID o1, dGeomID o2)
  {
    //checks if one of the collision objects is part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space ){
      int i,n;  
      const int N = 20;
      dContact contact[N];
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++){
	//      contact[i].surface.mode = dContactMu2 | dContactSlip1 | dContactSlip2 |
	//	dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |	
	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.slip1 = 0.001;
	contact[i].surface.slip2 = 0.001;
	contact[i].surface.mu = conf.frictionGround; //*10;
	//      contact[i].surface.mu2 = conf.frictionGround;
	contact[i].surface.soft_erp = 0.9;
	contact[i].surface.soft_cfm = 0.001;
	
	dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)); 
      }
      return true;
    }
    return false;
  }



  /** The list of all parameters with there value as allocated lists.
      @param keylist,vallist will be allocated with malloc (free it after use!)
      @return length of the lists
  */
  Configurable::paramlist Schlange::getParamList() const{
    paramlist list;
    list += pair<paramkey, paramval> (string("frictionground"), conf.frictionGround);
    list += pair<paramkey, paramval> (string("frictionjoint"), conf.frictionJoint);
    list += pair<paramkey, paramval> (string("motorpower"),   conf.motorPower);
    list += pair<paramkey, paramval> (string("sensorfactor"), conf.sensorFactor);
    return list;
  }
  
  
  Configurable::paramval Schlange::getParam(const paramkey& key) const{    
    if(key == "frictionground") return conf.frictionGround; 
    else if(key == "frictionjoint") return conf.frictionJoint; 
    else if(key == "motorpower") return conf.motorPower; 
    else if(key == "sensorfactor") return conf.sensorFactor; 
    else  return Configurable::getParam(key) ;
  }
  
  bool Schlange::setParam(const paramkey& key, paramval val){    
    if(key == "frictionground") conf.frictionGround = val; 
    else if(key == "frictionjoint") conf.frictionJoint = val; 
    else if(key == "motorpower") conf.motorPower = val; 
    else if(key == "sensorfactor") conf.sensorFactor = val; 
    else return Configurable::setParam(key, val);
    return true;
  }
  
  
  int Schlange::getSegmentsPosition(vector<Position> &poslist){
    assert(created);
    for(int n = 0; n < conf.segmNumber; n++){
      Pos p(objects[n]->getPosition());
      poslist.push_back(p.toPosition());
    }
    return conf.segmNumber;    
  }



  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void Schlange::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }
    
    odeHandle.space = dSimpleSpaceCreate (parentspace);
	
    int half = conf.segmNumber/2;
    for ( int n = 0; n < conf.segmNumber; n++ ) {
      Primitive* p = new Capsule(conf.segmDia/2 , conf.segmLength);
      p->init(odeHandle, conf.segmMass, osgHandle);    
      p->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) *
		 osg::Matrix::translate((n-half)*conf.segmLength, 0 , conf.segmDia/2) * 
		 pose);
      p->getOSGPrimitive()->setTexture("Images/wood.rgb");
      objects.push_back(p);
    }
    
    created=true;
  }; 


  /** destroys vehicle and space
   */
  void Schlange::destroy(){
    if (created){
      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
	if(*i) delete *i;
      }
      objects.clear();
      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
	if(*i) delete *i;
      }
      joints.clear();
      dSpaceDestroy(odeHandle.space);
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
// 				 dBodyGetPositionAll ( objektliste[0].body , 1 ) , 
// 				 dBodyGetPositionAll ( objektliste[0].body , 2 ) , 
// 				 dBodyGetPositionAll ( objektliste[0].body , 3 ) ); 
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



