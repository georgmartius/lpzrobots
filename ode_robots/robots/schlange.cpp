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
 *   Revision 1.35  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.34  2011/05/30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.33  2010/09/30 17:12:29  martius
 *   added anisotrop friction to schlange
 *
 *   Revision 1.32  2010/01/26 09:55:26  martius
 *   new collision model
 *
 *   Revision 1.31  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.30  2008/05/07 16:45:52  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.29  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.28  2007/09/06 18:48:00  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.27  2006/10/20 14:25:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.26  2006/09/21 16:17:18  der
 *   *** empty log message ***
 *
 *   Revision 1.25  2006/09/21 11:44:38  martius
 *   less hard collisions and more contact points
 *
 *   Revision 1.24  2006/09/20 12:56:16  martius
 *   Snakes have CreateSegment
 *
 *   Revision 1.23  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.22  2006/07/14 13:52:01  der
 *   setheadcolor
 *
 *   Revision 1.20.4.10  2006/06/29 16:39:56  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.20.4.9  2006/06/25 16:57:14  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.20.4.8  2006/05/23 13:39:02  robot3
 *   setting color to blue in init removed
 *
 *   Revision 1.20.4.7  2006/05/19 09:03:50  der
 *   -setTexture and setHeadTexture added
 *   -uses now whitemetal texture
 *
 *   Revision 1.20.4.6  2006/02/23 18:05:04  martius
 *   friction with angularmotor
 *
 *   Revision 1.20.4.5  2006/02/01 18:33:40  martius
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
#include "osgprimitive.h"

using namespace std;

namespace lpzrobots {

  Schlange::Schlange ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		       const SchlangeConf& conf, const std::string& name, const std::string& revision)
    : OdeRobot( odeHandle, osgHandle, name, revision), conf(conf) {
    
    addParameter("frictionground",&this->conf.frictionGround,0,2);
    addParameter("motorpower", &this->conf.motorPower,0,20);
    addParameter("sensorfactor", &this->conf.sensorFactor,0,5);
    addParameter("frictionjoint",&this->conf.frictionJoint,0,5);   
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

  void Schlange::doInternalStuff(GlobalData& global){
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

    if(conf.frictionRatio != 1)
      odeHandle.substance.toAnisotropFriction(conf.frictionRatio, Axis(0,0,1));

    for ( int n = 0; n < conf.segmNumber; n++ ) {
      Primitive* p;
      p = createSegment(n);
      p->setPose(p->getPose() * osg::Matrix::rotate(M_PI/2, 0, 1, 0)
		 * osg::Matrix::translate((n-half)*conf.segmLength, 0 , conf.segmDia/2) * pose);

      objects.push_back(p);

    }

//       if (n==-1* conf.segmNumber/2) {
// 		p = new Box(conf.segmLength*1.8,conf.segmLength*.8, conf.segmLength*1);
// 		//p = new Capsule(conf.segmDia*2 , conf.segmLength);
// 	p->init(odeHandle, conf.segmMass*2, osgHandle);    
//       }
//       //    else {
//       //if(n==0 || n== conf.segmNumber){ 

//       //  p = new Box(conf.segmLength,conf.segmLength*2, conf.segmLength);
//       //  p->init(odeHandle, conf.segmMass*2, osgHandle);
//       //	}	
//       else{


// 	if(n==-1/*== 0 | n== conf.segmNumber-1*/) { 
// 	  p = new Capsule(conf.segmDia*.8/*2.8*/ , conf.segmLength*1); 
// 	// p = new Box(conf.segmLength*.3,conf.segmLength, conf.segmLength*.9);
//         p->init(odeHandle, conf.segmMass*4, osgHandle);}
// 	else{
//  p = new Capsule(conf.segmDia*.8 , conf.segmLength); 
//  //	p = new Box(conf.segmLength*.3,conf.segmLength*0.3, conf.segmLength*1.0);
//  p->init(odeHandle, conf.segmMass, osgHandle); 
// 	} }  
// 	//	else {

//       //      p->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) *
//       //		 osg::Matrix::translate((n-half)*conf.segmLength*(1+((double)n)/10), 0 , conf.segmDia/2) * 
//       //		 pose);
//       p->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) *
// 	     //  p->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) *
// 		 osg::Matrix::translate((n-half)*conf.segmLength, 0 , conf.segmDia/2) * 
// 		 pose);
//       //      p->getOSGPrimitive()->setTexture("Images/wood.rgb");
//       //  p->getOSGPrimitive()->setTexture("Images/tire.rgb");
//       p->getOSGPrimitive()->setTexture("Images/whitemetal_farbig.rgb");
//       //      p->getOSGPrimitive()->setColor(Color(0.0f,0.0f,1.0f,0.2f));
      
//       objects.push_back(p);
// 	}
    
    created=true;
  }; 

  Primitive* Schlange::createSegment(int index){
    Primitive* p;
    p = new Capsule(conf.segmDia * 0.8, conf.segmLength);     
    p->setTexture("Images/whitemetal_farbig_small.rgb");    
    p->init(odeHandle, conf.segmMass, osgHandle);         
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



