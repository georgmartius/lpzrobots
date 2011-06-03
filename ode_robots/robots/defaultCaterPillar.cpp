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
 *                                                                         *
 *   $Log$
 *   Revision 1.9  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.8  2011/05/30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.7  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.6  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.5  2007/11/07 13:21:15  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.4  2007/09/06 18:47:59  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.3  2006/08/04 15:07:27  martius
 *   documentation
 *
 *   Revision 1.2  2006/07/14 12:23:39  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.12  2006/06/29 16:39:55  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.1.2.11  2006/06/25 16:57:12  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.10  2006/06/13 04:24:22  robot5
 *   Separated annular and linear placement in different robots.
 *   See "wheelie" for annular segment placement.
 *
 *   Revision 1.1.2.9  2006/05/29 22:05:52  martius
 *   ode_robot/Makefile should be called without argument to ensure proper build of libselforg
 *
 *   Revision 1.1.2.8  2006/05/29 20:28:43  robot5
 *   Annular placement of segments.
 *
 *   Revision 1.1.2.7  2006/05/22 14:19:11  robot5
 *   Added variable conf.firstJoint to represent the first slider type in the alternating sequence.
 *   Code cleaning.
 *
 *   Revision 1.1.2.6  2006/05/15 21:11:12  robot5
 *   Using slider and universal joints now (alternating)
 *
 *   Revision 1.1.2.5  2006/05/09 08:47:00  robot3
 *   getSensors() and getMotors() modified
 *
 *   Revision 1.1.2.4  2006/05/09 04:24:34  robot5
 *   *** empty log message ***
 *
 *   Revision 1.1.2.3  2006/04/25 09:03:03  robot3
 *   caterpillar is now represented by a box
 *
 *   Revision 1.1.2.2  2006/04/11 13:27:29  robot3
 *   caterpillar is using now methods from schlangeservo2
 *
 *   Revision 1.1.2.1  2006/04/11 09:28:27  robot3
 *   first version
 *
 *
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

       
  void DefaultCaterPillar::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)    
    create(pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.segmDia/2))); 
  }

  void DefaultCaterPillar::update(){
    assert(created); // robot must exist
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }
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



