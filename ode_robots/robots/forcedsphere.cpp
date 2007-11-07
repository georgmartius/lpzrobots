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
 *   Revision 1.9  2007-11-07 13:21:15  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.8  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.7  2006/08/11 15:44:29  martius
 *   has conf now and arbitrary sensors
 *
 *   Revision 1.6  2006/08/08 17:04:46  martius
 *   added new sensor model
 *
 *   Revision 1.5  2006/07/14 12:23:40  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.4.4.6  2006/07/10 12:05:01  martius
 *   Matrixlib now in selforg
 *   no namespace std in header files
 *
 *   Revision 1.4.4.5  2006/06/25 17:00:31  martius
 *   Id
 *
 *   Revision 1.4.4.4  2006/06/25 16:57:13  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.4.4.3  2006/01/10 22:25:09  martius
 *   moved to osg
 *
 *   Revision 1.1.2.1  2006/01/10 17:15:04  martius
 *   was sphererobotarms
 *   moved to osg
 *
 *   Revision 1.18.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.18.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.18  2005/11/09 13:26:57  martius
 *   irsensorrange
 *
 *   Revision 1.17  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/

#include <assert.h>
#include <selforg/matrix.h>
#include "primitive.h"
#include "forcedsphere.h"
#include "mathutils.h"

using namespace osg;
using namespace std;

namespace lpzrobots {


  ForcedSphereConf::ForcedSphereConf() {}

  ForcedSphereConf::~ForcedSphereConf(){}

  void ForcedSphereConf::destroy(){
    for(list<Sensor*>::iterator i = sensors.begin(); i != sensors.end(); i++){
      if(*i) delete *i;
    }    
    sensors.clear();
  }



  ForcedSphere::ForcedSphere ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			       const ForcedSphereConf& conf, const std::string& name)
    : OdeRobot( odeHandle, osgHandle, 
		name, "$Id$" ), conf(conf)
  {
    created = false;
    object[0] = 0;    
  }
	
  ForcedSphere::~ForcedSphere()
  {
    destroy(); 
    conf.destroy();
  }

  void ForcedSphere::update()
  {
    if(object[0]) object[0]->update();
  }
  
  int ForcedSphere::getSensors ( sensor* sensors, int sensornumber )
  {  
    assert(created);
    int len=0;
    FOREACH(list<Sensor*>, conf.sensors, i){
      len += (*i)->get(sensors+len, sensornumber-len);
    }
    return len;
  }

  void ForcedSphere::setMotors ( const motor* motors, int motornumber ) {
    assert(created);
    if (motornumber==getMotorNumber()){      
      int i=0;
      double x = (conf.drivenDimensions & X) ? motors[i++] : 0;
      double y = (conf.drivenDimensions & Y) ? motors[i++] : 0;
      double z = (conf.drivenDimensions & Z) ? motors[i++] : 0;
      if(!conf.speedDriven)
	dBodyAddForce(object[0]->getBody(), x*conf.maxForce, y*conf.maxForce, z*conf.maxForce);
      else{
	Position nom;
	nom.x=x; nom.y=y; nom.z=z;
        Position diff = (nom*conf.maxSpeed-getSpeed())*0.5*conf.maxForce;
	dBodyAddForce(object[0]->getBody(), diff.x, diff.y, diff.z);
	
      }
      int len=motornumber-i;
      FOREACH(list<Motor*>, conf.motors, m){
	int l=(*m)->set(motors+i,len);
	i+=l;
	len-=l;	
      }
      
    }
  }


  void ForcedSphere::place(const osg::Matrix& pose){
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.radius)); 
    create(p2);    
  };


  void ForcedSphere::doInternalStuff(GlobalData& global){
    FOREACH(list<Motor*>, conf.motors, i){
      (*i)->act(global);
    }
    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->sense(global);
    }

    // slow down rotation around z axis.
    dBodyID b = getMainPrimitive()->getBody();
    const double* vel = dBodyGetAngularVel( b);
    if(fabs(vel[2])>0.05){
      dBodyAddTorque ( b , 0 , 0 , -0.1*conf.maxForce*vel[2] );
    }



    

  }

  bool ForcedSphere::collisionCallback(void *data, dGeomID o1, dGeomID o2) {
    return false; // let the standard routine do it for us
  }


  int ForcedSphere::getMotorNumber(){    
    int s = 0;
    FOREACHC(list<Motor*>, conf.motors, i){
      s += (*i)->getMotorNumber();
    }
    return s + (conf.drivenDimensions & X) + ((conf.drivenDimensions & Y) >> 1) + 
      ((conf.drivenDimensions & Z) >> 2);
  }

  int ForcedSphere::getSensorNumber() {
    int s = 0;
    FOREACHC(list<Sensor*>, conf.sensors, i){
      s += (*i)->getSensorNumber();
    }
    return s;
  }


  void ForcedSphere::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }
    
    Transform* f;
    if(conf.cylinderBody){
      object[0] = new Cylinder(conf.radius,conf.radius/2);
      Primitive* core = new Box(conf.radius/1.5,conf.radius/1.5,conf.radius/2.5);
      f = new Transform(object[0], core, osg::Matrix::translate(0,0,0));
    }
    else
      object[0] = new Sphere(conf.radius);
    object[0]->init(odeHandle, conf.radius*conf.radius, osgHandle);
    if(conf.cylinderBody){
      f->init(odeHandle, 0, osgHandle);
    }
    object[0]->setPose(pose);    
    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->init(object[0]);
    }
    FOREACH(list<Motor*>, conf.motors, i){
      (*i)->init(object[0]);
    }
    created = true;
  }


  void ForcedSphere::destroy(){
    if (created){
      for (int i=0; i<1; i++){
	if(object[i]) delete object[i];
      }
    }
    created=false;
  }

}

