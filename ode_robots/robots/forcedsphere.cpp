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
 *   Revision 1.6  2006-08-08 17:04:46  martius
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

  ForcedSphere::ForcedSphere ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		 const char* name, const ForcedSphereConf& conf)
    : OdeRobot( odeHandle, osgHandle, name, "$Id$" ),
      conf(conf)
  {
    created = false;
    object[0] = 0;    
  }
	
  ForcedSphere::~ForcedSphere()
  {
    destroy(); 
  }

  void ForcedSphere::update()
  {
    if(object[0]) object[0]->update();
  }
  
  int ForcedSphere::getSensors ( sensor* sensors, int sensornumber )
  {  
    int len=0;
    for(list<Sensor*>::iterator i = conf.sensors.begin(); i != conf.sensors.end(); i++){
      len += (*i)->get(sensors+len, sensornumber-len);
    }
    return len;
  }

  void ForcedSphere::setMotors ( const motor* motors, int motornumber ) {
    if (motornumber==2){
      dBodyAddForce(object[0]->getBody(), motors[0]*conf.max_force, motors[1]*conf.max_force, 0);
    }
  }


  void ForcedSphere::place(const osg::Matrix& pose){
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.radius)); 
    create(p2);    
  };


  void ForcedSphere::doInternalStuff(const GlobalData& global){
  }

  bool ForcedSphere::collisionCallback(void *data, dGeomID o1, dGeomID o2) {
    return false; // let the standard routine do it for us
  }


  int ForcedSphere::getMotorNumber(){
    return 2;
  }

  int ForcedSphere::getSensorNumber() {
    int s = 0;
    for(list<Sensor*>::iterator i = conf.sensors.begin(); i != conf.sensors.end(); i++){
      s += (*i)->getSensorNumber();
    }
    return s;
  }


  void ForcedSphere::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    object[0] = new Sphere(conf.radius);
    object[0]->init(odeHandle, conf.radius, osgHandle);
    object[0]->setPose(pose);    
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

